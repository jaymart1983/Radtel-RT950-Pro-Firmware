/*
 * encoder.c - Rotary encoder driver for the RT-950 Pro
 *
 * Quadrature decoding on PB4 (channel A) / PB5 (channel B).
 * Uses a 4-state Gray-code table for reliable direction detection
 * with built-in debounce, matching the OEM firmware approach
 * (V0.27 @ fw 0x0800D710) which uses a state variable + debounce counter.
 *
 * The encoder produces one detent per full quadrature cycle (4 edges).
 * We track all 4 transitions and emit +1/-1 only on a valid complete
 * step, which inherently filters noise.
 */

#include "app/encoder.h"
#include "drivers/gpio.h"
#include "rt950_pinmap.h"

/*
 * Quadrature state machine.
 *
 * 2-bit state = (B << 1) | A, sampled from PB5:PB4.
 * The Gray-code sequence for CW rotation is: 00 -> 01 -> 11 -> 10 -> 00
 * For CCW: 00 -> 10 -> 11 -> 01 -> 00
 *
 * We use a lookup table indexed by (prev_state << 2 | new_state) to
 * get direction: +1 CW, -1 CCW, 0 invalid/no-change.
 */
static const int8_t quad_table[16] = {
    /* prev\new  00   01   10   11  */
    /* 00 */      0,  +1,  -1,   0,
    /* 01 */     -1,   0,   0,  +1,
    /* 10 */     +1,   0,   0,  -1,
    /* 11 */      0,  -1,  +1,   0
};

#define ENC_DEBOUNCE_THRESHOLD  2   /* consecutive identical reads required */

static uint8_t prev_state;          /* last accepted 2-bit Gray state */
/* Quadrature steps per mechanical detent.
 *
 * Measured on the RT-950 Pro: turning the knob one click produces TWO state
 * transitions, not four. Emitting a detent every 4 steps therefore reported one
 * detent per two clicks -- confirmed on hardware, where ten clicks gave five
 * detents while the raw A/B levels alternated cleanly once per click.
 *
 * This is a half-step encoder. Full-quadrature parts give four transitions per
 * detent and would need 4 here, so it is a named constant rather than a magic
 * number: it is a property of the part, not of the algorithm. */
#define ENC_STEPS_PER_DETENT    2

static int8_t  accum;              /* accumulated quadrature steps */
static uint8_t debounce_count;     /* consecutive identical raw reads */
static uint8_t debounce_state;     /* state being debounced */

/* Read encoder pins and return 2-bit state: bit0 = A (PB4), bit1 = B (PB5) */
static inline uint8_t read_encoder(void)
{
    uint8_t a = gpio_read_pin(ENC_A_PORT, ENC_A_PIN);
    uint8_t b = gpio_read_pin(ENC_B_PORT, ENC_B_PIN);
    return (uint8_t)((b << 1) | a);
}

void encoder_init(void)
{
    gpio_enable_clock(ENC_A_PORT);
    /* PB4 and PB5 as floating input */
    gpio_config_pin(ENC_A_PORT, ENC_A_PIN, GPIO_MODE_INPUT, GPIO_CNF_FLOATING);
    gpio_config_pin(ENC_B_PORT, ENC_B_PIN, GPIO_MODE_INPUT, GPIO_CNF_FLOATING);

    prev_state     = read_encoder();
    debounce_state = prev_state;
    debounce_count = 0;
    accum          = 0;
}

int8_t encoder_poll(void)
{
    uint8_t raw = read_encoder();

    /* Debounce: require ENC_DEBOUNCE_THRESHOLD consecutive identical reads */
    if (raw != debounce_state) {
        debounce_state = raw;
        debounce_count = 1;
        return 0;
    }

    debounce_count++;
    if (debounce_count < ENC_DEBOUNCE_THRESHOLD)
        return 0;
    debounce_count = ENC_DEBOUNCE_THRESHOLD;    /* clamp to avoid overflow */

    if (raw == prev_state)
        return 0;

    /* Valid transition - look up direction */
    uint8_t idx = (uint8_t)((prev_state << 2) | raw);
    int8_t dir = quad_table[idx];
    prev_state = raw;

    if (dir == 0)
        return 0;   /* invalid transition (noise or missed step) */

    /* Accumulate quarter-steps; emit detent on every 4th step */
    accum += dir;
    if (accum >= ENC_STEPS_PER_DETENT) {
        accum = 0;
        return +1;  /* CW detent */
    }
    if (accum <= -ENC_STEPS_PER_DETENT) {
        accum = 0;
        return -1;  /* CCW detent */
    }

    return 0;
}
