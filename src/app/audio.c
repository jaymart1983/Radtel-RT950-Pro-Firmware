/*
 * audio.c - System audio feedback for RT-950 Pro
 *
 * Non-blocking beep/tone system: caller starts a tone, audio_poll()
 * auto-stops it after the requested duration.  Uses the DAC sine-tone
 * generator (dac_audio_play_tone / dac_audio_stop).
 *
 * Audio routing GPIOs (V12 hardware-verified):
 *   PC12 (BEEP_SW)  - LOW routes DAC to speaker; HIGH routes BK4829/SI4732
 *   PB8  (AMP_EN)   - HIGH enables audio amplifier       @ 0x08006574
 *   PE4  (AMP_PWR)  - HIGH powers audio amplifier rail
 *   PE1  (SPK_MUTE) - HIGH mutes speaker (used during TX) @ 0x08019254
 *
 * OEM beep_play @ 0x08003808: CLRs PC12 before tone (DAC path), SETs after.
 * OEM spk_mute_on_ptt @ 0x08019254: SETs PE1 during TX.
 * OEM spk_unmute @ 0x0801929C: CLRs PE1 when returning to RX.
 *
 * Multi-tone sequences are driven by a step table: each step has a
 * frequency (tenths-of-Hz) and duration (ms).  audio_poll() advances
 * through the steps automatically.
 */

#include "app/audio.h"
#include "drivers/dac_audio.h"
#include "drivers/bk4829.h"
#include "drivers/gpio.h"
#include "rt950_pinmap.h"
#include "app/settings.h"

extern uint32_t get_tick(void);

/* Tone state */

static uint32_t tone_end_ms;
static uint8_t  tone_active;

/* Multi-tone sequence state */

typedef struct {
    uint16_t freq_x10;
    uint16_t duration_ms;
} tone_step_t;

#define SEQ_MAX_STEPS  8

static const tone_step_t *seq_steps;
static uint8_t  seq_count;
static uint8_t  seq_index;
static uint8_t  seq_active;

/* ========================================================================
 *  OEM spk_unmute sequence (V0.27 @ 0x0801929C):
 *
 *  Phase 0→1 (entering beep - called BEFORE any tone playback):
 *    rf_relay_switch(band, 0)           → ALL RF relays IDLE (PE7/12/13/14=L, PB0/1=L)
 *    bk4829_reg30_set_mode(0)           → R30=0x0000 (STANDBY) on BOTH chips
 *    bk4829_audio_mode_switch(1)        → R37=0x1D00 (beep filter) on BOTH chips
 *    + R47/R48 set to beep AF path
 *
 *  Phase 1→0 (leaving beep - called AFTER tone finishes):
 *    bk4829_audio_mode_switch(0)        → R37=0x9D1F (RX filter) on BOTH chips
 *    bk4829_reg30_set_mode(1)           → R30=0xBFF1 (RX mode) on BOTH chips
 *    rf_relay_switch(band, 1)           → Relays back to RX
 *
 *  CRITICAL: BK4829 R47/R48 must be zeroed to prevent AF output
 *  from loading the analog bus and blocking DAC audio.
 *  PC12=LOW routes DAC to speaker (V12 hardware-verified).
 * ======================================================================== */

/* Set all RF relay pins to IDLE (LOW) - OEM rf_relay_switch mode 0 */
static void rf_relays_idle(void)
{
    gpio_clear_pin(GPIOE, GPIO_PIN_7);      /* PE7  U3T_EN */
    gpio_clear_pin(GPIOE, GPIO_PIN_12);     /* PE12 U3R_EN */
    gpio_clear_pin(GPIOE, GPIO_PIN_13);     /* PE13 U6R_EN */
    gpio_clear_pin(GPIOE, GPIO_PIN_14);     /* PE14 SW3T */
    gpio_clear_pin(GPIOB, GPIO_PIN_0);      /* PB0  V3R */
    gpio_clear_pin(GPIOB, GPIO_PIN_1);      /* PB1  V3T */
}

/* Restore RF relays to RX mode (simplified - VHF default) */
static void rf_relays_rx(void)
{
    gpio_clear_pin(GPIOE, GPIO_PIN_7);      /* PE7  TX off */
    gpio_clear_pin(GPIOB, GPIO_PIN_1);      /* PB1  VHF TX off */
    gpio_clear_pin(GPIOE, GPIO_PIN_14);     /* PE14 TX switch off */
    gpio_set_pin(GPIOE, GPIO_PIN_12);       /* PE12 VHF RX on */
    gpio_set_pin(GPIOB, GPIO_PIN_0);        /* PB0  VHF LNA on */
}

/* Enter beep mode - OEM spk_unmute phase 0→1 */
static void audio_path_enable(void)
{
    /* Step 1: RF relays IDLE - isolate DAC from RF coupling */
    rf_relays_idle();

    /* Step 2: BK4829 STANDBY on BOTH chips - OEM R30=0x0000 */
    bk4829_standby(BK4829_CHIP0);
    bk4829_standby(BK4829_CHIP1);

    /* Step 3: Kill BK4829 AF output to prevent bus loading.
     * V12 proved BK4829 AF output loads the analog bus even in standby
     * unless R47/R48 are explicitly zeroed. */
    bk4829_write_reg(BK4829_CHIP0, 0x47, 0x0000);
    bk4829_write_reg(BK4829_CHIP1, 0x47, 0x0000);
    bk4829_write_reg(BK4829_CHIP0, 0x48, 0x0000);
    bk4829_write_reg(BK4829_CHIP1, 0x48, 0x0000);

    /* Step 4: Enable amplifier - PB8 HIGH */
    gpio_config_pin(AMP_EN_PORT, AMP_EN_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(AMP_EN_PORT, AMP_EN_PIN);      /* PB8 HIGH - amp ON */

    /* Step 5: PC12 LOW - routes DAC (PA4) to amplifier input.
     * V12 hardware test confirmed: PC12=LOW = DAC path, HIGH = BK4829 path */
    gpio_clear_pin(BEEP_SW_PORT, BEEP_SW_PIN);   /* PC12 LOW - DAC path */
}

/* Leave beep mode - OEM spk_unmute phase 1→0 */
static void audio_path_disable(void)
{
    /* Step 1: AMP off - OEM audio_state_machine state 0 */
    gpio_clear_pin(AMP_EN_PORT, AMP_EN_PIN);     /* PB8 LOW - amp OFF */

    /* Step 2: PC12 HIGH - switch back to BK4829/radio path */
    gpio_set_pin(BEEP_SW_PORT, BEEP_SW_PIN);     /* PC12 HIGH - radio path */

    /* Step 3: Restore BK4829 AF path for RX - R47/R48 */
    bk4829_set_af_rx(BK4829_CHIP0);
    bk4829_set_af_rx(BK4829_CHIP1);

    /* Step 4: Restore BK4829 to RX mode on BOTH chips - R30=0xBFF1 */
    bk4829_enable_rx(BK4829_CHIP0);
    bk4829_enable_rx(BK4829_CHIP1);

    /* Step 5: RF relays back to RX */
    rf_relays_rx();
}

static void tone_start(uint16_t freq_x10, uint16_t duration_ms)
{
    audio_path_enable();
    dac_audio_play_tone(freq_x10);
    tone_end_ms = get_tick() + duration_ms;
    tone_active = 1;
    seq_active = 0;
}

/* Start a multi-tone sequence */
static void seq_start(const tone_step_t *steps, uint8_t count)
{
    if (!settings_get()->beep || count == 0) return;
    seq_steps = steps;
    seq_count = count;
    seq_index = 0;
    seq_active = 1;
    if (steps[0].freq_x10 > 0) {
        audio_path_enable();
        dac_audio_play_tone(steps[0].freq_x10);
    } else {
        dac_audio_stop();
    }
    tone_end_ms = get_tick() + steps[0].duration_ms;
    tone_active = 1;
}

/* Public API */

void audio_init(void)
{
    tone_active = 0;
    tone_end_ms = 0;
    seq_active = 0;

    /* NOTE: dac_audio_init() is deliberately NOT called here.
     *
     * The DAC, TIM6 and DMA2 clocks must be up before any tone will sound --
     * without them every dac_audio_play_tone() writes to dead peripherals and
     * all three registers read back zero. hw_init() already calls
     * dac_audio_init() at the right point in the boot order, so the normal
     * firmware is fine.
     *
     * Callers that skip hw_init() -- the HW_TEST builds -- must call
     * dac_audio_init() themselves before expecting sound. */
}

void audio_beep(void)
{
    if (!settings_get()->beep) return;
    tone_start(10000, 50);   /* 1000.0 Hz x 50 ms */
}

void audio_error_beep(void)
{
    if (!settings_get()->beep) return;
    tone_start(5000, 100);   /* 500.0 Hz x 100 ms */
}

void audio_beep_freq(uint16_t freq_x10, uint16_t duration_ms)
{
    if (!settings_get()->beep) return;
    tone_start(freq_x10, duration_ms);
}

static const uint16_t roger_freq_x10[4] = { 10000, 14500, 17500, 21000 };

void audio_roger_beep(uint8_t freq_sel)
{
    if (freq_sel > 3) freq_sel = 0;
    tone_start(roger_freq_x10[freq_sel], 200);
}

/* Power-on chime: ascending 3-tone (800 -> 1200 -> 1600 Hz) */
static const tone_step_t power_on_seq[] = {
    { 8000,  80 },  /* 800 Hz, 80 ms */
    {    0,  30 },  /* silence gap */
    { 12000, 80 },  /* 1200 Hz, 80 ms */
    {    0,  30 },  /* silence gap */
    { 16000, 120 }, /* 1600 Hz, 120 ms */
};

void audio_power_on(void)
{
    seq_start(power_on_seq, 5);
}

/* Alert: rapid 2-tone alternation (1200/800 Hz, 3 cycles) */
static const tone_step_t alert_seq[] = {
    { 12000, 100 }, { 8000, 100 },
    { 12000, 100 }, { 8000, 100 },
    { 12000, 100 }, { 8000, 100 },
};

void audio_alert(void)
{
    seq_start(alert_seq, 6);
}

/* Scan hit: quick rising chirp (600 -> 1000 Hz) */
static const tone_step_t scan_hit_seq[] = {
    { 6000,  60 },
    { 8000,  60 },
    { 10000, 80 },
};

void audio_scan_hit(void)
{
    seq_start(scan_hit_seq, 3);
}

/* Poll (call from super_loop >=100 Hz) */

void audio_poll(void)
{
    if (!tone_active) return;

    if (get_tick() >= tone_end_ms) {
        if (seq_active && seq_index + 1 < seq_count) {
            seq_index++;
            const tone_step_t *s = &seq_steps[seq_index];
            if (s->freq_x10 > 0) {
                audio_path_enable();
                dac_audio_play_tone(s->freq_x10);
            } else {
                dac_audio_stop();
                /* Keep path enabled during silence gaps - OEM does not
                 * toggle PC12/PB8 between sequence steps. */
            }
            tone_end_ms = get_tick() + s->duration_ms;
        } else {
            dac_audio_stop();
            audio_path_disable();
            tone_active = 0;
            seq_active = 0;
        }
    }
}

/* ========================================================================
 *  Speaker mute / unmute - for TX path control.
 *
 *  OEM spk_mute_on_ptt @ 0x08019254: SETs PE1 (mute speaker) + PB8 (mic on)
 *  OEM spk_unmute       @ 0x0801929C: CLRs PE1 (unmute) + CLRs PB8 (mic off)
 *
 *  PE1 polarity: HIGH = muted, LOW = unmuted.
 *  Called by radio TX/RX transitions (not by beep/tone path).
 * ======================================================================== */

void audio_speaker_mute(void)
{
    gpio_set_pin(SPK_MUTE_PORT, SPK_MUTE_PIN);     /* PE1 HIGH = muted */
}

void audio_speaker_unmute(void)
{
    gpio_clear_pin(SPK_MUTE_PORT, SPK_MUTE_PIN);   /* PE1 LOW = unmuted */
}
