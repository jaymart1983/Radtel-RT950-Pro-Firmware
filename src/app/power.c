/*
 * power.c - Power management for the RT-950 Pro
 *
 * Battery: ADC2 channel 0 (PA0), 12-bit result shifted to 8-bit for
 * threshold comparison.  Voltage: mV = adc_8bit x 482 / 10.
 *
 * Seven ascending thresholds define 8 battery states.  Threshold[6] is
 * forced to 0xBB (~9.01V) per RE analysis of calibration loader.
 *
 * Auto power-off: idle counter incremented by power_poll() (~1Hz).
 *
 * Hardware power-off: PE0 is the power switch input. A long press
 * (1.5s) triggers a shutdown sequence that saves state, disables
 * peripherals, and releases the PB9 power latch. The hardware
 * regulator then cuts power completely.
 *
 * Backlight: direct GPIO on LCD_BL_PORT / LCD_BL_PIN (PC6).
 */

#include "app/power.h"
#include "app/audio.h"
#include "app/settings.h"
#include "kernel/event.h"
#include "drivers/adc.h"
#include "drivers/gpio.h"
#include "drivers/lcd.h"
#include "drivers/dac_audio.h"
#include "rt950_pinmap.h"
#include "cortex_m4.h"
#include "debug_uart.h"
#include <stddef.h>

/*
 * Default thresholds (ascending, 8-bit ADC scale).
 * These are overwritten at runtime by power_set_thresholds() when
 * calibration data is loaded from flash 0xF200.
 */
static uint8_t batt_thresholds[7] = {
    0x86, 0x8C, 0x92, 0x98, 0x9E, 0xA4, 0xBB
};

static uint16_t auto_off_minutes;
static uint32_t idle_seconds;

/* Low-battery alert: beep every 30s when battery <= BATT_LOW */
#define LOW_BATT_BEEP_INTERVAL  30   /* seconds between warning beeps */
static uint8_t  low_batt_counter;     /* seconds since last beep */
static uint8_t  low_batt_alert;       /* 1 = in low-battery state */

/* Auto keypad lock: lock after 60s of inactivity */
#define AUTO_KEYLOCK_SECONDS  60
static uint32_t keylock_idle_seconds;

/* Power button (PE0) long-press detection.
 * The power switch is a knob/switch - PE0 reads LOW when pressed.
 * We debounce at 50 Hz (called from power_button_poll) and require
 * 75 consecutive LOW reads (~1.5 seconds) for a long press. */
#define PWR_BTN_LONG_PRESS_COUNT  75  /* 75 x 20ms = 1.5 seconds */
static uint16_t pwr_btn_held_count;
static uint8_t  pwr_btn_shutdown_triggered;

extern void delay_ms(uint32_t ms);

/* ========================================================================
 *  power_init - Set up ADC and reset auto-off state.
 *
 *  OEM power-on path @ 0x0801E268 configures PE0 as input (power switch)
 *  and sets PB9 HIGH (power latch hold).
 * ======================================================================== */

void power_init(void)
{
    auto_off_minutes = 0;
    idle_seconds = 0;
    pwr_btn_held_count = 0;
    pwr_btn_shutdown_triggered = 0;

    /* Configure PE0 (power switch) as input with pull-up */
    gpio_config_pin(PWR_SWITCH_PORT, PWR_SWITCH_PIN,
                    GPIO_MODE_INPUT, GPIO_CNF_PULL);
    gpio_set_pin(PWR_SWITCH_PORT, PWR_SWITCH_PIN);  /* enable pull-up */

    /* Force highest threshold to 0xBB per RE analysis */
    batt_thresholds[6] = 0xBB;
}

/* ========================================================================
 *  power_read_battery_raw - 8-bit battery ADC reading.
 *
 *  OEM ADC_Read_PA0 @ 0x0801385C reads channel 0, returns UBFX(DR,4,8).
 *  adc_read_battery() matches this: 12-bit >> 4 = bits [11:4].
 * ======================================================================== */

uint8_t power_read_battery_raw(void)
{
    return adc_read_battery();
}

/* ========================================================================
 *  power_get_battery_level - Map ADC reading to battery_level_t.
 *
 *  Compares 8-bit raw value against 7 ascending thresholds.
 *  Returns BATT_CRITICAL (0) if below threshold[0], up to
 *  BATT_FULL (7) if at or above threshold[6].
 * ======================================================================== */

battery_level_t power_get_battery_level(void)
{
    uint8_t raw = power_read_battery_raw();

    for (uint8_t i = 0; i < 7; i++) {
        if (raw < batt_thresholds[i])
            return (battery_level_t)i;
    }
    return BATT_FULL;
}

/* ========================================================================
 *  power_get_battery_mv - Convert 8-bit ADC to millivolts.
 *
 *  Formula: mV = adc_8bit x 482 / 10
 *  Verified: 0xBB (187) x 482 / 10 = 9013 mV ~ 9.01V
 * ======================================================================== */

uint16_t power_get_battery_mv(void)
{
    uint8_t raw = power_read_battery_raw();
    return (uint16_t)((uint32_t)raw * 482U / 10U);
}

/* ========================================================================
 *  power_set_thresholds - Load thresholds from calibration data.
 *
 *  Accepts 7 ascending 8-bit values.  Threshold[6] is forced to 0xBB
 *  regardless of input, matching OEM calibration loader behavior.
 * ======================================================================== */

void power_set_thresholds(const uint8_t thresholds[7])
{
    for (uint8_t i = 0; i < 7; i++)
        batt_thresholds[i] = thresholds[i];

    /* OEM forces the top threshold */
    batt_thresholds[6] = 0xBB;
}

/* ========================================================================
 *  Auto power-off
 * ======================================================================== */

void power_set_auto_off(uint16_t minutes)
{
    auto_off_minutes = minutes;
    idle_seconds = 0;
}

void power_reset_idle_timer(void)
{
    idle_seconds = 0;
    keylock_idle_seconds = 0;
}

void power_poll(void)
{
    /* Log battery state every call (1Hz) */
    {
        uint8_t raw = power_read_battery_raw();
        uint16_t mv = power_get_battery_mv();
        battery_level_t level = power_get_battery_level();
        dbg_reg("[BAT] raw=", raw);
        dbg_reg("  mv=", mv);
        dbg_reg("  level=", (uint32_t)level);
        (void)raw; (void)mv; (void)level;
    }

    /* Log audio ADC level (PA1 channel 1) for VOX calibration.
     * OEM ADC_Read_PA1 @ 0x08013820, UBFX(DR,4,8) → 8-bit. */
    {
        uint8_t audio = adc_read_audio_level();
        dbg_reg("[AUD] raw=", audio);
        (void)audio;
    }

    /* Auto power-off countdown */
    if (auto_off_minutes != 0) {
        idle_seconds++;
        if (idle_seconds >= (uint32_t)auto_off_minutes * 60U)
            power_shutdown();
    }

    /* Auto keypad lock after inactivity */
    {
        const settings_t *s = settings_get();
        if (s->auto_keypad_lock && !s->keypad_lock) {
            keylock_idle_seconds++;
            if (keylock_idle_seconds >= AUTO_KEYLOCK_SECONDS)
                settings_set_u8(offsetof(settings_t, keypad_lock), 1);
        }
    }

    /* Low-battery warning: periodic beep when <= BATT_LOW */
    battery_level_t level = power_get_battery_level();
    if (level <= BATT_LOW) {
        low_batt_alert = 1;
        low_batt_counter++;
        if (low_batt_counter >= LOW_BATT_BEEP_INTERVAL) {
            low_batt_counter = 0;
            audio_beep();  /* audible warning every 30s */
        }
    } else {
        low_batt_alert = 0;
        low_batt_counter = 0;
    }
}

uint8_t power_is_low_batt_alert(void)
{
    return low_batt_alert;
}

/* ========================================================================
 *  Backlight control - PC6 + PB3 (BINARY VERIFIED @ 0x08017C40)
 * ======================================================================== */

void power_backlight_on(void)
{
    gpio_set_pin(LCD_BL_PORT, LCD_BL_PIN);
    gpio_set_pin(LCD_BL_SEC_PORT, LCD_BL_SEC_PIN);
}

void power_backlight_off(void)
{
    gpio_clear_pin(LCD_BL_PORT, LCD_BL_PIN);
    gpio_clear_pin(LCD_BL_SEC_PORT, LCD_BL_SEC_PIN);
}

void power_backlight_toggle(void)
{
    if (LCD_BL_PORT->ODR & LCD_BL_PIN) {
        gpio_clear_pin(LCD_BL_PORT, LCD_BL_PIN);
        gpio_clear_pin(LCD_BL_SEC_PORT, LCD_BL_SEC_PIN);
    } else {
        gpio_set_pin(LCD_BL_PORT, LCD_BL_PIN);
        gpio_set_pin(LCD_BL_SEC_PORT, LCD_BL_SEC_PIN);
    }
}

/* ========================================================================
 *  Breathing light - software PWM triangle wave on backlight GPIO.
 *
 *  Called at ~200 Hz.  PWM period = 20 ticks (100 Hz effective).
 *  Duty cycle ramps 0->20->0 over 400 ticks total (~2 s cycle).
 *  Only active when settings.breathing_light is enabled.
 * ======================================================================== */

#define BREATH_PWM_PERIOD  20   /* ticks per PWM cycle */
#define BREATH_RAMP_STEPS  20   /* duty steps from min to max */

static uint16_t breath_counter;  /* 0..399 */

void power_breathing_poll(void)
{
    if (!settings_get()->breathing_light) return;

    uint16_t phase = breath_counter % (BREATH_RAMP_STEPS * 2 * BREATH_PWM_PERIOD);
    uint16_t ramp_pos = phase / BREATH_PWM_PERIOD;  /* 0..39 */
    uint16_t duty;

    /* Triangle: ramp up 0->20, then down 20->0 */
    if (ramp_pos < BREATH_RAMP_STEPS)
        duty = ramp_pos;
    else
        duty = (BREATH_RAMP_STEPS * 2) - ramp_pos;

    uint16_t pwm_pos = phase % BREATH_PWM_PERIOD;
    if (pwm_pos < duty)
        gpio_set_pin(LCD_BL_PORT, LCD_BL_PIN);
    else
        gpio_clear_pin(LCD_BL_PORT, LCD_BL_PIN);

    breath_counter++;
    if (breath_counter >= BREATH_RAMP_STEPS * 2 * BREATH_PWM_PERIOD)
        breath_counter = 0;
}

/* ========================================================================
 *  power_button_poll - Monitor PE0 for power-off via rotary switch.
 *
 *  PE0 is LOW when the switch is in the ON position (grounded by switch).
 *  PE0 goes HIGH (pulled up) when the switch is turned to the OFF position.
 *  After 1.5 seconds of sustained HIGH, triggers hardware power-off.
 * ======================================================================== */

void power_button_poll(void)
{
    if (pwr_btn_shutdown_triggered)
        return;

    /* PE0 reads LOW when switch is in ON position (grounded).
     * When turned to OFF, PE0 floats HIGH via pull-up.
     * Detect HIGH = switch moved to off position. */
    uint8_t off_pos = (PWR_SWITCH_PORT->IDR & PWR_SWITCH_PIN) ? 1 : 0;

    if (off_pos) {
        pwr_btn_held_count++;
        if (pwr_btn_held_count == 1)
            dbg_puts("[PWR] switch -> OFF detected\n");
        if (pwr_btn_held_count >= PWR_BTN_LONG_PRESS_COUNT) {
            dbg_puts("[PWR] sustained OFF - powering down\n");
            pwr_btn_shutdown_triggered = 1;
            event_post(EVT_POWER_BUTTON, 1);
            power_off();
        }
    } else {
        if (pwr_btn_held_count > 0) {
            dbg_puts("[PWR] switch -> ON (cancelled)\n");
            event_post(EVT_POWER_BUTTON, 0);
        }
        pwr_btn_held_count = 0;
    }
}

/* ========================================================================
 *  power_off - Hardware power-off via PB9 latch release.
 *
 *  OEM shutdown @ 0x0801E2A4: disables RF paths, mutes speaker, clears
 *  backlight, then releases PB9 latch → hardware cuts Vcc.
 *
 *  Sequence:
 *    1. Disable interrupts (prevent further processing)
 *    2. Turn off RF (clear PA enable, TX relays)
 *    3. Turn off LCD backlight and display
 *    4. Turn off LEDs
 *    5. Brief shutdown tone (if audio was working)
 *    6. Release PB9 power latch - hardware cuts power
 *    7. WFI safety loop (in case hardware doesn't cut immediately)
 * ======================================================================== */

void power_off(void)
{
    dbg_puts("[PWR] shutdown sequence\n");

    __disable_irq();

    /* Turn off RF - clear PA enable (PE4) and relay controls */
    gpio_clear_pin(GPIOE, GPIO_PIN_4);   /* PA_EN off */
    gpio_clear_pin(GPIOE, GPIO_PIN_7);   /* U3T_EN off */
    gpio_clear_pin(GPIOE, GPIO_PIN_14);  /* SW3T_EN off */
    gpio_clear_pin(GPIOB, GPIO_PIN_0);   /* V3R_EN off */
    gpio_clear_pin(GPIOB, GPIO_PIN_1);   /* V3T_EN off */

    /* Mute speaker */
    gpio_set_pin(GPIOE, GPIO_PIN_1);     /* SPK_MUTE assert */

    /* Turn off LCD backlight and LEDs */
    gpio_clear_pin(LCD_BL_PORT, LCD_BL_PIN);
    gpio_clear_pin(LCD_BL_SEC_PORT, LCD_BL_SEC_PIN);
    gpio_clear_pin(GPIOC, GPIO_PIN_13);  /* red LED off */
    gpio_clear_pin(GPIOC, GPIO_PIN_14);  /* green LED off */

    /* Release PB9 power latch - hardware regulator cuts power */
    gpio_clear_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN);

    /* Give the regulator time to actually drop the rail. */
    for (volatile uint32_t i = 0; i < 3000000UL; i++)
        ;

    /* Still executing? Then the supply did NOT collapse. Wait for the switch,
     * and reset only when it returns to ON.
     *
     * Three behaviours have to be right at once, and only this gets all three:
     *
     *   - If the rail really does drop, none of the code below ever runs and
     *     the radio is genuinely off. Nothing here can cost battery.
     *   - If the rail stays up, an unconditional NVIC_SystemReset() here would
     *     boot the radio straight back up: it would never actually turn off and
     *     would drain the battery. So we do not reset unconditionally.
     *   - `for (;;) __WFI()` -- what this used to be -- hangs the CPU with
     *     interrupts disabled. The screen is dark because the backlight is off
     *     above, so the radio LOOKS off while the CPU is alive and
     *     unrecoverable. Turning the knob back on does nothing; only pulling
     *     the battery works. That is exactly the fault being fixed.
     *
     * So: sleep in WFI, wake periodically, and watch PE0. It reads LOW when the
     * switch is ON. When the user turns the knob back on, reset and boot -- the
     * bootloader then starts the application normally.
     *
     * Radtel's RT-900 resets after its power-off too (BoardFun.c
     * CheckPowerOff: POWER_OFF, DelayMs(500), NVIC_SystemReset(), commented
     * "reset the system, to avoid powering on immediately after shutdown").
     * This is the same idea, made conditional so it cannot boot-loop.
     *
     * Draw while waiting: RF, PA, backlight and LEDs are all off already, the
     * core is in WFI most of the time, and this state is only ever reached if
     * the hardware failed to cut power -- in which case something is drawing
     * current regardless. */
    for (;;) {
        for (volatile uint32_t i = 0; i < 200000UL; i++)
            ;

        if (!(PWR_SWITCH_PORT->IDR & PWR_SWITCH_PIN)) {
            /* Switch back ON -- reboot into the application. */
            __DSB();
            SCB->AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
            __DSB();
        }

        __WFI();
    }
}

/* ========================================================================
 *  power_reset - MCU reset via NVIC SYSRESETREQ.
 *
 *  Use for error recovery when a full hardware power cycle isn't needed.
 *  After reset the bootloader runs and re-enters firmware normally.
 * ======================================================================== */

void power_reset(void)
{
    __disable_irq();
    __DSB();
    SCB->AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
    __DSB();
    for (;;)
        __WFI();
}

/* ========================================================================
 *  power_shutdown - Backward-compatible shutdown entry point.
 *
 *  Now calls power_off() for true hardware power-off instead of
 *  SYSRESETREQ. Auto power-off timer and menu both call this.
 * ======================================================================== */

void power_shutdown(void)
{
    power_off();
}
