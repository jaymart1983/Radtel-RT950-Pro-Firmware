/*
 * main.c - Entry point for the RT-950 Pro custom firmware
 *
 * Super-loop architecture matching OEM firmware pattern:
 *   SystemInit -> peripherals -> infinite loop with periodic tasks.
 *
 * Subsystems are brought up in dependency order:
 *   1. DMA (needed by LCD + DAC)
 *   2. SPI2 (flash)
 *   3. BK4829 (dual RF transceivers)
 *   4. SI4732 (broadcast receiver)
 *   5. UART (GPS + Bluetooth)
 *   6. ADC (battery + audio level)
 *   7. DAC (CTCSS/AFSK tone gen)
 *   8. LCD + display
 *   9. Keypad + encoder
 *   10. GPS parser
 *   11. Radio control
 *
 * Build with -DHW_TEST=N to run a standalone hardware test instead.
 * See include/tests/hw_test.h for test list.
 */

#ifdef HW_TEST
#include "tests/hw_test.h"
#include "app/update_listener.h"

int main(void)
{
    /* PB9 POWER LATCH FIRST -- before anything else whatsoever.
     *
     * The power/volume knob only applies power momentarily; the firmware has to
     * grab this latch within milliseconds or the radio dies before it finishes
     * booting. The normal main() does this as its first action for exactly that
     * reason, but the HW_TEST main() did not, so a test build could power off
     * cleanly and then refuse to power back on -- the knob would apply power,
     * the firmware would still be initialising, and it would drop dead again.
     * Recovering needed a battery pull.
     *
     * Bare register writes, not gpio_config_pin(), so this happens before any
     * driver is touched: GPIOB clock on, PB9 as 2 MHz push-pull output, set. */
    {
        volatile uint32_t *apb2en = (volatile uint32_t *)0x40021018UL;
        *apb2en |= (1UL << 3);                       /* IOPBEN */
        volatile uint32_t *crh = (volatile uint32_t *)(0x40010C00UL + 0x04UL);
        uint32_t v = *crh;
        v &= ~(0xFUL << 4);                          /* PB9 = CRH bits [7:4] */
        v |=  (0x2UL << 4);                          /* mode 10 (2 MHz), cnf 00 */
        *crh = v;
        *(volatile uint32_t *)(0x40010C00UL + 0x10UL) = (1UL << 9);  /* SCR: PB9 high */
    }

    /* Arm the soft update listener in the hardware tests too. Without it, every
     * rung of the bring-up ladder would need the side-button + battery-pull
     * dance to move to the next one, which is most of the cost of testing. */
    update_listener_init();

#if   HW_TEST == 1
    test_blinky();
#elif HW_TEST == 2
    test_uart_echo();
#elif HW_TEST == 3
    test_lcd_pattern();
#elif HW_TEST == 4
    test_bk4829_id();
#elif HW_TEST == 5
    test_si4732_rev();
#elif HW_TEST == 6
    test_spi_flash_id();
#elif HW_TEST == 7
    test_adc_monitor();
#elif HW_TEST == 8
    test_dac_tone();
#elif HW_TEST == 9
    test_keypad_encoder();
#elif HW_TEST == 10
    test_gps_display();
#elif HW_TEST == 11
    test_full_diagnostic();
#else
#error "Unknown HW_TEST value (valid: 1-11)"
#endif
    return 0;
}

#else /* Normal firmware build */

#include "at32f403a.h"
#include "rt950_pinmap.h"
#include "debug_uart.h"
#include "drivers/gpio.h"
#include "drivers/dma.h"
#include "drivers/spi.h"
#include "drivers/bk4829.h"
#include "drivers/si4732.h"
#include "drivers/uart.h"
#include "drivers/adc.h"
#include "drivers/dac_audio.h"
#include "drivers/timer.h"
#include "drivers/lcd.h"
#include "app/display.h"
#include "app/keypad.h"
#include "app/encoder.h"
#include "app/gps.h"
#include "app/radio.h"
#include "app/vox.h"
#include "app/scanner.h"
#include "app/power.h"
#include "app/fm_radio.h"
#include "app/channel.h"
#include "app/menu.h"
#include "app/channel_picker.h"
#include "app/zone_filter.h"
#include "app/zone_browser.h"
#include "app/update_listener.h"
#include "app/freq_entry.h"
#include "app/dtmf.h"
#include "app/splash.h"
#include "app/vfo.h"
#include "app/aprs.h"
#include "app/bluetooth.h"
#include "app/am_radio.h"
#include "app/cps.h"
#include "app/settings.h"
#include "app/audio.h"
#include "app/noaa.h"
#include "app/crossband.h"
#include "drivers/calibration.h"
#include "drivers/flash_wearleveling.h"
#include "kernel/scheduler.h"
#include "kernel/event.h"

extern void delay_ms(uint32_t ms);
extern uint32_t get_tick(void);


/* Periodic task intervals (ms) - used by sched_register in app_init */
#define TICK_KEYPAD_MS      20      /* 50 Hz keypad scan */
#define TICK_ENCODER_MS     5       /* 200 Hz encoder poll */
#define TICK_BATTERY_MS     1000    /* 1 Hz battery check */
#define TICK_GPS_MS         100     /* 10 Hz GPS parse */
#define TICK_DISPLAY_MS     33      /* ~30 fps display refresh */
#define TICK_DUALWATCH_MS   200     /* 5 Hz dual-watch RSSI check */
#define TICK_POWER_BTN_MS   20      /* 50 Hz power button monitor */
#define TICK_BUTTONS_MS     20      /* 50 Hz PTT + side button monitor */
#define TICK_AUDIO_MS       5       /* 200 Hz audio tone management */
#define TICK_CPS_MS         50      /* 20 Hz CPS programming poll */
#define TICK_UI_MS          10      /* 100 Hz UI event dispatch */

/* Feed IWDG early - bootloader enables watchdog before jumping to us */
#define IWDG_FEED()  (*(volatile uint32_t *)0x40003000UL = 0x0000AAAAUL)

/* Global calibration data - loaded once in hw_init, used by radio/power */
calibration_t cal_data;

/* Forward declarations ------------------------------------------------ */
static void hw_init(void);
static void app_init(void);

/* Scheduler task wrappers - adapt module APIs to void(*)(void) ---------- */

/* One-shot diagnostic counters */
static uint8_t keypad_diag_count = 0;
static uint8_t encoder_diag_count = 0;

static const char * const key_names[] = {
    "1","2","3","A/VFO", "4","5","6","B/SCAN",
    "7","8","9","C/MENU", "*","0","#","D/BAND",
    "C4R0","C4R1","C4R2","C4R3",
    "SIDE1","SIDE4"
};

/* Unique tone per key (freq x10): each key gets a distinct pitch */
static const uint16_t key_tones[] = {
    /* 1=700Hz  2=800Hz  3=900Hz  A=1900Hz */
    7000, 8000, 9000, 19000,
    /* 4=1000Hz 5=1100Hz 6=1200Hz B=2000Hz */
    10000, 11000, 12000, 20000,
    /* 7=1300Hz 8=1400Hz 9=1500Hz C=2100Hz */
    13000, 14000, 15000, 21000,
    /* *=600Hz  0=1600Hz #=1700Hz D=2200Hz */
    6000, 16000, 17000, 22000,
    /* C4R0=500Hz C4R1=2400Hz C4R2=2600Hz C4R3=2800Hz */
    5000, 24000, 26000, 28000,
    /* SIDE1=3000Hz SIDE4=3200Hz */
    30000, 32000,
};

static void task_keypad(void)
{
    /* Dump raw GPIO state every ~2 seconds for first 10 samples */
    if (keypad_diag_count < 10) {
        static uint16_t diag_ticks = 0;
        diag_ticks++;
        if (diag_ticks >= 100) {  /* 100 * 20ms = 2s */
            diag_ticks = 0;
            keypad_diag_count++;
            /* Side buttons (PE5/PA12), cols (PC0-3), rows (PD4-7) */
            volatile uint32_t *gpioc_idr = (volatile uint32_t *)0x40011008UL;
            volatile uint32_t *gpiod_idr = (volatile uint32_t *)0x40011408UL;
            volatile uint32_t *gpioe_idr = (volatile uint32_t *)0x40011808UL;
            volatile uint32_t *gpioa_idr = (volatile uint32_t *)0x40010808UL;
            uint32_t pc = *gpioc_idr;
            (void)pc;
            uint32_t pd = *gpiod_idr;
            (void)pd;
            uint32_t pe = *gpioe_idr;
            uint32_t pa = *gpioa_idr;
            dbg_puts("[KBD_DIAG] PC=");
            dbg_reg("", pc);
            dbg_puts(" PD=");
            dbg_reg("", pd);
            uint8_t side1 = (pe >> 5) & 1;  /* PE5 TOP_PROG */
            (void)side1;
            uint8_t side4 = (pa >> 12) & 1; /* PA12 BOT_PROG */
            (void)side4;
            dbg_puts(" SIDE1=");
            dbg_reg("", side1);
            dbg_puts(" SIDE4=");
            dbg_reg("", side4);
        }
    }

    key_event_t evt;
    if (keypad_get_event(&evt)) {
        const char *kn = (evt.key < KEY_COUNT) ? key_names[evt.key] : "??";
        (void)kn;
        switch (evt.type) {
        case KEY_EVT_PRESS:
            dbg_puts("[KEY] press: ");
            dbg_puts(kn);
            dbg_puts("\n");
            if (evt.key < KEY_COUNT)
                audio_beep_freq(key_tones[evt.key], 60);
            event_post(EVT_KEY_PRESS, evt.key);
            power_reset_idle_timer();
            break;
        case KEY_EVT_RELEASE:
            dbg_puts("[KEY] release: ");
            dbg_puts(kn);
            dbg_puts("\n");
            event_post(EVT_KEY_RELEASE, evt.key);
            break;
        case KEY_EVT_REPEAT:
            dbg_puts("[KEY] repeat: ");
            dbg_puts(kn);
            dbg_puts("\n");
            event_post(EVT_KEY_PRESS, evt.key);
            break;
        default:
            break;
        }
    }
}

static void task_encoder(void)
{
    /* Dump raw PB4/PB5 state every ~2s for first 10 samples */
    if (encoder_diag_count < 10) {
        static uint16_t enc_diag_ticks = 0;
        enc_diag_ticks++;
        if (enc_diag_ticks >= 400) {  /* 400 * 5ms = 2s */
            enc_diag_ticks = 0;
            encoder_diag_count++;
            volatile uint32_t *gpiob_idr = (volatile uint32_t *)0x40010C08UL;
            uint32_t pb = *gpiob_idr;
            uint8_t enc_a = (pb >> 4) & 1;
            (void)enc_a;
            uint8_t enc_b = (pb >> 5) & 1;
            (void)enc_b;
            dbg_puts("[ENC_DIAG] PB=");
            dbg_reg("", pb);
            dbg_puts(" A=");
            dbg_reg("", enc_a);
            dbg_puts(" B=");
            dbg_reg("", enc_b);
        }
    }

    int8_t dir = encoder_poll();
    if (dir > 0) {
        dbg_puts("[ENC] CW\n");
        audio_beep_freq(18000, 30);  /* 1800 Hz, 30 ms - rising feel */
        event_post(EVT_ENCODER_CW, 1);
        power_reset_idle_timer();
    } else if (dir < 0) {
        dbg_puts("[ENC] CCW\n");
        audio_beep_freq(12000, 30);  /* 1200 Hz, 30 ms - falling feel */
        event_post(EVT_ENCODER_CCW, 1);
        power_reset_idle_timer();
    }
}

/* PTT and side-port button monitor (Phase 12.3/12.4).
 * Reads standalone GPIO buttons NOT on the keypad matrix.
 * OEM references:
 *   PE3 = PTT primary   (gpio_modes_init @ 0x0801391C, BINARY_VERIFIED)
 *   PE2 = PTT2 secondary (gpio_modes_init @ 0x0801391C, BINARY_VERIFIED)
 *   PE6 = EXT_PTT        (spk_mute_on_ptt @ 0x08019254, BINARY_VERIFIED)
 * Reports state changes via debug UART. Does NOT activate TX relays. */
static uint8_t prev_ptt   = 1;   /* idle HIGH (active-low) */
static uint8_t prev_ptt2  = 1;
static uint8_t prev_extptt = 1;

static void task_buttons(void)
{
    uint8_t ptt    = gpio_read_pin(PTT_PORT,     PTT_PIN)     ? 1 : 0;
    uint8_t ptt2   = gpio_read_pin(PTT2_PORT,    PTT2_PIN)    ? 1 : 0;
    uint8_t extptt = gpio_read_pin(EXT_PTT_PORT, EXT_PTT_PIN) ? 1 : 0;

    if (ptt != prev_ptt) {
        prev_ptt = ptt;
        dbg_puts(ptt ? "[BTN] PTT released\n" : "[BTN] PTT pressed\n");
        event_post(ptt ? EVT_KEY_RELEASE : EVT_KEY_PRESS, 0xE3);
        if (!ptt) power_reset_idle_timer();
    }
    if (ptt2 != prev_ptt2) {
        prev_ptt2 = ptt2;
        dbg_puts(ptt2 ? "[BTN] PTT2 released\n" : "[BTN] PTT2 pressed\n");
    }
    if (extptt != prev_extptt) {
        prev_extptt = extptt;
        dbg_puts(extptt ? "[BTN] EXT_PTT released\n" : "[BTN] EXT_PTT pressed\n");
    }
}

static void task_gps(void)
{
    gps_process();
}

static void task_cps(void)
{
    cps_poll();
}

/* ========================================================================
 *  audio_diagnostic_v12 - Isolated in own stack frame to prevent
 *  main() stack bloat that causes HardFault during bk4829_init.
 * ======================================================================== */
__attribute__((noinline)) static void audio_diagnostic_v12(void)
{
    dbg_puts("[AUD] === AUDIO DIAGNOSTIC V12 ===\n");
    dbg_puts("[AUD] Aggressive reset + PC12=LOW confirmed path\n");

    /* ================================================================
     * PHASE 1: AGGRESSIVE HARDWARE RESET
     * BK4829 AF output loads the bus even in "standby" if the chip's
     * internal LDO still has charge.  We must fully isolate it.
     * ================================================================ */

    /* 1a: BK4829 deep shutdown - both chips */
    bk4829_standby(BK4829_CHIP0);
    bk4829_standby(BK4829_CHIP1);
    /* Mute AF output explicitly: R47[15:14]=0 disables AF DAC */
    bk4829_write_reg(BK4829_CHIP0, 0x47, 0x0000);
    bk4829_write_reg(BK4829_CHIP1, 0x47, 0x0000);
    /* R48=0 disables IF path */
    bk4829_write_reg(BK4829_CHIP0, 0x48, 0x0000);
    bk4829_write_reg(BK4829_CHIP1, 0x48, 0x0000);

    /* 1b: Full DAC reset */
    DAC->CR = 0;
    DMA2_CH(3)->CCR = 0;
    ((TIM_TypeDef*)0x40001000)->CR1 = 0;  /* TIM6 stop */

    /* 1c: Amp power cycle - force PE4 LOW for 100ms to discharge */
    gpio_config_pin(GPIOE, GPIO_PIN_4, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_clear_pin(GPIOE, GPIO_PIN_4);   /* PE4 LOW - amp power OFF */
    gpio_config_pin(GPIOB, GPIO_PIN_8, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_clear_pin(GPIOB, GPIO_PIN_8);   /* PB8 LOW - amp disable */
    gpio_config_pin(GPIOE, GPIO_PIN_1, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(GPIOE, GPIO_PIN_1);     /* PE1 HIGH - mute */
    gpio_config_pin(GPIOC, GPIO_PIN_12, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_clear_pin(GPIOC, GPIO_PIN_12);  /* PC12 LOW */

    /* PA3 OD LOW - match OEM */
    gpio_config_pin(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_10MHZ, GPIO_CNF_OD);
    gpio_clear_pin(GPIOA, GPIO_PIN_3);

    dbg_puts("[AUD] Amp power-cycled OFF, waiting 500ms...\n");
    delay_ms(500); IWDG_FEED();

    /* 1d: Bring amp back up with settling time */
    gpio_set_pin(GPIOE, GPIO_PIN_4);     /* PE4 HIGH - amp power ON */
    gpio_set_pin(GPIOB, GPIO_PIN_8);     /* PB8 HIGH - amp enable */
    gpio_clear_pin(GPIOE, GPIO_PIN_1);   /* PE1 LOW - unmute */
    dbg_puts("[AUD] Amp powered ON, settling 200ms...\n");
    delay_ms(200); IWDG_FEED();

    /* ================================================================
     * PHASE 2: FOCUSED AUDIO TESTS
     * All tests use PC12=LOW (confirmed working).
     * ================================================================ */

    /* ---- T1: BF square wave 500Hz, PC12=LOW, BOFF1=1, 5s ---- */
    dbg_puts("[AUD] T1: BF 500Hz PC12=LOW BOFF1=1 5s\n");
    {
        gpio_clear_pin(GPIOC, GPIO_PIN_12);
        /* EN2 + BOFF1 + EN1 */
        DAC->CR = (1UL << 16) | (1UL << 1) | (1UL << 0);
        delay_ms(1);
        uint32_t t0 = get_tick();
        while (get_tick() - t0 < 5000) {
            DAC->DHR12R1 = 0xFFF; delay_ms(1);
            DAC->DHR12R1 = 0x000; delay_ms(1);
            IWDG_FEED();
        }
        DAC->CR = 0;
        dbg_puts("[AUD] T1 done\n");
    }
    delay_ms(500); IWDG_FEED();

    /* ---- T2: BF 500Hz, PC12=LOW, buffer ON (BOFF1=0), 5s ---- */
    dbg_puts("[AUD] T2: BF 500Hz PC12=LOW bufON 5s\n");
    {
        gpio_clear_pin(GPIOC, GPIO_PIN_12);
        /* EN2 + EN1, NO BOFF1 = buffer enabled (lower impedance) */
        DAC->CR = (1UL << 16) | (1UL << 0);
        delay_ms(1);
        uint32_t t0 = get_tick();
        while (get_tick() - t0 < 5000) {
            DAC->DHR12R1 = 0xFFF; delay_ms(1);
            DAC->DHR12R1 = 0x000; delay_ms(1);
            IWDG_FEED();
        }
        DAC->CR = 0;
        dbg_puts("[AUD] T2 done\n");
    }
    delay_ms(500); IWDG_FEED();

    /* ---- T3: DMA 1kHz tone, PC12=LOW, 5s ---- */
    dbg_puts("[AUD] T3: DMA 1kHz PC12=LOW 5s\n");
    {
        gpio_clear_pin(GPIOC, GPIO_PIN_12);
        dac_audio_play_tone(10000);
        dbg_reg("[AUD] T3 DAC_CR=0x", DAC->CR);
        delay_ms(5000); IWDG_FEED();
        dac_audio_stop();
        dbg_puts("[AUD] T3 done\n");
    }
    delay_ms(500); IWDG_FEED();

    /* ---- T4: DMA frequency sweep PC12=LOW ---- */
    dbg_puts("[AUD] T4: Sweep 500/1000/2000Hz PC12=LOW 3s each\n");
    {
        gpio_clear_pin(GPIOC, GPIO_PIN_12);
        dac_audio_play_tone(5000);  /* 500 Hz */
        dbg_puts("[AUD] 500Hz\n");
        delay_ms(3000); IWDG_FEED();
        dac_audio_stop(); delay_ms(100);
        dac_audio_play_tone(10000); /* 1000 Hz */
        dbg_puts("[AUD] 1000Hz\n");
        delay_ms(3000); IWDG_FEED();
        dac_audio_stop(); delay_ms(100);
        dac_audio_play_tone(20000); /* 2000 Hz */
        dbg_puts("[AUD] 2000Hz\n");
        delay_ms(3000); IWDG_FEED();
        dac_audio_stop();
        dbg_puts("[AUD] T4 done\n");
    }
    delay_ms(500); IWDG_FEED();

    /* ---- T5: PC12=HIGH comparison (should be silent) 3s ---- */
    dbg_puts("[AUD] T5: DMA 1kHz PC12=HIGH 3s (expect silent)\n");
    {
        gpio_set_pin(GPIOC, GPIO_PIN_12);
        dac_audio_play_tone(10000);
        delay_ms(3000); IWDG_FEED();
        dac_audio_stop();
        dbg_puts("[AUD] T5 done\n");
    }
    delay_ms(500); IWDG_FEED();

    /* ---- T6: Continuous 1kHz tone for enjoyment ---- */
    dbg_puts("[AUD] T6: Continuous 1kHz PC12=LOW (10s)\n");
    {
        gpio_clear_pin(GPIOC, GPIO_PIN_12);
        dac_audio_play_tone(10000);
        delay_ms(10000); IWDG_FEED();
        dac_audio_stop();
        dbg_puts("[AUD] T6 done\n");
    }

    /* Cleanup: stop audio, mute amp */
    dac_audio_stop();
    DAC->CR = 0;
    gpio_clear_pin(GPIOB, GPIO_PIN_8);   /* amp disable */
    gpio_set_pin(GPIOE, GPIO_PIN_1);     /* mute */
    gpio_clear_pin(GPIOC, GPIO_PIN_12);
    bk4829_standby(BK4829_CHIP0);
    bk4829_standby(BK4829_CHIP1);

    dbg_puts("[AUD] === TESTS COMPLETE ===\n");
}

/* ======================================================================== */

int main(void)
{
    /*
     * SystemInit() has already been called by Reset_Handler in startup.c.
     * At this point: SYSCLK = 120 MHz, SysTick @ 1 ms, GPIO clocks A-E on.
     */

    /* Dump reset reason flags from CRM->CTRLSTS (0x40021024).
     * Bit 28: WDT (IWDG), Bit 29: SFT (software), Bit 30: POR,
     * Bit 31: PIN (NRST), Bit 27: WWDT. Clear flags after reading. */
    {
        volatile uint32_t *ctrlsts = (volatile uint32_t *)0x40021024UL;
        uint32_t rst = *ctrlsts;
        (void)rst;
        dbg_reg("[DBG] RST_FLAGS=0x", rst);
        *ctrlsts |= (1UL << 24);  /* RSTFC: clear reset flags */
    }

    dbg_puts("[DBG] main() entered\n");

    /*
     * Disable all DMA channels - bootloader may have left transfers active.
     * Must be done after DMA clocks are implicitly enabled (AHB peripheral
     * clocks survive software resets on AT32F403A).
     * DMA1 channels at base 0x40020000, DMA2 at 0x40020400.
     * Each channel: CCR at offset +0x08 + (ch-1)*0x14.
     */
    {
        /* Enable DMA clocks first (may already be on from bootloader) */
        volatile uint32_t *ahben = (volatile uint32_t *)0x40021014UL;
        *ahben |= (1UL << 0) | (1UL << 1);  /* DMA1EN + DMA2EN */

        /* DMA1 channels 1-7: write 0 to CCR to disable */
        for (int ch = 0; ch < 7; ch++)
            *(volatile uint32_t *)(0x40020008UL + ch * 0x14UL) = 0;
        /* DMA2 channels 1-5: write 0 to CCR to disable */
        for (int ch = 0; ch < 5; ch++)
            *(volatile uint32_t *)(0x40020408UL + ch * 0x14UL) = 0;
    }

    /* Hex table self-test: verify .rodata integrity after BTF upload.
     * If corrupted, you'll see garbled chars instead of 0123456789ABCDEF. */
    dbg_puts("[DBG] HEX_TEST=");
    for (uint32_t i = 0; i < 16; i++)
        dbg_hex8((unsigned char)i);
    dbg_puts(" (expect 000102...0F)\n");

    /* ABSOLUTE MINIMUM: PB9 power latch must be asserted immediately
     * or the radio will power off. */
    gpio_config_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN);
    IWDG_FEED();

    /* Arm the soft update listener BEFORE anything else.
     *
     * Placed here on purpose: everything below can fail, and the whole point of
     * this listener is to reflash a radio whose firmware is broken. It runs
     * from the UART4 RX interrupt, so it survives a hung main loop or a wedged
     * scheduler. Send the normal handshake with firmware_upload.py (no --ptt)
     * and the radio hands itself to the bootloader -- no side buttons, no
     * battery pull.
     *
     * The side-button entry remains the guaranteed path: this cannot survive a
     * fault that kills interrupts. */
    update_listener_init();

    /* Initialize event queue */
    event_init();

    /* Bring up hardware peripherals */
    hw_init();

    /* Initialize application-layer modules and register scheduler tasks */
    app_init();

    /* ================================================================
     * POST-INIT AUDIO TESTS
     * ================================================================ */
    dbg_puts("[AUD] === AUDIO TESTS ===\n");
    {
        /* Configure BK4829 audio filter/AF path (OEM audio_route_setup) */
        bk4829_audio_filter_config(BK4829_CHIP0);
        bk4829_audio_filter_config(BK4829_CHIP1);
        dbg_puts("[AUD] BK4829 audio filters configured\n");

        /* Set BK4829 AF path to BEEP mode (OEM @ 0x0801BC2C mode=3)
         * REG_47=0xEB4F routes MCU DAC analog input to speaker driver
         * REG_48=0xB0A3 sets fixed gain for beep output
         * Without this, the BK4829 speaker path ignores the DAC signal! */
        bk4829_set_af_beep(BK4829_CHIP0);
        bk4829_set_af_beep(BK4829_CHIP1);
        dbg_puts("[AUD] BK4829 AF path set to BEEP mode\n");

        /* Verify REG 0x47 readback */
        dbg_reg("[AUD]   C0 REG47=0x", bk4829_read_reg(BK4829_CHIP0, 0x47));
        dbg_reg("[AUD]   C0 REG48=0x", bk4829_read_reg(BK4829_CHIP0, 0x48));

        /* Audio path GPIOs: PE1 LOW=unmute, PB8 HIGH=amp,
         * PC12 HIGH=radio/BK4829 path (LOW=DAC beep path, V12 verified),
         * PE7 HIGH=RX/speaker mode (ACTIVE LOW relay), PE9 LOW=speaker (not BT).
         * CRITICAL: PB8 must be configured as push-pull output BEFORE
         * setting it HIGH.  Without this, PB8 stays in floating-input
         * mode after reset and gpio_set_pin only enables a weak ~40kΩ
         * internal pull-up - not enough to drive the amplifier. */
        gpio_set_pin(GPIOE, GPIO_PIN_7);             /* U3T_EN HIGH = RX/spk */
        gpio_clear_pin(GPIOE, GPIO_PIN_9);           /* SW_TO_BT LOW */
        gpio_clear_pin(GPIOE, GPIO_PIN_1);
        gpio_config_pin(GPIOB, GPIO_PIN_8,           /* AMP_EN */
                        GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
        gpio_set_pin(GPIOB, GPIO_PIN_8);             /* enable amplifier */
        gpio_set_pin(GPIOC, GPIO_PIN_12);            /* PC12 HIGH = radio path */

        /* Verify audio routing GPIO state */
        dbg_reg("[AUD] PE7=0x", (GPIOE->IDR >> 7) & 1);
        dbg_reg("[AUD] PE9=0x", (GPIOE->IDR >> 9) & 1);
        dbg_reg("[AUD] PE1=0x", (GPIOE->IDR >> 1) & 1);
        dbg_reg("[AUD] PB8=0x", (GPIOB->IDR >> 8) & 1);
        dbg_reg("[AUD] PC12=0x", (GPIOC->IDR >> 12) & 1);

        /* Dump ALL GPIO port registers to find any missing config.
         * Compare CRL/CRH/ODR with OEM to find hidden control signals. */
        dbg_puts("[GPIO] === PORT REGISTER DUMP ===\n");
        {
            typedef struct { volatile uint32_t CRL, CRH, IDR, ODR, BSR, BRR, LOCK; } GP;
            static const struct { const char *name; uint32_t base; } ports[] = {
                {"A", 0x40010800}, {"B", 0x40010C00}, {"C", 0x40011000},
                {"D", 0x40011400}, {"E", 0x40011800}
            };
            int p;
            for (p = 0; p < 5; p++) {
                volatile GP *g = (volatile GP *)ports[p].base;
                (void)g;
                dbg_puts("[GPIO] PORT");
                dbg_puts(ports[p].name);
                dbg_puts(": ");
                dbg_reg("CRL=", g->CRL);
                dbg_reg(" CRH=", g->CRH);
                dbg_reg(" ODR=", g->ODR);
                dbg_puts("\n");
            }
        }
        dbg_puts("[GPIO] === END DUMP ===\n");

        /* Verify PA4/PA5 analog mode + DAC state before test */
        dbg_reg("[AUD] GPIOA_CRL=0x", ((GPIO_TypeDef *)GPIOA_BASE)->CRL);
        dbg_reg("[AUD] DAC_CR=0x", DAC->CR);

        /* Dump clock enable registers to verify all peripherals enabled */
        dbg_reg("[CLK] APB1EN=0x", CRM->APB1EN);
        dbg_reg("[CLK] APB2EN=0x", CRM->APB2EN);
        dbg_reg("[CLK] AHBEN=0x",  CRM->AHBEN);

        /* Verify AFIO IOMUX_REMAP - SWJ_CFG should be 010 (bits 26:24) */
        dbg_reg("[AFIO] REMAP=0x", *(volatile uint32_t *)0x40010004UL);

        /* Dump BK4829 GPIO-related registers - BK4829 GPIO outputs may
         * physically control audio mux/switch components on the PCB */
        dbg_reg("[BK] C0 R30=0x", bk4829_read_reg(BK4829_CHIP0, 0x30));
        dbg_reg("[BK] C0 R31=0x", bk4829_read_reg(BK4829_CHIP0, 0x31));
        dbg_reg("[BK] C0 R33=0x", bk4829_read_reg(BK4829_CHIP0, 0x33));
        dbg_reg("[BK] C0 R3F=0x", bk4829_read_reg(BK4829_CHIP0, 0x3F));

        /* Dump DAC peripheral registers in detail */
        dbg_reg("[DAC] CR=0x",    DAC->CR);
        dbg_reg("[DAC] SR=0x",    DAC->SR);
        dbg_reg("[DAC] DOR1=0x",  DAC->DOR1);
        dbg_reg("[DAC] DOR2=0x",  DAC->DOR2);

        /* Full BK4829 CHIP0 register dump (0x00-0x3F) for OEM comparison */
        dbg_puts("[BKDUMP] C0 regs 0x00-0x3F:\n");
        {
            int r;
            for (r = 0; r <= 0x3F; r++) {
                uint16_t v = bk4829_read_reg(BK4829_CHIP0, (uint8_t)r);
                (void)v;
                if (r % 8 == 0) {
                    dbg_reg("[BKDUMP] ", r);
                    dbg_puts(":");
                }
                dbg_reg(" ", v);
                if (r % 8 == 7) dbg_puts("\n");
            }
        }
        dbg_puts("[BKDUMP] C1 regs 0x00-0x3F:\n");
        {
            int r;
            for (r = 0; r <= 0x3F; r++) {
                uint16_t v = bk4829_read_reg(BK4829_CHIP1, (uint8_t)r);
                (void)v;
                if (r % 8 == 0) {
                    dbg_reg("[BKDUMP] ", r);
                    dbg_puts(":");
                }
                dbg_reg(" ", v);
                if (r % 8 == 7) dbg_puts("\n");
            }
        }
        IWDG_FEED();

        /* === AUDIO PATH TESTS (V11) === */
        audio_diagnostic_v12();
    }

    /* Main loop - never returns */
    dbg_puts("[DBG] entering scheduler\n");
    sched_run();

    return 0;  /* unreachable */
}

/* ========================================================================
 *  hw_init - Bring up all hardware peripherals
 * ======================================================================== */

static void hw_init(void)
{
    /* POWER LATCH: PB9 must be held HIGH to keep the radio powered.
     * The bootloader asserts this briefly; our firmware must take over
     * immediately or the radio will power off. */
    gpio_config_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN);

    IWDG_FEED();

    /* DMA (must be before LCD and DAC) -------------------------------- */
    dbg_puts("[DBG] dma_init...\n");
    dma_init();
    IWDG_FEED();

    /* SPI flash (SPI2) ----------------------------------------------- */
    dbg_puts("[DBG] spi2_init...\n");
    spi2_init();
    IWDG_FEED();

    /* Verify flash chip: expect W25Q16 (Winbond 0xEF, 16Mbit 0x4015)
     * OEM: periph_init_flash_adc @ 0x08013820 reads JEDEC after SPI2 init.
     * Valid IDs: 0xEF4015 (W25Q16BV), 0xEF4016 (W25Q32). */
    {
        uint32_t jedec = spi_flash_read_id();
        dbg_reg("[DBG] SPI JEDEC=0x", jedec);
        if ((jedec & 0xFFFF00) == 0xEF4000)
            dbg_puts("[DBG] Flash: Winbond W25Q detected OK\n");
        else if (jedec == 0x000000 || jedec == 0xFFFFFF)
            dbg_puts("[ERR] Flash: no response (check SPI2 wiring)\n");
        else
            dbg_puts("[WARN] Flash: unexpected JEDEC ID\n");
    }

    /* Flash read sanity: dump first 16B of channel memory (0x0000)
     * and calibration validity flag (0xF0E0).
     * OEM: flash_data_load @ 0x08007358 reads these sectors. */
    {
        uint8_t sample[16];
        uint8_t all_ff = 1;
        spi_flash_read(0x000000, sample, 16);
        dbg_puts("[DBG] Flash[0x0000]: ");
        for (int i = 0; i < 16; i++) {
            dbg_hex8(sample[i]);
            dbg_puts(" ");
            if (sample[i] != 0xFF) all_ff = 0;
        }
        dbg_puts("\n");
        if (all_ff)
            dbg_puts("[WARN] Flash channel sector appears blank\n");

        uint8_t cal_flag;
        spi_flash_read(0x00F0E0, &cal_flag, 1);
        dbg_reg("[DBG] Cal flag @ 0xF0E0 = 0x", cal_flag);
        if (cal_flag == 0xFF)
            dbg_puts("[WARN] No calibration data programmed\n");
        else
            dbg_puts("[DBG] Calibration data present\n");
    }

    /* Calibration + wear-leveling (needs SPI flash) ------------------
     * OEM: hw_init_main calls calibration_load then initializes all 5
     * WL sectors. Bitmap scan @ 0x0800EF68, SYSCFG write @ 0x0800F918. */
    dbg_puts("[DBG] calibration+wl...\n");
    calibration_load(&cal_data);

    wl_init(&WL_SYSCFG);
    wl_init(&WL_VFOCFG);
    wl_init(&WL_EXTCFG);
    wl_init(&WL_VFOSEL);
    wl_init(&WL_CHCFG);

    /* Probe each WL sector: attempt a read to check valid/empty/corrupt.
     * OEM validates sectors on boot before dispatching main task loop.
     * Buffer must be large enough for the LARGEST record (EXTCFG = 160B). */
    {
        uint8_t probe[160];
        const char *names[] = {"SYSCFG","VFOCFG","EXTCFG","VFOSEL","CHCFG "};
        (void)names;
        const wl_sector_t *secs[] = {&WL_SYSCFG,&WL_VFOCFG,&WL_EXTCFG,&WL_VFOSEL,&WL_CHCFG};
        for (int i = 0; i < 5; i++) {
            int rc = wl_read(secs[i], probe);
            (void)rc;
            dbg_puts("[DBG] WL ");
            dbg_puts(names[i]);
            dbg_puts(rc == 0 ? ": valid\n" : ": empty/uninitialized\n");
        }
    }
    IWDG_FEED();

    /* BK4829 dual RF transceivers (GPIOE bit-bang) ------------------- */
    /* Configure BK4829 SPI GPIO pins as outputs before init.
     * OEM gpio_modes_init @ 0x0801391C: configures PE8/10/11/15 as
     * push-pull outputs and resets them LOW (CS idle = HIGH after init). */
    dbg_puts("[DBG] bk4829_gpio_init...\n");
    gpio_config_pin(BK4829_SEN1_PORT, BK4829_SEN1_PIN,
                    GPIO_MODE_OUT_50MHZ, GPIO_CNF_PP);      /* PE8 = SEN1 */
    gpio_set_pin(BK4829_SEN1_PORT, BK4829_SEN1_PIN);       /* CS idle HIGH */
    gpio_config_pin(BK4829_SEN2_PORT, BK4829_SEN2_PIN,
                    GPIO_MODE_OUT_50MHZ, GPIO_CNF_PP);      /* PE15 = SEN2 */
    gpio_set_pin(BK4829_SEN2_PORT, BK4829_SEN2_PIN);       /* CS idle HIGH */
    gpio_config_pin(BK4829_SCK_PORT, BK4829_SCK_PIN,
                    GPIO_MODE_OUT_50MHZ, GPIO_CNF_PP);      /* PE10 = SCK */
    gpio_clear_pin(BK4829_SCK_PORT, BK4829_SCK_PIN);       /* SCK idle LOW */
    gpio_config_pin(BK4829_SDA_PORT, BK4829_SDA_PIN,
                    GPIO_MODE_OUT_50MHZ, GPIO_CNF_PP);      /* PE11 = SDA */

    /* BK4829 full init + frequency programming happens in
     * radio_init() → vfo_init() during app_init phase.
     * Removed from here to avoid double-init HardFault. */
    dbg_puts("[DBG] bk4829 GPIO ready (init deferred to vfo_init)\n");
    IWDG_FEED();

    /* SI4732 broadcast receiver (GPIOB bit-bang I2C) ----------------- */
    dbg_puts("[DBG] si4732_init SKIPPED (radio disabled)\n");
    IWDG_FEED();

    /* UARTs ---------------------------------------------------------- */
    dbg_puts("[DBG] uart_bt_init...\n");
    bt_init();              /* USART1 @ 115200 - Bluetooth + AT config */
    IWDG_FEED();
    dbg_puts("[DBG] uart_gps_init...\n");
    uart_gps_init();        /* USART3 @ 9600 - GPS (PB10/PB11) */
    IWDG_FEED();

    /* ADC (battery + audio level) ------------------------------------ */
    dbg_puts("[DBG] adc_init...\n");
    adc_init();
    IWDG_FEED();

    /* DAC + TIM6 (CTCSS/AFSK tone generation) ----------------------- */
    dbg_puts("[DBG] dac_audio_init...\n");
    dac_audio_init();
    IWDG_FEED();

    /* Audio routing GPIO init (OEM gpio_modes_init @ 0x0801391C).
     * OEM configures ALL audio path pins in gpio_modes_init:
     *   PE7  (U3T_EN)  = GP push-pull output - ACTIVE LOW relay
     *                     HIGH = RX/speaker (deasserted), LOW = TX/mic
     *   PE9  (SW_TO_BT) = GP push-pull output, LOW → speaker (not BT)
     *   PE1  (SPK_MUTE) = GP push-pull output, LOW → unmuted
     *   PC12 (BEEP_SW)  = GP push-pull output, LOW at boot (enabled per-beep)
     *   PB8  (AMP_EN)   = GP push-pull output, LOW at boot (enabled per-beep)
     *
     * OEM boot state: PB8=LOW, PC12=LOW - both enabled only during beep_play.
     * PE7=HIGH (RX mode), PE9=LOW (speaker not BT), PE1=LOW (unmuted). */
    dbg_puts("[DBG] spk_unmute+beep_sw...\n");
    /* PE7 = U3T_EN: HIGH = RX/speaker mode (relay deasserted, ACTIVE LOW) */
    gpio_config_pin(GPIOE, GPIO_PIN_7,
                    GPIO_MODE_OUT_10MHZ, GPIO_CNF_PP);
    gpio_set_pin(GPIOE, GPIO_PIN_7);                /* RX audio mode */
    /* PE9 = SW_TO_BT: LOW = speaker path (not Bluetooth) */
    gpio_config_pin(GPIOE, GPIO_PIN_9,
                    GPIO_MODE_OUT_10MHZ, GPIO_CNF_PP);
    gpio_clear_pin(GPIOE, GPIO_PIN_9);              /* speaker mode */
    /* PE1 = SPK_MUTE: LOW = unmuted */
    gpio_config_pin(SPK_MUTE_PORT, SPK_MUTE_PIN,
                    GPIO_MODE_OUT_10MHZ, GPIO_CNF_PP);
    gpio_clear_pin(SPK_MUTE_PORT, SPK_MUTE_PIN);   /* unmute speaker */
    /* PE4 = AMP_PWR: powers the audio amplifier rail (V12 discovery).
     * Must be HIGH for any audio output. */
    gpio_config_pin(GPIOE, GPIO_PIN_4,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(GPIOE, GPIO_PIN_4);               /* amp power ON */
    /* PB8 = AMP_EN: configured as PP output, LOW at boot (OEM default).
     * audio_path_enable() sets HIGH before beep. */
    gpio_config_pin(AMP_EN_PORT, AMP_EN_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_clear_pin(AMP_EN_PORT, AMP_EN_PIN);        /* amp OFF at boot */
    /* PC12 = BEEP_SW: configured as PP output, HIGH at boot (radio path).
     * audio_path_enable() sets LOW to route DAC to speaker. */
    gpio_config_pin(BEEP_SW_PORT, BEEP_SW_PIN,
                    GPIO_MODE_OUT_10MHZ, GPIO_CNF_PP);
    gpio_set_pin(BEEP_SW_PORT, BEEP_SW_PIN);        /* radio path at boot */
    IWDG_FEED();

    /* LCD ------------------------------------------------------------ */
    dbg_puts("[DBG] lcd_init...\n");
    lcd_init();
    IWDG_FEED();
    dbg_puts("[DBG] lcd_backlight...\n");
    /* LCD backlight on ----------------------------------------------- */
    gpio_config_pin(LCD_BL_PORT, LCD_BL_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(LCD_BL_PORT, LCD_BL_PIN);

    /* LCD diagnostic: fill screen blue to confirm 8080 bus works */
    dbg_puts("[DBG] lcd_fill blue...\n");
    lcd_fill_rect(0, 0, 240, 320, 0x001F);  /* RGB565 blue */
    IWDG_FEED();

    /* -- Audio diagnostic SKIPPED (moved to post-init beep test) -- */


    /* Configure PTT and side-button inputs with internal pull-ups.
     * OEM gpio_modes_init leaves PE0-6 as default (floating input).
     * We explicitly configure with pull-ups for reliable reads.
     * OEM @ 0x0801391C: PE2-3 listed as PTT, PE5 as SIDE_KEY. */
    gpio_config_pin(GPIOE, GPIO_PIN_3, GPIO_MODE_INPUT, GPIO_CNF_PULL);
    gpio_set_pin(GPIOE, GPIO_PIN_3);    /* PE3 pull-UP (PTT1, active LOW) */
    gpio_config_pin(GPIOE, GPIO_PIN_2, GPIO_MODE_INPUT, GPIO_CNF_PULL);
    gpio_set_pin(GPIOE, GPIO_PIN_2);    /* PE2 pull-UP (PTT2, active LOW) */
    gpio_config_pin(GPIOE, GPIO_PIN_5, GPIO_MODE_INPUT, GPIO_CNF_PULL);
    gpio_set_pin(GPIOE, GPIO_PIN_5);    /* PE5 pull-UP (TopProg, active LOW) */
    gpio_config_pin(GPIOE, GPIO_PIN_0, GPIO_MODE_INPUT, GPIO_CNF_PULL);
    gpio_set_pin(GPIOE, GPIO_PIN_0);    /* PE0 pull-UP (PWR_DET) */
    gpio_config_pin(GPIOA, GPIO_PIN_12, GPIO_MODE_INPUT, GPIO_CNF_PULL);
    gpio_set_pin(GPIOA, GPIO_PIN_12);   /* PA12 pull-UP (BotProg, active LOW) */
    dbg_puts("[DBG] PTT+direct inputs configured\n");

    /* OEM gpio_modes_init @ 0x0801391C configures additional pins that
     * we previously omitted. These match OEM boot state per assembly. */
    gpio_config_pin(GPIOA, GPIO_PIN_2,  GPIO_MODE_INPUT, GPIO_CNF_ANALOG);
                                        /* PA2: ADC RSSI input (CH2) */
    gpio_config_pin(GPIOA, GPIO_PIN_3,  GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_clear_pin(GPIOA, GPIO_PIN_3);  /* PA3: BT UART OUT, idle LOW */
    gpio_config_pin(GPIOA, GPIO_PIN_7,  GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_clear_pin(GPIOA, GPIO_PIN_7);  /* PA7: RF scan latch, idle LOW */
    gpio_config_pin(GPIOA, GPIO_PIN_11, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(GPIOA, GPIO_PIN_11);   /* PA11: power-off control, idle HIGH */
    gpio_config_pin(GPIOA, GPIO_PIN_15, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(GPIOA, GPIO_PIN_15);   /* PA15: replay control, idle HIGH */
    gpio_config_pin(GPIOB, GPIO_PIN_2,  GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_clear_pin(GPIOB, GPIO_PIN_2);  /* PB2: BOOT1 pin, force LOW */
    gpio_config_pin(GPIOC, GPIO_PIN_5,  GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_clear_pin(GPIOC, GPIO_PIN_5);  /* PC5: RF scan enable, idle LOW */
    gpio_config_pin(GPIOC, GPIO_PIN_9,  GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_clear_pin(GPIOC, GPIO_PIN_9);  /* PC9: sideport control, idle LOW */
    /* PE6 = external PTT input (OEM: input with pull-up) */
    gpio_config_pin(GPIOE, GPIO_PIN_6,  GPIO_MODE_INPUT, GPIO_CNF_PULL);
    gpio_set_pin(GPIOE, GPIO_PIN_6);    /* PE6 pull-UP (EXT_PTT, active LOW) */
    dbg_puts("[DBG] OEM-matching GPIO pins configured\n");


    /* Boot indicator: 3 backlight blinks + beep tone.
     * Uses blocking delays since super_loop isn't running yet.
     * Confirms SysTick, DAC, and LCD backlight GPIO are working. */
    dbg_puts("[DBG] boot indicator...\n");
    {
        int i;
        for (i = 0; i < 3; i++) {
            lcd_backlight_on();
            dac_audio_play_tone(10000);  /* 1000 Hz */
            delay_ms(80);
            IWDG_FEED();
            lcd_backlight_off();
            dac_audio_stop();
            delay_ms(120);
            IWDG_FEED();
        }
        lcd_backlight_on();  /* leave backlight on */
    }
}

/* ========================================================================
 *  app_init - Initialize application-layer modules and register
 *  scheduler tasks.
 *
 *  Module init functions set up internal state. Scheduler tasks are
 *  registered for periodic polling. Tasks can be enabled/disabled at
 *  runtime via sched_enable() as peripherals are brought online.
 * ======================================================================== */


/* ----------------------------------------------------------------------
 * task_ui - UI event dispatcher
 *
 * The first consumer of the event queue. Until now every input task posted
 * events and nothing ever called event_poll(), so the queue simply filled and
 * dropped its oldest entries; input was visible only as debug UART output.
 *
 * Responsibilities, in order:
 *   1. Service the channel picker's idle timeout.
 *   2. Drain the queue, offering each input event to the picker first.
 *      The picker returns PICKER_IDLE when it is closed, so events flow
 *      through untouched when the overlay is not up.
 *
 * ui_current_ch is this layer's idea of the tuned channel. It changes ONLY on
 * an explicit commit -- that is what makes the knob a browser rather than a
 * tuner, and it is the entire point of the overlay.
 * ---------------------------------------------------------------------- */

static uint16_t ui_current_ch = 1;

/* Apply a committed selection. Kept separate so that the eventual "actually
 * retune the RF chain" call has one obvious home. */
static void ui_commit_channel(uint16_t ch_num)
{
    ui_current_ch = ch_num;
    dbg_puts("[UI] channel commit ");
    dbg_reg("", ch_num);

    channel_t ch;
    if (channel_load((uint16_t)(ch_num - 1), &ch) == 0) {
        channel_to_vfo(&ch, 0);
    }
}

static void task_ui(void)
{
    /* Timeout fires once and cancels, leaving the tuned channel untouched. */
    if (channel_picker_tick() == PICKER_CANCEL)
        dbg_puts("[UI] picker timed out\n");

    event_t ev;
    while (event_poll(&ev)) {
        switch (ev.type) {

        case EVT_ENCODER_CW:
        case EVT_ENCODER_CCW: {
            int8_t dir = (ev.type == EVT_ENCODER_CW) ? +1 : -1;

            /* The zone checklist owns the knob while it is open. */
            if (zone_browser_is_active()) {
                zone_browser_handle_encoder(dir);
                break;
            }
            channel_picker_handle_encoder(dir, ui_current_ch);
            break;
        }

        case EVT_KEY_PRESS: {
            uint8_t key = (uint8_t)ev.param;

            if (zone_browser_is_active()) {
                zone_browser_handle_key(key);
                break;
            }

            picker_result_t r = channel_picker_handle_key(key);
            if (r == PICKER_COMMIT) {
                ui_commit_channel(channel_picker_get_selection());
                break;
            }
            if (r != PICKER_IDLE) break;   /* picker consumed or cancelled */

            /* PROVISIONAL: '*' opens the zone checklist. This belongs behind a
             * Menu entry, and moves there once the menu system routes to
             * submodules -- it is a key binding so the feature is reachable and
             * testable on hardware now, not a considered choice of key. */
            if (key == KEY_STAR) {
                zone_browser_open();
                break;
            }
            break;
        }

        default:
            break;
        }
    }
}

static void app_init(void)
{
    /* Boot splash (blocks ~1 s while LCD shows image) ---------------- */
    dbg_puts("[DBG] splash_show...\n");
    splash_show();
    IWDG_FEED();

    /* Input devices -------------------------------------------------- */
    dbg_puts("[DBG] keypad+encoder...\n");
    keypad_init();
    encoder_init();
    IWDG_FEED();

    /* Persistent settings (needs WL_SYSCFG ready) -------------------- */
    dbg_puts("[DBG] settings_init...\n");
    settings_init();
    IWDG_FEED();

    /* Radio subsystems (software init only - RF hardware skipped) ------ */
    dbg_puts("[DBG] radio_init...\n");
    radio_init();   /* radio_init → vfo_init internally */
    IWDG_FEED();
    dbg_puts("[DBG] vox+scanner+dtmf...\n");
    vox_init();
    scanner_init();
    dtmf_load_config();
    IWDG_FEED();
    dbg_puts("[DBG] audio_init...\n");
    audio_init();
    audio_power_on();
    IWDG_FEED();

    /* Auxiliary modules ---------------------------------------------- */
    dbg_puts("[DBG] gps+power_init...\n");
    gps_init();
    power_init();
    IWDG_FEED();

    /* Apply factory calibration to power thresholds (if cal data valid) */
    /* NOTE: OEM V0.27 does NOT read battery thresholds from flash 0xF200.
     * Battery thresholds may be hardcoded or in a different calibration block
     * (possibly 0xF0C0 or 0xF0D0). Using hardcoded defaults for now. */
    (void)cal_data.validity_flag; /* suppress unused warning */

    dbg_puts("[DBG] menu+aprs+am...\n");
    menu_init();
    aprs_init();
    am_radio_init();
    crossband_set_mode((xband_mode_t)settings_get()->crossband_mode);
    IWDG_FEED();

    /* Display last - shows fully-initialized state ------------------- */
    /* Zone filter before display: the main screen shows the zone name of the
     * tuned channel, and the picker needs the mask to scope its list. */
    dbg_puts("[DBG] zone_filter_init...\n");
    zone_filter_init();
    IWDG_FEED();

    dbg_puts("[DBG] display_init...\n");
    display_init();
    IWDG_FEED();

    /* CPS protocol handler (UART4, always ready).
     * In DEBUG builds, UART4 is used for debug output - skip CPS to
     * avoid the CPS module interpreting debug bytes as handshake. */
#ifndef DEBUG_UART
    dbg_puts("[DBG] cps_init...\n");
    cps_init();
#else
    dbg_puts("[DBG] cps_init SKIPPED (DEBUG: UART4 shared)\n");
#endif
    IWDG_FEED();

    /* ================================================================
     * Register scheduler tasks.
     * Tasks are dispatched in registration order by sched_run().
     * Critical high-frequency tasks are registered first.
     * ================================================================ */
    dbg_puts("[DBG] registering scheduler tasks...\n");

    sched_register("encoder",   task_encoder,          TICK_ENCODER_MS);
    sched_register("keypad",    task_keypad,           TICK_KEYPAD_MS);
    sched_register("buttons",   task_buttons,          TICK_BUTTONS_MS);
    sched_register("pwr_btn",   power_button_poll,     TICK_POWER_BTN_MS);
    sched_register("display",   display_update,        TICK_DISPLAY_MS);
    sched_register("gps",       task_gps,              TICK_GPS_MS);
    sched_register("dualwatch", radio_dual_watch_poll, TICK_DUALWATCH_MS);
    sched_register("battery",   power_poll,            TICK_BATTERY_MS);
    sched_register("audio",     audio_poll,            TICK_AUDIO_MS);
    sched_register("cps",       task_cps,              TICK_CPS_MS);
    sched_register("ui",        task_ui,               TICK_UI_MS);

    dbg_puts("[DBG] app_init complete\n");
}

#endif /* HW_TEST */
