/*
 * hw_test.c - Hardware test suite for RT-950 Pro bring-up
 *
 * Each test is standalone and exercises one peripheral subsystem.
 *
 * Output goes to UART4 (PC10/PC11) at 115200 baud -- the programming cable.
 *
 * These helpers used to write to USART1, the Bluetooth port, which is NOT
 * connected to the programming cable. Every test printed diagnostics that
 * nothing could receive, so the whole suite looked mute on a bench setup and
 * a working test was indistinguishable from a dead radio. They deliberately
 * shadow the global dbg_* from debug_uart.h, so the mistake was invisible at
 * the call sites.
 *
 * Build with:  make test TEST=N
 * Flash via:   tools/encrypt_btf.py to create flashable BTF
 */

#include "at32f403a.h"
#include "rt950_pinmap.h"
#include "tests/hw_test.h"
#include "drivers/gpio.h"
#include "drivers/dma.h"
#include "drivers/spi.h"
#include "drivers/bk4829.h"
#include "drivers/si4732.h"
#include "drivers/uart.h"
#include "drivers/adc.h"
#include "drivers/dac_audio.h"
#include "drivers/lcd.h"
#include "app/display.h"
#include "app/font.h"
#include "app/keypad.h"
#include "app/update_listener.h"
#include "app/encoder.h"
#include "app/gps.h"
#include "app/power.h"

extern void delay_ms(uint32_t ms);
extern uint32_t get_tick(void);

/* Bring up UART4 for test output. dbg_init() only runs in DEBUG builds, so the
 * tests cannot rely on it; uart_cps_init() configures the same peripheral at
 * 115200 either way. Safe to call twice. */
static void test_debug_init(void)
{
    uart_acc_init();   /* UART4 on PC10/PC11 -- the programming cable */
}


/* ---------------------------------------------------------------------------
 * On-screen debug console
 *
 * Mirrors every debug line to the LCD as well as the UART. The point is to
 * make the firmware's INTENT observable without depending on the serial link:
 * if the screen shows the counter climbing but the cable stays silent, the
 * fault is isolated to UART TX and the firmware is fine. Without this, a mute
 * cable and a dead radio look identical -- which has already cost several
 * test cycles.
 * ------------------------------------------------------------------------- */

#define CON_ROWS      18
#define CON_ROW_H     12
#define CON_TOP       84          /* below the colour bars */
#define CON_COLS      38

static char     con_line[CON_COLS + 1];
static uint8_t  con_pos;
static uint8_t  con_row;
static uint8_t  con_ready;        /* only draw once lcd_init() has run */

static void con_enable(void)   __attribute__((unused));
static void con_enable(void)   { con_ready = 1; con_row = 0; con_pos = 0; }

static void con_flush(void)
{
    if (!con_ready) { con_pos = 0; return; }
    con_line[con_pos] = '\0';

    /* Wrap by wiping the console area, so old text never overlaps new. */
    if (con_row >= CON_ROWS) {
        con_row = 0;
        lcd_fill_rect(0, CON_TOP, LCD_WIDTH,
                      (uint16_t)(CON_ROWS * CON_ROW_H), COLOR_BLACK);
    }

    /* Clear this row first: shorter lines would otherwise leave a tail. */
    lcd_fill_rect(0, (uint16_t)(CON_TOP + con_row * CON_ROW_H),
                  LCD_WIDTH, CON_ROW_H, COLOR_BLACK);
    font_draw_string(FONT_SMALL, 2,
                     (uint16_t)(CON_TOP + con_row * CON_ROW_H),
                     con_line, COLOR_WHITE, COLOR_BLACK);
    con_row++;
    con_pos = 0;
}

static void con_putc(char c)
{
    if (c == '\n') { con_flush(); return; }
    if (c == '\r') return;
    if (con_pos < CON_COLS) con_line[con_pos++] = c;
}

/* Debug output helpers ------------------------------------------------ */

static void dbg_puts(const char *s)
{
    while (*s) {
        uart_send_byte(UART4, (uint8_t)*s);
        con_putc(*s);
        s++;
    }
}

static void dbg_newline(void)
{
    uart_send_byte(UART4, '\r');
    uart_send_byte(UART4, '\n');
    con_flush();
}

static void dbg_println(const char *s)
{
    dbg_puts(s);
    dbg_newline();
}

static void dbg_hex8(uint8_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    uart_send_byte(UART4, hex[v >> 4]);
    uart_send_byte(UART4, hex[v & 0xF]);
    con_putc(hex[v >> 4]);
    con_putc(hex[v & 0xF]);
}

static void dbg_hex16(uint16_t v)
{
    dbg_hex8((uint8_t)(v >> 8));
    dbg_hex8((uint8_t)(v & 0xFF));
}

static void dbg_dec(uint32_t v)
{
    char buf[12];
    int i = 0;
    if (v == 0) {
        uart_send_byte(UART4, '0');
        con_putc('0');
        return;
    }
    while (v > 0) {
        buf[i++] = '0' + (char)(v % 10);
        v /= 10;
    }
    while (i > 0) {
        char c = buf[--i];
        uart_send_byte(UART4, (uint8_t)c);
        con_putc(c);
    }
}

/* ==========================================================================
 *  TEST 1: Blinky - Toggle LCD backlight at 1 Hz
 *  Confirms: GPIO output, clock init, SysTick
 * ========================================================================== */

__attribute__((unused))
static void draw_kv(uint16_t y, const char *label, uint32_t v, uint16_t col)
{
    char b[32]; uint8_t n = 0;
    while (*label) b[n++] = *label++;
    char t[12]; int ti = 0;
    if (!v) t[ti++] = '0';
    while (v) { t[ti++] = (char)('0' + (v % 10)); v /= 10; }
    while (ti) b[n++] = t[--ti];
    b[n++] = ' '; b[n++] = ' '; b[n] = '\0';
    lcd_draw_string(6, y, b, col, 0x0000);
}

__attribute__((unused))
static void draw_hex(uint16_t y, const char *label, uint32_t v, uint16_t col)
{
    static const char H[] = "0123456789ABCDEF";
    char b[32]; uint8_t n = 0;
    while (*label) b[n++] = *label++;
    b[n++] = H[(v >> 4) & 0xF];
    b[n++] = H[v & 0xF];
    b[n++] = ' '; b[n++] = ' '; b[n] = '\0';
    lcd_draw_string(6, y, b, col, 0x0000);
}

void test_blinky(void)
{
    test_debug_init();

    gpio_config_pin(LCD_BL_PORT, LCD_BL_PIN, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(LCD_BL_PORT, LCD_BL_PIN);
    lcd_init();
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, 0x0000);

    lcd_draw_string(6,  4, "POWER SWITCH TEST", 0xFFE0, 0x0000);
    lcd_draw_string(6, 18, "turn volume knob OFF", 0x07FF, 0x0000);
    dbg_println("POWER SWITCH TEST (PE0 / PB9 latch)");

    /* The power latch must be held or the radio drops dead immediately. main()
     * asserts PB9 before anything else; keep it asserted here too. */
    gpio_config_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(GPIO_PB9_PWREN_PORT, GPIO_PB9_PWREN_PIN);

    power_init();

    /* Power-off is implemented in power.c but is registered as a SCHEDULER
     * task, and the hardware tests never run the scheduler -- which is why the
     * volume knob appears dead in every test build. Polling it here at the
     * same 20 ms rate the scheduler would gives the tests the same behaviour
     * as the real firmware.
     *
     * PE0 reads LOW with the switch ON (grounded) and HIGH when turned OFF.
     * power_button_poll() requires 1.5 s of sustained OFF before releasing the
     * PB9 latch, so a brief wobble while turning the volume will not shut the
     * radio down. */
    uint32_t hb = 0;
    while (1) {
        /* power_button_poll() is DELIBERATELY NOT CALLED.
         *
         * Power-off works -- PE0 is detected and releasing the PB9 latch cuts
         * the supply. But the radio will not come back on afterwards, even with
         * PB9 asserted in Reset_Handler, which is the earliest instruction the
         * CPU can execute. That rules out latch timing: the knob is not
         * restoring power at all, and recovery needs the battery out.
         *
         * A power-off that costs a battery pull to undo is worse than none, so
         * the test builds observe the switch without acting on it. The real
         * firmware still registers power_button_poll() as a scheduler task; this
         * only affects HW_TEST. */

        if ((hb % 25u) == 0u) {
            uint8_t pe0 = (PWR_SWITCH_PORT->IDR & PWR_SWITCH_PIN) ? 1 : 0;
            uint8_t pb9 = (GPIO_PB9_PWREN_PORT->ODR & GPIO_PB9_PWREN_PIN) ? 1 : 0;
            draw_kv(44, "PE0 (sw): ", pe0, pe0 ? 0xF800 : 0x07E0);
            draw_kv(58, "  0=ON 1=OFF", 0, 0x07FF);
            draw_kv(78, "PB9 latch:", pb9, pb9 ? 0x07E0 : 0xF800);
            draw_kv(98, "seconds:  ", hb / 50u, 0xFFFF);

            dbg_puts("PE0=");  dbg_dec(pe0);
            dbg_puts(" PB9="); dbg_dec(pb9);
            dbg_newline();
        }
        hb++;
        delay_ms(20);
    }
}

/* ==========================================================================
 *  TEST 2: UART Echo - Echo bytes on BT (USART1), monitor GPS (USART3)
 *  Confirms: USART1 TX/RX, USART3 RX
 * ========================================================================== */

void test_uart_echo(void)
{
    uart_bt_init();
    test_debug_init();
    uart_gps_init();
    dbg_println("=== TEST 2: UART Echo ===");
    dbg_println("BT(USART1): echo mode  |  GPS(USART3): passthrough to BT");

    while (1) {
        /* Echo BT input */
        if (uart_bt_rx_available()) {
            uint8_t c = uart_bt_rx_read();
            uart_send_byte(UART4, c);
        }
        /* Forward GPS to BT */
        if (gps_rx_available()) {
            uint8_t c = gps_rx_read();
            uart_send_byte(UART4, c);
        }
    }
}

/* ==========================================================================
 *  TEST 3: LCD Test Pattern - Fill screen with color bars
 *  Confirms: LCD 8080 bus, reset, init sequence, pixel write
 * ========================================================================== */

void test_lcd_pattern(void)
{
    uart_bt_init();
    test_debug_init();
    dbg_println("=== TEST 3: LCD Test Pattern ===");

    /* LCD backlight on */
    gpio_config_pin(LCD_BL_PORT, LCD_BL_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(LCD_BL_PORT, LCD_BL_PIN);

    lcd_init();
    dbg_println("LCD init done");

    /* RGB565 color bars: Red, Green, Blue, White, Black, Yellow, Magenta, Cyan */
    static const uint16_t colors[] = {
        0xF800, 0x07E0, 0x001F, 0xFFFF, 0x0000, 0xFFE0, 0xF81F, 0x07FF,
    };
    #define NUM_COLORS 8
    #define BAR_HEIGHT (LCD_HEIGHT / NUM_COLORS)

    uint32_t pass = 0;
    while (1) {
        dbg_puts("Drawing pattern #");
        dbg_dec(pass++);
        dbg_newline();

        for (int bar = 0; bar < NUM_COLORS; bar++) {
            uint16_t y_start = (uint16_t)(bar * BAR_HEIGHT);
            lcd_fill_rect(0, y_start, LCD_WIDTH, BAR_HEIGHT, colors[bar]);
        }

        dbg_println("Pattern drawn. Waiting 3s...");
        delay_ms(3000);
    }
    #undef NUM_COLORS
    #undef BAR_HEIGHT
}

/* ==========================================================================
 *  TEST 4: BK4829 Chip ID - Read register 0x00 from both transceivers
 *  Confirms: GPIOE bit-bang SPI, chip presence
 * ========================================================================== */

void test_bk4829_id(void)
{
    uart_bt_init();
    test_debug_init();
    dbg_println("=== TEST 4: BK4829 Chip ID ===");

    bk4829_init(BK4829_CHIP0);
    bk4829_init(BK4829_CHIP1);

    while (1) {
        uint16_t id0 = bk4829_read_reg(BK4829_CHIP0, 0x00);
        uint16_t id1 = bk4829_read_reg(BK4829_CHIP1, 0x00);

        dbg_puts("Chip0 REG[0x00] = 0x");
        dbg_hex16(id0);
        dbg_puts("  Chip1 REG[0x00] = 0x");
        dbg_hex16(id1);
        dbg_newline();

        /* Also read RSSI (reg 0x67) */
        uint16_t rssi0 = bk4829_read_reg(BK4829_CHIP0, 0x67);
        uint16_t rssi1 = bk4829_read_reg(BK4829_CHIP1, 0x67);
        dbg_puts("Chip0 RSSI=0x");
        dbg_hex16(rssi0);
        dbg_puts("  Chip1 RSSI=0x");
        dbg_hex16(rssi1);
        dbg_newline();
        dbg_newline();

        delay_ms(1000);
    }
}

/* ==========================================================================
 *  TEST 5: SI4732 Chip Revision - Read GET_REV response
 *  Confirms: Bit-bang I2C on PB6/PB7, chip presence
 * ========================================================================== */

void test_si4732_rev(void)
{
    uart_bt_init();
    test_debug_init();
    dbg_println("=== TEST 5: SI4732 Revision ===");

    si4732_init();

    /* Power up in FM mode first */
    dbg_println("Powering up SI4732 in FM mode...");
    si4732_power_up_fm();
    delay_ms(500);

    uint8_t part_number = 0;
    si4732_get_rev(&part_number);

    dbg_puts("Part Number : 0x");
    dbg_hex8(part_number);
    dbg_newline();
    dbg_puts(part_number == 0x20 ? "  -> SI4732 detected" :
             part_number == 0x00 ? "  -> FAIL (no response)" :
             "  -> Unexpected PN (check I2C)");
    dbg_newline();

    /* Tune to a known FM station as additional test */
    dbg_println("\nTuning to 100.0 MHz FM...");
    si4732_fm_tune(10000);
    delay_ms(500);

    struct si4732_tune_status status;
    si4732_fm_tune_status(&status);
    dbg_puts("Freq: ");
    dbg_dec(status.freq);
    dbg_puts("0 kHz  RSSI: ");
    dbg_dec(status.rssi);
    dbg_puts("  SNR: ");
    dbg_dec(status.snr);
    dbg_newline();

    while (1) {
        delay_ms(2000);
        si4732_fm_tune_status(&status);
        dbg_puts("RSSI=");
        dbg_dec(status.rssi);
        dbg_puts(" SNR=");
        dbg_dec(status.snr);
        dbg_newline();
    }
}

/* ==========================================================================
 *  TEST 6: SPI Flash JEDEC ID - Read 3-byte manufacturer/device ID
 *  Confirms: SPI2 HW, flash chip present
 * ========================================================================== */

void test_spi_flash_id(void)
{
    uart_bt_init();
    test_debug_init();
    dbg_println("=== TEST 6: SPI Flash JEDEC ID ===");

    spi2_init();

    while (1) {
        /* Use the built-in flash read ID function */
        uint32_t jedec = spi_flash_read_id();
        uint8_t mfr  = (uint8_t)(jedec >> 16);
        uint8_t type = (uint8_t)(jedec >> 8);
        uint8_t cap  = (uint8_t)(jedec);

        dbg_puts("JEDEC ID: Mfr=0x");
        dbg_hex8(mfr);
        dbg_puts(" Type=0x");
        dbg_hex8(type);
        dbg_puts(" Cap=0x");
        dbg_hex8(cap);

        /* Decode common manufacturers */
        dbg_puts("  -> ");
        if (mfr == 0xEF) dbg_puts("Winbond");
        else if (mfr == 0xC8) dbg_puts("GigaDevice");
        else if (mfr == 0x20) dbg_puts("Micron/Numonyx");
        else if (mfr == 0x1F) dbg_puts("Adesto/Atmel");
        else if (mfr == 0x01) dbg_puts("Spansion/Cypress");
        else if (mfr == 0xBF) dbg_puts("SST");
        else if (mfr == 0x9D) dbg_puts("ISSI");
        else if (mfr == 0x0B) dbg_puts("XTX");
        else dbg_puts("Unknown");

        dbg_puts(" ");
        uint32_t size_kb = (1UL << cap) / 1024;
        dbg_dec(size_kb);
        dbg_puts("KB");
        dbg_newline();

        if (mfr == 0xFF || mfr == 0x00) {
            dbg_println("  !! No flash detected (check SPI2 wiring)");
        }

        delay_ms(2000);
    }
}

/* ==========================================================================
 *  TEST 7: ADC Monitor - Continuously read battery and audio level
 *  Confirms: ADC2, PA0 (battery), PA1 (audio)
 * ========================================================================== */

void test_adc_monitor(void)
{
    uart_bt_init();
    test_debug_init();
    dbg_println("=== TEST 7: ADC Monitor ===");

    adc_init();

    while (1) {
        uint8_t batt = adc_read_battery();
        uint8_t audio = adc_read_audio_level();

        dbg_puts("Battery(PA0)=");
        dbg_dec(batt);
        dbg_puts("/255  Audio(PA1)=");
        dbg_dec(audio);
        dbg_puts("/255");

        /* Rough voltage estimate: assuming 1:2 divider, 3.3V ref */
        uint32_t mv = (uint32_t)batt * 3300 * 2 / 4095;
        dbg_puts("  ~");
        dbg_dec(mv);
        dbg_puts("mV");
        dbg_newline();

        delay_ms(500);
    }
}

/* ==========================================================================
 *  TEST 8: DAC Tone - Output 1 kHz test tone on PA4
 *  Confirms: DAC1, TIM6, DMA2 CH3
 * ========================================================================== */

void test_dac_tone(void)
{
    uart_bt_init();
    test_debug_init();
    dma_init();
    dbg_println("=== TEST 8: DAC 1kHz Tone on PA4 ===");

    dac_audio_init();

    /* 1000.0 Hz = 10000 in freqx10 format */
    dbg_println("Playing 1000 Hz tone...");
    dac_audio_play_tone(10000);

    uint32_t sec = 0;
    while (1) {
        delay_ms(5000);
        sec += 5;
        dbg_puts("Playing for ");
        dbg_dec(sec);
        dbg_puts("s  DMA active=");
        dbg_dec((uint32_t)dac_audio_is_playing());
        dbg_newline();

        /* Cycle through tones every 10 seconds */
        if ((sec % 20) == 10) {
            dbg_println("Switching to 67.0 Hz (CTCSS)...");
            dac_audio_play_tone(670);
        } else if ((sec % 20) == 0) {
            dbg_println("Switching to 1000 Hz...");
            dac_audio_play_tone(10000);
        }
    }
}

/* ==========================================================================
 *  TEST 9: Keypad + Encoder - Scan and report events
 *  Confirms: Matrix scan (PC0-3/PD4-7), encoder (PB4/PB5)
 * ========================================================================== */

void test_keypad_encoder(void)
{
    uart_bt_init();
    test_debug_init();
    dbg_println("=== TEST 9: Keypad + Encoder ===");
    dbg_println("Press keys or turn encoder. Output on USART1.");

    keypad_init();
    encoder_init();

    static const char * const key_names[] = {
        "1", "2", "3", "A/VFO",
        "4", "5", "6", "B/SCAN",
        "7", "8", "9", "C/MENU",
        "*", "0", "#", "D/BAND",
        "PTT", "SIDE1", "SIDE2", "SIDE3"
    };

    while (1) {
        /* Keypad events */
        key_event_t evt;
        if (keypad_get_event(&evt)) {
            dbg_puts("KEY: ");
            if (evt.key < 20)
                dbg_puts(key_names[evt.key]);
            else {
                dbg_puts("0x");
                dbg_hex8(evt.key);
            }
            switch (evt.type) {
            case KEY_EVT_PRESS:   dbg_puts(" PRESS");   break;
            case KEY_EVT_REPEAT:  dbg_puts(" REPEAT");  break;
            case KEY_EVT_RELEASE: dbg_puts(" RELEASE"); break;
            default:              dbg_puts(" ???");      break;
            }
            dbg_newline();
        }

        /* Encoder */
        int8_t enc = encoder_poll();
        if (enc > 0) {
            dbg_println("ENC: CW  (+1)");
        } else if (enc < 0) {
            dbg_println("ENC: CCW (-1)");
        }

        delay_ms(5);  /* ~200 Hz poll rate */
    }
}

/* ==========================================================================
 *  TEST 10: GPS Display - Show NMEA sentences and parsed data
 *  Confirms: USART3 RX from GPS module, NMEA parsing
 * ========================================================================== */

void test_gps_display(void)
{
    uart_bt_init();
    test_debug_init();
    uart_gps_init();
    dbg_println("=== TEST 10: GPS NMEA Display ===");
    dbg_println("Raw NMEA forwarded to USART1. Parsed data every 2s.");

    gps_init();
    uint32_t last_print = get_tick();

    while (1) {
        /* Forward raw NMEA bytes to debug port */
        if (gps_rx_available()) {
            uint8_t c = gps_rx_read();
            uart_send_byte(UART4, c);
        }

        /* Periodic parsed data dump */
        gps_process();
        uint32_t now = get_tick();
        if ((now - last_print) >= 2000) {
            last_print = now;
            const gps_data_t *gps = gps_get_data();
            dbg_newline();
            dbg_puts("[GPS] Fix=");
            dbg_dec(gps->fix_quality);
            dbg_puts(" Sats=");
            dbg_dec(gps->num_satellites);
            dbg_puts(" Time=");
            dbg_dec(gps->hour);
            dbg_puts(":");
            if (gps->minute < 10) dbg_puts("0");
            dbg_dec(gps->minute);
            dbg_puts(":");
            if (gps->second < 10) dbg_puts("0");
            dbg_dec(gps->second);
            dbg_newline();
        }
    }
}

/* ==========================================================================
 *  TEST 11: Full System Diagnostic - Init all peripherals, report status
 *  Confirms: Complete system bring-up sequence
 * ========================================================================== */

void test_full_diagnostic(void)
{
    uart_bt_init();
    test_debug_init();
    dbg_println("========================================");
    dbg_println("  RT-950 Pro Custom Firmware Diagnostic");
    dbg_println("  Build: " __DATE__ " " __TIME__);
    dbg_println("========================================");
    dbg_newline();

    /* Clock info ---------------------------------------------------- */
    dbg_puts("SYSCLK: 120 MHz (HSE 8 MHz x 15)");
    dbg_newline();
    dbg_puts("SysTick: ");
    dbg_dec(get_tick());
    dbg_puts(" ms since boot");
    dbg_newline();
    dbg_newline();

    /* DMA ----------------------------------------------------------- */
    dbg_puts("[DMA ] Init... ");
    dma_init();
    dbg_println("OK");

    /* SPI Flash ----------------------------------------------------- */
    dbg_puts("[FLASH] SPI2 init... ");
    spi2_init();
    uint32_t jedec = spi_flash_read_id();
    uint8_t flash_mfr  = (uint8_t)(jedec >> 16);
    uint8_t flash_type = (uint8_t)(jedec >> 8);
    uint8_t flash_cap  = (uint8_t)(jedec);
    dbg_puts("JEDEC=");
    dbg_hex8(flash_mfr);
    dbg_hex8(flash_type);
    dbg_hex8(flash_cap);
    if (flash_mfr != 0xFF && flash_mfr != 0x00) {
        dbg_puts(" OK (");
        dbg_dec((1UL << flash_cap) / 1024);
        dbg_puts("KB)");
    } else {
        dbg_puts(" FAIL");
    }
    dbg_newline();

    /* BK4829 -------------------------------------------------------- */
    dbg_puts("[RF  ] BK4829 #0 init... ");
    bk4829_init(BK4829_CHIP0);
    uint16_t rf_id0 = bk4829_read_reg(BK4829_CHIP0, 0x00);
    dbg_puts("REG0=0x");
    dbg_hex16(rf_id0);
    dbg_puts(rf_id0 != 0xFFFF && rf_id0 != 0x0000 ? " OK" : " FAIL");
    dbg_newline();

    dbg_puts("[RF  ] BK4829 #1 init... ");
    bk4829_init(BK4829_CHIP1);
    uint16_t rf_id1 = bk4829_read_reg(BK4829_CHIP1, 0x00);
    dbg_puts("REG0=0x");
    dbg_hex16(rf_id1);
    dbg_puts(rf_id1 != 0xFFFF && rf_id1 != 0x0000 ? " OK" : " FAIL");
    dbg_newline();

    /* SI4732 -------------------------------------------------------- */
    dbg_puts("[SI47] Init + power up FM... ");
    si4732_init();
    si4732_power_up_fm();
    delay_ms(500);
    uint8_t si_pn = 0;
    si4732_get_rev(&si_pn);
    dbg_puts("PN=0x");
    dbg_hex8(si_pn);
    dbg_puts(si_pn == 0x20 ? " OK (SI4732)" :
             si_pn == 0x00 ? " FAIL (no response)" : " ? (unexpected PN)");
    dbg_newline();

    /* ADC ----------------------------------------------------------- */
    dbg_puts("[ADC ] Init + read... ");
    adc_init();
    uint8_t batt = adc_read_battery();
    uint8_t audio = adc_read_audio_level();
    dbg_puts("Batt=");
    dbg_dec(batt);
    dbg_puts(" Audio=");
    dbg_dec(audio);
    dbg_puts(batt > 0 ? " OK" : " FAIL");
    dbg_newline();

    /* GPS UART ------------------------------------------------------ */
    dbg_puts("[GPS ] USART3 init... ");
    uart_gps_init();
    dbg_puts("listening...");
    dbg_newline();

    /* Wait up to 3 seconds for GPS data */
    uint32_t start = get_tick();
    int gps_ok = 0;
    while ((get_tick() - start) < 3000) {
        if (gps_rx_available()) {
            gps_ok = 1;
            break;
        }
    }
    dbg_puts("[GPS ] Data: ");
    dbg_println(gps_ok ? "RECEIVED" : "NO DATA (may need longer warm-up)");

    /* LCD ------------------------------------------------------------ */
    dbg_puts("[LCD ] Init... ");
    gpio_config_pin(LCD_BL_PORT, LCD_BL_PIN,
                    GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(LCD_BL_PORT, LCD_BL_PIN);
    lcd_init();
    dbg_println("OK (visual check needed)");

    /* Keypad -------------------------------------------------------- */
    dbg_puts("[KBD ] Init... ");
    keypad_init();
    dbg_println("OK");

    /* Encoder ------------------------------------------------------- */
    dbg_puts("[ENC ] Init... ");
    encoder_init();
    dbg_println("OK");

    /* DAC ------------------------------------------------------------ */
    dbg_puts("[DAC ] Init... ");
    dac_audio_init();
    dbg_println("OK");

    /* Summary ------------------------------------------------------- */
    dbg_newline();
    dbg_println("========================================");
    dbg_println("  Diagnostic complete.");
    dbg_println("  Connect scope to PA4 for DAC test.");
    dbg_println("  Check LCD for test pattern.");
    dbg_println("  Turn encoder / press keys for input test.");
    dbg_println("========================================");
    dbg_newline();

    /* Fill LCD with a gradient test pattern */
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, 0x001F); /* blue fill */

    /* Enter interactive mode: show keypad/encoder on debug + LCD */
    dbg_println("Entering interactive mode (keys + encoder)...");
    while (1) {
        key_event_t evt;
        if (keypad_get_event(&evt) && evt.type == KEY_EVT_PRESS) {
            dbg_puts("KEY ");
            dbg_dec(evt.key);
            dbg_newline();
        }
        int8_t enc = encoder_poll();
        if (enc != 0) {
            dbg_puts("ENC ");
            dbg_dec(enc > 0 ? 1 : (uint32_t)-1);
            dbg_newline();
        }
        delay_ms(5);
    }
}
