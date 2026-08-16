/*
 * lcd.c - 8080-parallel LCD driver for the RT-950 Pro
 *
 * Controller: ST7789V (240x320 IPS panel)
 * Init sequence and bus protocol from V0.27 firmware binary.
 *
 * Bus wiring (ALL on GPIOD, verified from firmware):
 *   Data:  PD8-PD15  (D0-D7 on upper byte of GPIOD)
 *   WR:    PD0       (write strobe, active-low pulse)
 *   CS:    PD1       (chip select, active-low)
 *   RST:   PD2       (hardware reset, active-low) - corrected from PC14
 *   D/C:   PD3       (0 = command, 1 = data)
 *
 * Write timing from V0.27 binary (LCD_WriteCommand @ fw 0x080267B8):
 *   1. Set D/C
 *   2. Assert CS low
 *   3. Place data on PD8-PD15
 *   4. Pulse WR low then high
 *   5. Deassert CS high
 */

#include "drivers/lcd.h"
#include "drivers/gpio.h"
#include "rt950_pinmap.h"

extern void delay_ms(uint32_t ms);

/* Internal: put 8-bit value on data bus (PD8-PD15) ------------------ */
static inline void lcd_set_data(uint8_t byte)
{
    /*
     * Drive PD8-PD15 atomically via the set/clear registers.
     *
     * This was a read-modify-write of the whole ODR: read all 16 pins, mask the
     * data byte in, write all 16 back. That silently reverts any change another
     * context made to GPIOD between the read and the write -- and the keypad
     * rows are PD4-PD7 on this very port, as are WR (PD0), CS (PD1) and DC
     * (PD3). It survives only while nothing else touches GPIOD, which stops
     * being true the moment the keypad is scanned.
     *
     * CLR then SCR is two atomic writes that affect only the data pins, so no
     * other pin state can be lost, and no interrupt window exists at all.
     */
    LCD_DATA_PORT->CLR = (0xFFUL << LCD_DATA_SHIFT);
    if (byte)
        LCD_DATA_PORT->SCR = ((uint32_t)byte << LCD_DATA_SHIFT);
}

/* Internal: pulse WR low -> high ------------------------------------- */
static inline void lcd_pulse_wr(void)
{
    /*
     * Widened from 4 NOPs (~33 ns at 120 MHz) and given explicit setup and
     * hold time.
     *
     * 33 ns with no data setup is marginal for an 8080-bus panel, and marginal
     * timing here does more than blur pixels: a strobe missed or doubled during
     * lcd_set_window() shifts the command stream, so the ADDRESS WINDOW lands
     * somewhere else and every subsequent pixel is written to the wrong part of
     * the panel. That is what corrupted text drawn earlier at boot -- writes
     * were escaping their intended rectangle entirely.
     *
     * The data pins are set by the caller immediately before this, so the NOPs
     * before WR falls are the setup time; those after it are the low-pulse
     * width. Cheap insurance: the whole frame is a few milliseconds either way.
     */
    __asm volatile ("nop\n nop\n nop\n nop\n");            /* data setup */
    gpio_clear_pin(LCD_WR_PORT, LCD_WR_PIN);                  /* WR low */
    __asm volatile ("nop\n nop\n nop\n nop\n"
                    "nop\n nop\n nop\n nop\n"
                    "nop\n nop\n nop\n nop\n");            /* ~100 ns low */
    gpio_set_pin(LCD_WR_PORT, LCD_WR_PIN);                    /* WR high, latch */
    __asm volatile ("nop\n nop\n nop\n nop\n");            /* hold */
}

/* ========================================================================
 *  lcd_write_command - Send a command byte (D/C = 0).
 *  V0.27 LCD_WriteCmd @ 0x080267B8: CS LOW, DC LOW, WR strobe, data on PD8-15.
 * ======================================================================== */

void lcd_write_command(uint8_t cmd)
{
    gpio_clear_pin(LCD_DC_PORT, LCD_DC_PIN);     /* D/C = command */
    gpio_clear_pin(LCD_CS_PORT, LCD_CS_PIN);     /* CS assert */
    lcd_set_data(cmd);
    lcd_pulse_wr();
    gpio_set_pin(LCD_CS_PORT, LCD_CS_PIN);       /* CS deassert */
}

/* ========================================================================
 *  lcd_write_data - Send a data byte (D/C = 1).
 *  V0.27 LCD_WriteData @ 0x08026818: CS LOW, DC HIGH, WR strobe, data on PD8-15.
 * ======================================================================== */

void lcd_write_data(uint8_t data)
{
    gpio_set_pin(LCD_DC_PORT, LCD_DC_PIN);       /* D/C = data */
    gpio_clear_pin(LCD_CS_PORT, LCD_CS_PIN);     /* CS assert */
    lcd_set_data(data);
    lcd_pulse_wr();
    gpio_set_pin(LCD_CS_PORT, LCD_CS_PIN);       /* CS deassert */
}

/* ========================================================================
 *  lcd_set_window - Set column and row address range for pixel writes.
 *
 *  Column Address Set (0x2A): x1..x2
 *  Row Address Set    (0x2B): y1..y2
 * ======================================================================== */

void lcd_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    lcd_write_command(0x2A);    /* Column Address Set */
    lcd_write_data(x1 >> 8);
    lcd_write_data(x1 & 0xFF);
    lcd_write_data(x2 >> 8);
    lcd_write_data(x2 & 0xFF);

    lcd_write_command(0x2B);    /* Row Address Set */
    lcd_write_data(y1 >> 8);
    lcd_write_data(y1 & 0xFF);
    lcd_write_data(y2 >> 8);
    lcd_write_data(y2 & 0xFF);
}

/* ========================================================================
 *  lcd_gram_write - Issue Memory Write command (0x2C).
 *  Follow with lcd_write_data() to push pixel data.
 * ======================================================================== */

void lcd_gram_write(void)
{
    lcd_write_command(0x2C);
}

/* ========================================================================
 *  lcd_panel_reset - Hardware reset via PD2 (active-low).
 *  V0.27 @ 0x08026954: HIGH->LOW(1ms)->HIGH then 120ms delay.
 * ======================================================================== */

void lcd_panel_reset(void)
{
    gpio_set_pin(LCD_RST_PORT, LCD_RST_PIN);       /* start HIGH */
    delay_ms(1);                                    /* V0.27: movs r0, 1 */
    gpio_clear_pin(LCD_RST_PORT, LCD_RST_PIN);     /* assert reset */
    delay_ms(1);                                    /* V0.27: movs r0, 1 */
    gpio_set_pin(LCD_RST_PORT, LCD_RST_PIN);       /* release reset */
    delay_ms(120);                                  /* V0.27: movs r0, 0x78 */
}

/* ========================================================================
 *  lcd_backlight_on / lcd_backlight_off
 *
 *  OEM toggles both PC6 and PB3 together @ 0x08017C40.
 *  PC6 = primary backlight enable, PB3 = secondary driver enable.
 * ======================================================================== */

void lcd_backlight_on(void)
{
    gpio_set_pin(LCD_BL_PORT, LCD_BL_PIN);
    gpio_set_pin(LCD_BL_SEC_PORT, LCD_BL_SEC_PIN);  /* PB3 secondary enable */
}

void lcd_backlight_off(void)
{
    gpio_clear_pin(LCD_BL_PORT, LCD_BL_PIN);
    gpio_clear_pin(LCD_BL_SEC_PORT, LCD_BL_SEC_PIN);
}

/* ========================================================================
 *  lcd_fill_rect - Fill a rectangle with a solid 16-bit RGB565 color.
 * ======================================================================== */

void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    lcd_set_window(x, y, x + w - 1, y + h - 1);
    lcd_gram_write();

    uint32_t total = (uint32_t)w * h;
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    for (uint32_t i = 0; i < total; i++) {
        lcd_write_data(hi);
        lcd_write_data(lo);
    }
}

/* ========================================================================
 *  Embedded 8x8 bitmap font (printable ASCII 0x20-0x7E)
 *
 *  Each character is 8 bytes, one byte per row, MSB = left pixel.
 *  Does not require SPI flash - compiled directly into firmware.
 * ======================================================================== */

static const uint8_t font8x8[] = {
    /* 0x20 space */ 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    /* 0x21 !     */ 0x18,0x18,0x18,0x18,0x18,0x00,0x18,0x00,
    /* 0x22 "     */ 0x6C,0x6C,0x24,0x00,0x00,0x00,0x00,0x00,
    /* 0x23 #     */ 0x24,0x24,0x7E,0x24,0x7E,0x24,0x24,0x00,
    /* 0x24 $     */ 0x18,0x3E,0x60,0x3C,0x06,0x7C,0x18,0x00,
    /* 0x25 %     */ 0x62,0x64,0x08,0x10,0x20,0x4C,0x8C,0x00,
    /* 0x26 &     */ 0x30,0x48,0x30,0x56,0x88,0x8C,0x72,0x00,
    /* 0x27 '     */ 0x18,0x18,0x30,0x00,0x00,0x00,0x00,0x00,
    /* 0x28 (     */ 0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00,
    /* 0x29 )     */ 0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00,
    /* 0x2A *     */ 0x00,0x24,0x18,0x7E,0x18,0x24,0x00,0x00,
    /* 0x2B +     */ 0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00,
    /* 0x2C ,     */ 0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30,
    /* 0x2D -     */ 0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,
    /* 0x2E .     */ 0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,
    /* 0x2F /     */ 0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x00,
    /* 0x30 0     */ 0x3C,0x46,0x4A,0x52,0x62,0x42,0x3C,0x00,
    /* 0x31 1     */ 0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00,
    /* 0x32 2     */ 0x3C,0x42,0x02,0x0C,0x30,0x40,0x7E,0x00,
    /* 0x33 3     */ 0x3C,0x42,0x02,0x1C,0x02,0x42,0x3C,0x00,
    /* 0x34 4     */ 0x0C,0x14,0x24,0x44,0x7E,0x04,0x04,0x00,
    /* 0x35 5     */ 0x7E,0x40,0x7C,0x02,0x02,0x42,0x3C,0x00,
    /* 0x36 6     */ 0x1C,0x20,0x40,0x7C,0x42,0x42,0x3C,0x00,
    /* 0x37 7     */ 0x7E,0x02,0x04,0x08,0x10,0x10,0x10,0x00,
    /* 0x38 8     */ 0x3C,0x42,0x42,0x3C,0x42,0x42,0x3C,0x00,
    /* 0x39 9     */ 0x3C,0x42,0x42,0x3E,0x02,0x04,0x38,0x00,
    /* 0x3A :     */ 0x00,0x18,0x18,0x00,0x18,0x18,0x00,0x00,
    /* 0x3B ;     */ 0x00,0x18,0x18,0x00,0x18,0x18,0x30,0x00,
    /* 0x3C <     */ 0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00,
    /* 0x3D =     */ 0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00,
    /* 0x3E >     */ 0x60,0x30,0x18,0x0C,0x18,0x30,0x60,0x00,
    /* 0x3F ?     */ 0x3C,0x42,0x02,0x0C,0x10,0x00,0x10,0x00,
    /* 0x40 @     */ 0x3C,0x42,0x4E,0x52,0x4E,0x40,0x3C,0x00,
    /* 0x41 A     */ 0x18,0x24,0x42,0x42,0x7E,0x42,0x42,0x00,
    /* 0x42 B     */ 0x7C,0x42,0x42,0x7C,0x42,0x42,0x7C,0x00,
    /* 0x43 C     */ 0x3C,0x42,0x40,0x40,0x40,0x42,0x3C,0x00,
    /* 0x44 D     */ 0x78,0x44,0x42,0x42,0x42,0x44,0x78,0x00,
    /* 0x45 E     */ 0x7E,0x40,0x40,0x7C,0x40,0x40,0x7E,0x00,
    /* 0x46 F     */ 0x7E,0x40,0x40,0x7C,0x40,0x40,0x40,0x00,
    /* 0x47 G     */ 0x3C,0x42,0x40,0x4E,0x42,0x42,0x3C,0x00,
    /* 0x48 H     */ 0x42,0x42,0x42,0x7E,0x42,0x42,0x42,0x00,
    /* 0x49 I     */ 0x3C,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,
    /* 0x4A J     */ 0x1E,0x04,0x04,0x04,0x04,0x44,0x38,0x00,
    /* 0x4B K     */ 0x42,0x44,0x48,0x70,0x48,0x44,0x42,0x00,
    /* 0x4C L     */ 0x40,0x40,0x40,0x40,0x40,0x40,0x7E,0x00,
    /* 0x4D M     */ 0x42,0x66,0x5A,0x5A,0x42,0x42,0x42,0x00,
    /* 0x4E N     */ 0x42,0x62,0x52,0x4A,0x46,0x42,0x42,0x00,
    /* 0x4F O     */ 0x3C,0x42,0x42,0x42,0x42,0x42,0x3C,0x00,
    /* 0x50 P     */ 0x7C,0x42,0x42,0x7C,0x40,0x40,0x40,0x00,
    /* 0x51 Q     */ 0x3C,0x42,0x42,0x42,0x4A,0x44,0x3A,0x00,
    /* 0x52 R     */ 0x7C,0x42,0x42,0x7C,0x48,0x44,0x42,0x00,
    /* 0x53 S     */ 0x3C,0x42,0x40,0x3C,0x02,0x42,0x3C,0x00,
    /* 0x54 T     */ 0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00,
    /* 0x55 U     */ 0x42,0x42,0x42,0x42,0x42,0x42,0x3C,0x00,
    /* 0x56 V     */ 0x42,0x42,0x42,0x42,0x24,0x24,0x18,0x00,
    /* 0x57 W     */ 0x42,0x42,0x42,0x5A,0x5A,0x66,0x42,0x00,
    /* 0x58 X     */ 0x42,0x24,0x18,0x18,0x18,0x24,0x42,0x00,
    /* 0x59 Y     */ 0x42,0x42,0x24,0x18,0x18,0x18,0x18,0x00,
    /* 0x5A Z     */ 0x7E,0x04,0x08,0x10,0x20,0x40,0x7E,0x00,
    /* 0x5B [     */ 0x3C,0x30,0x30,0x30,0x30,0x30,0x3C,0x00,
    /* 0x5C \     */ 0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x00,
    /* 0x5D ]     */ 0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00,
    /* 0x5E ^     */ 0x10,0x28,0x44,0x00,0x00,0x00,0x00,0x00,
    /* 0x5F _     */ 0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00,
    /* 0x60 `     */ 0x30,0x18,0x0C,0x00,0x00,0x00,0x00,0x00,
    /* 0x61 a     */ 0x00,0x00,0x3C,0x02,0x3E,0x42,0x3E,0x00,
    /* 0x62 b     */ 0x40,0x40,0x5C,0x62,0x42,0x62,0x5C,0x00,
    /* 0x63 c     */ 0x00,0x00,0x3C,0x42,0x40,0x42,0x3C,0x00,
    /* 0x64 d     */ 0x02,0x02,0x3A,0x46,0x42,0x46,0x3A,0x00,
    /* 0x65 e     */ 0x00,0x00,0x3C,0x42,0x7E,0x40,0x3C,0x00,
    /* 0x66 f     */ 0x0C,0x12,0x10,0x7C,0x10,0x10,0x10,0x00,
    /* 0x67 g     */ 0x00,0x00,0x3A,0x46,0x46,0x3A,0x02,0x3C,
    /* 0x68 h     */ 0x40,0x40,0x5C,0x62,0x42,0x42,0x42,0x00,
    /* 0x69 i     */ 0x18,0x00,0x38,0x18,0x18,0x18,0x3C,0x00,
    /* 0x6A j     */ 0x0C,0x00,0x1C,0x0C,0x0C,0x0C,0x4C,0x38,
    /* 0x6B k     */ 0x40,0x40,0x44,0x48,0x70,0x48,0x44,0x00,
    /* 0x6C l     */ 0x38,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,
    /* 0x6D m     */ 0x00,0x00,0x76,0x4A,0x4A,0x4A,0x4A,0x00,
    /* 0x6E n     */ 0x00,0x00,0x5C,0x62,0x42,0x42,0x42,0x00,
    /* 0x6F o     */ 0x00,0x00,0x3C,0x42,0x42,0x42,0x3C,0x00,
    /* 0x70 p     */ 0x00,0x00,0x5C,0x62,0x62,0x5C,0x40,0x40,
    /* 0x71 q     */ 0x00,0x00,0x3A,0x46,0x46,0x3A,0x02,0x02,
    /* 0x72 r     */ 0x00,0x00,0x5C,0x62,0x40,0x40,0x40,0x00,
    /* 0x73 s     */ 0x00,0x00,0x3E,0x40,0x3C,0x02,0x7C,0x00,
    /* 0x74 t     */ 0x10,0x10,0x7C,0x10,0x10,0x12,0x0C,0x00,
    /* 0x75 u     */ 0x00,0x00,0x42,0x42,0x42,0x46,0x3A,0x00,
    /* 0x76 v     */ 0x00,0x00,0x42,0x42,0x42,0x24,0x18,0x00,
    /* 0x77 w     */ 0x00,0x00,0x42,0x42,0x5A,0x66,0x42,0x00,
    /* 0x78 x     */ 0x00,0x00,0x42,0x24,0x18,0x24,0x42,0x00,
    /* 0x79 y     */ 0x00,0x00,0x42,0x42,0x46,0x3A,0x02,0x3C,
    /* 0x7A z     */ 0x00,0x00,0x7E,0x04,0x18,0x20,0x7E,0x00,
    /* 0x7B {     */ 0x0E,0x18,0x18,0x70,0x18,0x18,0x0E,0x00,
    /* 0x7C |     */ 0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00,
    /* 0x7D }     */ 0x70,0x18,0x18,0x0E,0x18,0x18,0x70,0x00,
    /* 0x7E ~     */ 0x00,0x00,0x32,0x4C,0x00,0x00,0x00,0x00,
};

/* Draw a single 8x8 character at pixel position (x, y) */
static void lcd_draw_char(uint16_t x, uint16_t y, char ch,
                          uint16_t fg, uint16_t bg)
{
    if (ch < 0x20 || ch > 0x7E) ch = ' ';
    const uint8_t *glyph = &font8x8[(ch - 0x20) * 8];

    lcd_set_window(x, y, x + 7, y + 7);
    lcd_gram_write();

    for (int row = 0; row < 8; row++) {
        uint8_t bits = glyph[row];
        for (int col = 0; col < 8; col++) {
            uint16_t c = (bits & 0x80) ? fg : bg;
            lcd_write_data(c >> 8);
            lcd_write_data(c & 0xFF);
            bits <<= 1;
        }
    }
}

/* Draw a string at 1x scale (8px tall) */
void lcd_draw_string(uint16_t x, uint16_t y, const char *str,
                     uint16_t fg, uint16_t bg)
{
    while (*str) {
        lcd_draw_char(x, y, *str, fg, bg);
        x += 8;
        str++;
    }
}

/* Draw a single character at 2x scale (16x16 pixels) */
static void lcd_draw_char_2x(uint16_t x, uint16_t y, char ch,
                              uint16_t fg, uint16_t bg)
{
    if (ch < 0x20 || ch > 0x7E) ch = ' ';
    const uint8_t *glyph = &font8x8[(ch - 0x20) * 8];

    lcd_set_window(x, y, x + 15, y + 15);
    lcd_gram_write();

    for (int row = 0; row < 8; row++) {
        uint8_t bits = glyph[row];
        /* Each glyph row emits two screen rows */
        for (int dup = 0; dup < 2; dup++) {
            uint8_t b = bits;
            for (int col = 0; col < 8; col++) {
                uint16_t c = (b & 0x80) ? fg : bg;
                uint8_t hi = c >> 8, lo = c & 0xFF;
                lcd_write_data(hi); lcd_write_data(lo);
                lcd_write_data(hi); lcd_write_data(lo);
                b <<= 1;
            }
        }
    }
}

/* Draw a string at 2x scale (16px tall) */
void lcd_draw_string_2x(uint16_t x, uint16_t y, const char *str,
                         uint16_t fg, uint16_t bg)
{
    while (*str) {
        lcd_draw_char_2x(x, y, *str, fg, bg);
        x += 16;
        str++;
    }
}

/* ========================================================================
 *  lcd_init - Full ST7789V initialization sequence.
 *
 *  V0.27 LCD_Init @ 0x08026954 (corrected from 0x08029954 -0x3000).
 *  write_cmd calls 0x080267B8, write_data calls 0x08026818.
 *  Commands verified byte-by-byte as ST7789V-specific.
 * ======================================================================== */

void lcd_init(void)
{
    /* PC14 was originally thought to be an "LCD enable gate" but is actually
     * the GREEN LED (HW confirmed). Setting it HIGH here is harmless. */
    gpio_config_pin(GPIOC, GPIO_PIN_14, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);
    gpio_set_pin(GPIOC, GPIO_PIN_14);

    /* Configure control pins as push-pull outputs */
    gpio_config_pin(LCD_WR_PORT,  LCD_WR_PIN,  GPIO_MODE_OUT_50MHZ, GPIO_CNF_PP);
    gpio_config_pin(LCD_CS_PORT,  LCD_CS_PIN,  GPIO_MODE_OUT_50MHZ, GPIO_CNF_PP);
    gpio_config_pin(LCD_DC_PORT,  LCD_DC_PIN,  GPIO_MODE_OUT_50MHZ, GPIO_CNF_PP);
    gpio_config_pin(LCD_RST_PORT, LCD_RST_PIN, GPIO_MODE_OUT_2MHZ,  GPIO_CNF_PP);
    gpio_config_pin(LCD_BL_PORT,  LCD_BL_PIN,  GPIO_MODE_OUT_2MHZ,  GPIO_CNF_PP);
    /* PB3 secondary backlight driver - OEM toggles both @ 0x08017C40 */
    gpio_enable_clock(LCD_BL_SEC_PORT);
    gpio_config_pin(LCD_BL_SEC_PORT, LCD_BL_SEC_PIN, GPIO_MODE_OUT_2MHZ, GPIO_CNF_PP);

    /* Configure data pins PD8-PD15 as push-pull outputs */
    for (uint8_t i = 8; i <= 15; i++) {
        gpio_config_pin(LCD_DATA_PORT, (1U << i), GPIO_MODE_OUT_50MHZ, GPIO_CNF_PP);
    }

    /* Idle state: CS high, WR high */
    gpio_set_pin(LCD_CS_PORT, LCD_CS_PIN);
    gpio_set_pin(LCD_WR_PORT, LCD_WR_PIN);

    /* Hardware reset - V0.27 @ 0x08026954: PD2 HIGH->LOW(1ms)->HIGH(120ms) */
    lcd_panel_reset();

    /* ST7789V init sequence (V0.27 @ 0x08026982) ------------------- */

    lcd_write_command(0x11);    /* Sleep Out */
    delay_ms(120);              /* V0.27: movs r0, 0x78 */

    /* Porch control - V0.27 @ 0x0802698E */
    lcd_write_command(0xB2);
    lcd_write_data(0x05);       /* front porch */
    lcd_write_data(0x05);       /* back porch */
    lcd_write_data(0x00);       /* porch enable */
    lcd_write_data(0x33);       /* separate porch */
    lcd_write_data(0x33);

    /* Gate control - V0.27 @ 0x080269B2: VGH=13.26V, VGL=-10.43V */
    lcd_write_command(0xB7);
    lcd_write_data(0x35);

    /* LCM control - V0.27 @ 0x080269BE */
    lcd_write_command(0xC0);
    lcd_write_data(0x2C);

    /* VDV and VRH command enable - V0.27 @ 0x080269CA */
    lcd_write_command(0xC2);
    lcd_write_data(0x01);

    /* VRH set - V0.27 @ 0x080269D6: 0x0F */
    lcd_write_command(0xC3);
    lcd_write_data(0x0F);

    /* VDV set - V0.27 @ 0x080269E2 */
    lcd_write_command(0xC4);
    lcd_write_data(0x20);

    /* Frame rate control - V0.27 @ 0x080269EE */
    lcd_write_command(0xC6);
    lcd_write_data(0x11);

    /* Power control 1 - V0.27 @ 0x080269FA */
    lcd_write_command(0xD0);
    lcd_write_data(0xA4);
    lcd_write_data(0xA1);

    /* Equalizing time control - V0.27 @ 0x08026A0C */
    lcd_write_command(0xE8);
    lcd_write_data(0x03);

    /* Gate timing control - V0.27 @ 0x08026A18 */
    lcd_write_command(0xE9);
    lcd_write_data(0x09);
    lcd_write_data(0x09);
    lcd_write_data(0x08);

    /* VCOM setting - V0.27 @ 0x08026A30 */
    lcd_write_command(0xBB);
    lcd_write_data(0x3F);

    /* Positive voltage gamma - V0.27 @ 0x08026A3C (14 params) */
    lcd_write_command(0xE0);
    lcd_write_data(0xD0);
    lcd_write_data(0x05);
    lcd_write_data(0x09);
    lcd_write_data(0x09);
    lcd_write_data(0x08);
    lcd_write_data(0x14);
    lcd_write_data(0x28);
    lcd_write_data(0x33);
    lcd_write_data(0x3F);
    lcd_write_data(0x07);
    lcd_write_data(0x13);
    lcd_write_data(0x14);
    lcd_write_data(0x28);
    lcd_write_data(0x30);

    /* Negative voltage gamma - V0.27 @ 0x08026A96 (14 params) */
    lcd_write_command(0xE1);
    lcd_write_data(0xD0);
    lcd_write_data(0x05);
    lcd_write_data(0x09);
    lcd_write_data(0x09);
    lcd_write_data(0x08);
    lcd_write_data(0x03);
    lcd_write_data(0x24);
    lcd_write_data(0x32);
    lcd_write_data(0x32);
    lcd_write_data(0x3B);
    lcd_write_data(0x14);
    lcd_write_data(0x13);
    lcd_write_data(0x28);
    lcd_write_data(0x2F);

    /* Memory Access Control - V0.27 @ 0x08026AF0: 180 deg rotation */
    lcd_write_command(0x36);
    lcd_write_data(0xC0);

    /* Pixel format - V0.27 @ 0x08026AFC: 16-bit MCU (RGB565) */
    lcd_write_command(0x3A);
    lcd_write_data(0x05);

    /* Display Inversion On - V0.27 @ 0x08026B08 (required for IPS) */
    lcd_write_command(0x21);

    /* Display ON - V0.27 @ 0x08026B0E */
    lcd_write_command(0x29);
    delay_ms(20);

    /* Turn on backlight */
    lcd_backlight_on();
}
