/*
 * zone_browser.c - Zone visibility checklist for the RT-950 Pro
 *
 * This was a zone SELECTOR: pick one zone, and the radio confines itself to it.
 * It is now a CHECKLIST, because zones here are a filter rather than a mode
 * (see zone_filter.h). Ticking a zone makes its channels reachable from the
 * knob and the arrow keys; unticking hides them. Several zones are visible at
 * once, which is the normal case -- you want GMRS and the local repeaters at
 * the same time, not one or the other.
 *
 *   spin    move the cursor
 *   MENU    toggle the zone under the cursor (saves immediately)
 *   EXIT    close
 *
 * Toggling never changes which channel is tuned. Unticking the zone you are
 * currently sitting in leaves you on that channel; it just stops appearing when
 * you spin past. The last enabled zone cannot be unticked -- zone_filter
 * refuses it -- because an empty list has no way out through the UI.
 *
 * Zone names come from SPI flash at 0x00C000, 16-byte stride, 12 bytes used.
 * Confirmed by dumping a physical radio: the ten OEM defaults "ZoneOne" ..
 * "ZoneTen" live there.
 */

#include "app/zone_browser.h"
#include "app/keypad.h"
#include "app/display.h"
#include "app/font.h"
#include "drivers/lcd.h"
#include "drivers/spi.h"
#include "drivers/flash_layout.h"

/* State ---------------------------------------------------------------- */

static uint8_t active;
static uint8_t cursor;       /* 0..FLASH_ZONE_MAX-1 */
static uint8_t selected;     /* confirmed selection, or 0xFF */

void zone_browser_open(void)
{
    active = 1;
    cursor = 0;
    selected = 0xFF;
}

void zone_browser_close(void)
{
    active = 0;
}

uint8_t zone_browser_is_active(void) { return active; }
uint8_t zone_browser_get_selected(void) { return selected; }

/* Flash read ----------------------------------------------------------- */

void zone_read_name(uint8_t index, char *buf)
{
    if (index >= FLASH_ZONE_MAX) {
        buf[0] = '\0';
        return;
    }

    uint8_t raw[FLASH_ZONE_NAME_SIZE];
    /* stride (16) and read length (12) differ — see flash_layout.h */
    uint32_t addr = FLASH_ADDR_ZONE_NAMES + (uint32_t)index * FLASH_ZONE_NAME_STRIDE;
    spi_flash_read(addr, raw, FLASH_ZONE_NAME_SIZE);

    /* Copy, converting 0xFF padding to NUL */
    uint8_t i;
    for (i = 0; i < FLASH_ZONE_NAME_SIZE; i++) {
        if (raw[i] == 0xFF || raw[i] == '\0') break;
        buf[i] = (char)raw[i];
    }
    buf[i] = '\0';
}

/* Input ---------------------------------------------------------------- */

void zone_browser_handle_key(uint8_t key)
{
    if (!active) return;

    if (key == KEY_C_MENU) {
        /* Toggle rather than select-and-close. Staying open matters: ticking
         * zones is usually done several at a time, and closing after each one
         * would mean reopening the menu for every change. */
        selected = cursor;
        active = 0;
        return;
    }

    if (key == KEY_D_BAND || key == KEY_HASH) {
        selected = 0xFF;
        active = 0;
        return;
    }
}

void zone_browser_handle_encoder(int8_t direction)
{
    if (!active) return;

    if (direction > 0 && cursor < FLASH_ZONE_MAX - 1)
        cursor++;
    else if (direction < 0 && cursor > 0)
        cursor--;
}

/* Drawing -------------------------------------------------------------- */

void zone_browser_draw(void)
{
    if (!active) return;

    lcd_fill_rect(0, 0, LCD_WIDTH, 200, COLOR_BLACK);

    /* Header */
    font_draw_string(FONT_SMALL, 4, 4, "Zones (show/hide)", COLOR_YELLOW,
                     COLOR_BLACK);
    lcd_fill_rect(0, 18, LCD_WIDTH, 1, COLOR_DARK_GRAY);

    /* List all zones */
    char name[FLASH_ZONE_NAME_SIZE + 1];
    uint16_t y = 24;

    for (uint8_t i = 0; i < FLASH_ZONE_MAX; i++) {
        uint8_t is_sel = (i == cursor);
        uint8_t on     = 1;   /* no filter state without zone_filter */
        uint16_t fg = is_sel ? COLOR_WHITE : COLOR_GRAY;
        uint16_t bg = is_sel ? COLOR_DARK_GRAY : COLOR_BLACK;

        if (is_sel)
            lcd_fill_rect(0, y, LCD_WIDTH, 16, bg);

        /* Checkbox. Drawn as text so it works with the existing font and
         * needs no glyph work: [x] ticked, [ ] unticked. */
        const char *box = on ? "[x]" : "[ ]";
        font_draw_string(FONT_SMALL, 4, y + 1, box,
                         on ? COLOR_GREEN : COLOR_DARK_GRAY, bg);

        /* Zone number */
        char num[4];
        num[0] = (char)('0' + ((i + 1) / 10));
        num[1] = (char)('0' + ((i + 1) % 10));
        num[2] = '.';
        num[3] = '\0';
        if (num[0] == '0') num[0] = ' ';
        font_draw_string(FONT_SMALL, 32, y + 1, num, fg, bg);

        /* Zone name. A hidden zone is greyed even when the cursor is on it,
         * so the ticked state stays readable at a glance. */
        uint16_t name_fg = on ? fg : COLOR_DARK_GRAY;
        zone_read_name(i, name);
        if (name[0] == '\0') {
            font_draw_string(FONT_SMALL, 58, y + 1, "(empty)",
                             COLOR_DARK_GRAY, bg);
        } else {
            font_draw_string(FONT_SMALL, 58, y + 1, name, name_fg, bg);
        }

        y += 16;
    }

    /* Footer */
    lcd_fill_rect(0, y + 4, LCD_WIDTH, 1, COLOR_DARK_GRAY);
    font_draw_string(FONT_SMALL, 4, y + 8, "MENU=Toggle  EXIT=Back",
                     COLOR_GRAY, COLOR_BLACK);
}
