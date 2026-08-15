/*
 * channel_picker.c - Knob-spin channel picker overlay
 *
 * See channel_picker.h for the behaviour and why preview-before-commit matters.
 */

#include "app/channel_picker.h"
#include "app/zone_filter.h"
#include "app/channel.h"
#include "app/keypad.h"
#include "app/display.h"
#include "app/font.h"
#include "drivers/lcd.h"
#include "drivers/timer.h"
#include "drivers/flash_layout.h"

/* Geometry. The panel slides in from the right and deliberately does not cover
 * the full width: the frequency stays visible on the left while browsing, so
 * you can see what you are still tuned to. */
#define PANEL_W      150
#define PANEL_X      (LCD_WIDTH - PANEL_W)
#define ROW_H        18
#define HEADER_H     20
#define PANEL_Y      0
#define PANEL_H      (HEADER_H + PICKER_VISIBLE_ROWS * ROW_H + 14)

/* State ---------------------------------------------------------------- */

static uint8_t       active;
static picker_mode_t mode;
static uint16_t      cursor_ch;      /* highlighted channel (1-based) */
static uint16_t      origin_ch;      /* what we were tuned to on open */
static uint32_t      last_input_ms;
static uint8_t       timed_out;      /* so timeout reports exactly once */

static void touch(void)
{
    last_input_ms = timer_get_fast_tick();
}

void channel_picker_open(picker_mode_t m, uint16_t current_ch)
{
    if (!active) {
        mode      = m;
        origin_ch = current_ch;
        /* Start on the current channel if it is visible. If it is not -- its
         * zone may have just been unticked -- fall back to the next visible
         * one rather than highlighting something the list does not contain. */
        cursor_ch = zone_channel_visible(current_ch)
                  ? current_ch
                  : zone_find_next_visible(current_ch, +1);
        if (cursor_ch == 0xFFFF) cursor_ch = current_ch;
        active    = 1;
        timed_out = 0;
    }
    touch();
}

void channel_picker_close(void)
{
    active = 0;
}

uint8_t  channel_picker_is_active(void)   { return active; }
uint16_t channel_picker_get_selection(void) { return cursor_ch; }

/* Input ---------------------------------------------------------------- */

picker_result_t channel_picker_handle_encoder(int8_t direction, uint16_t current_ch)
{
    if (!active) {
        /* First detent opens the overlay showing where you already are. It
         * does NOT move the highlight -- otherwise the act of looking at the
         * list would already have moved you off your channel. */
        channel_picker_open(PICKER_MODE_CHANNEL, current_ch);
        return PICKER_CONSUMED;
    }

    touch();

    if (mode == PICKER_MODE_CHANNEL) {
        uint16_t next = zone_find_next_visible(cursor_ch, direction);
        /* 0xFFFF means nothing is visible at all; leave the cursor alone
         * rather than parking it on a channel the list cannot show. */
        if (next != 0xFFFF) cursor_ch = next;
    }

    return PICKER_CONSUMED;
}

picker_result_t channel_picker_handle_key(uint8_t key)
{
    if (!active) return PICKER_IDLE;

    switch (key) {
    case KEY_C_MENU:
        active = 0;
        return PICKER_COMMIT;

    case KEY_D_BAND:
    case KEY_HASH:
        active    = 0;
        cursor_ch = origin_ch;
        return PICKER_CANCEL;

    default:
        /* Any other key dismisses the overlay and is then handled normally by
         * the caller, so the picker never swallows a keypress the user meant
         * for something else. */
        active    = 0;
        cursor_ch = origin_ch;
        return PICKER_CANCEL;
    }
}

picker_result_t channel_picker_tick(void)
{
    if (!active) return PICKER_IDLE;

    /* Unsigned subtraction, so this stays correct across the 32-bit tick
     * wrap at ~49.7 days of uptime. */
    uint32_t elapsed = timer_get_fast_tick() - last_input_ms;
    if (elapsed >= PICKER_TIMEOUT_MS) {
        active    = 0;
        cursor_ch = origin_ch;
        if (!timed_out) {
            timed_out = 1;
            return PICKER_CANCEL;
        }
    }
    return PICKER_IDLE;
}

/* Drawing -------------------------------------------------------------- */

/* Walk back n visible channels from `from`, stopping at the start of the list
 * so the top of the list does not scroll past channel 1. */
static uint16_t back_n_visible(uint16_t from, uint8_t n)
{
    uint16_t cur = from;
    for (uint8_t i = 0; i < n; i++) {
        uint16_t prev = zone_find_next_visible(cur, -1);
        if (prev == 0xFFFF || prev >= cur) break;   /* none left, or wrapped */
        cur = prev;
    }
    return cur;
}

void channel_picker_draw(void)
{
    if (!active) return;

    lcd_fill_rect(PANEL_X, PANEL_Y, PANEL_W, PANEL_H, COLOR_BLACK);
    /* A one-pixel rule down the left edge reads as "this slid over the top of
     * the normal screen" rather than as a mode change. */
    lcd_fill_rect(PANEL_X, PANEL_Y, 1, PANEL_H, COLOR_DARK_GRAY);

    /* Header: the zone of the highlighted channel, which is the whole reason
     * zone names are worth keeping visible while browsing. */
    char zname[FLASH_ZONE_NAME_SIZE + 1];
    zone_name_for_channel(cursor_ch, zname);
    font_draw_string(FONT_SMALL, PANEL_X + 5, PANEL_Y + 4,
                     zname[0] ? zname : "All Channels",
                     COLOR_YELLOW, COLOR_BLACK);
    lcd_fill_rect(PANEL_X + 1, PANEL_Y + HEADER_H - 2, PANEL_W - 1, 1,
                  COLOR_DARK_GRAY);

    /* Keep the highlight in the middle of the panel once there is enough
     * above it, so the eye has a fixed target while the list moves. */
    uint16_t top = back_n_visible(cursor_ch, PICKER_VISIBLE_ROWS / 2);

    channel_t ch;
    uint16_t  y  = PANEL_Y + HEADER_H;
    uint16_t  it = top;

    for (uint8_t row = 0; row < PICKER_VISIBLE_ROWS; row++) {
        if (it == 0xFFFF) break;

        uint8_t  sel = (it == cursor_ch);
        uint16_t fg  = sel ? COLOR_BLACK : COLOR_WHITE;
        uint16_t bg  = sel ? COLOR_YELLOW : COLOR_BLACK;

        if (sel) lcd_fill_rect(PANEL_X + 1, y, PANEL_W - 1, ROW_H, bg);

        /* Channel number, 1-based to match what the radio's own display
         * shows -- mixing bases here makes every comparison ambiguous. */
        char num[5];
        num[0] = (char)('0' + (it / 100) % 10);
        num[1] = (char)('0' + (it / 10) % 10);
        num[2] = (char)('0' + (it % 10));
        num[3] = '\0';
        if (num[0] == '0') {
            num[0] = ' ';
            if (num[1] == '0') num[1] = ' ';
        }
        font_draw_string(FONT_SMALL, PANEL_X + 4, y + 3, num, fg, bg);

        /* Name, falling back to the number when a channel has none. */
        if (channel_load((uint16_t)(it - 1), &ch) == 0 && ch.name[0])
            font_draw_string(FONT_SMALL, PANEL_X + 32, y + 3, ch.name, fg, bg);
        else
            font_draw_string(FONT_SMALL, PANEL_X + 32, y + 3, "-", fg, bg);

        y += ROW_H;

        uint16_t next = zone_find_next_visible(it, +1);
        if (next == 0xFFFF || next <= it) break;   /* end of list, or wrapped */
        it = next;
    }

    /* Footer. Naming the keys matters because the knob does not push -- there
     * is no "click to confirm" to discover by trying. */
    uint16_t fy = PANEL_Y + PANEL_H - 12;
    lcd_fill_rect(PANEL_X + 1, fy - 3, PANEL_W - 1, 1, COLOR_DARK_GRAY);
    font_draw_string(FONT_SMALL, PANEL_X + 4, fy, "MENU=OK  EXIT",
                     COLOR_GRAY, COLOR_BLACK);
}
