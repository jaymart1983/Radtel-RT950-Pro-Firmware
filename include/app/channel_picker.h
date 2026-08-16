/*
 * channel_picker.h - Knob-spin channel picker overlay
 *
 * Spinning the top knob slides a list out from the right showing the programmed
 * channels. Further spinning moves a highlight through that list. The radio
 * stays on the channel it was already on until the selection is confirmed --
 * spinning past twenty channels does not retune the radio twenty times, which
 * is the whole point: on the OEM firmware every detent is an immediate channel
 * change, so finding a channel means transmitting-ready on every one you pass.
 *
 *   spin        open the overlay, then move the highlight
 *   MENU        commit -- the highlighted channel becomes the active one
 *   EXIT/BAND   cancel -- nothing changes
 *   30 s idle   cancel, same as EXIT
 *
 * Scope. In channel mode the list holds only channels that are programmed AND
 * whose zone is ticked (see zone_filter.h) -- not all 990 slots, which would be
 * unusable. In frequency mode there is no list to scope: the knob steps the
 * frequency by the band's channel spacing, and the same preview-then-commit
 * rule applies.
 *
 * The timeout is measured from the last interaction of any kind, not from when
 * the overlay opened, so a slow scroll is never cut off mid-way.
 */

#ifndef APP_CHANNEL_PICKER_H
#define APP_CHANNEL_PICKER_H

#include <stdint.h>

/* Idle time before the overlay gives up and cancels. */
#define PICKER_TIMEOUT_MS   30000u

/* Rows drawn at once. The list scrolls under a fixed highlight once the
 * cursor reaches the middle, so the eye has a stable target. */
#define PICKER_VISIBLE_ROWS 9

typedef enum {
    PICKER_MODE_CHANNEL = 0,   /* list of programmed, visible channels */
    PICKER_MODE_FREQ    = 1,   /* frequency stepping by band spacing */
} picker_mode_t;

/* Result of feeding the picker an event. */
typedef enum {
    PICKER_IDLE = 0,       /* not open; caller should handle the event itself */
    PICKER_CONSUMED,       /* open and handled it; do not act on it elsewhere */
    PICKER_COMMIT,         /* closed, and the caller should tune the selection */
    PICKER_CANCEL,         /* closed with no change */
} picker_result_t;

/* Open on the channel the radio is currently tuned to. Safe to call when
 * already open -- it just refreshes the timeout. */
void channel_picker_open(picker_mode_t mode, uint16_t current_ch);

void channel_picker_close(void);
uint8_t channel_picker_is_active(void);

/* The highlighted channel. Only meaningful after PICKER_COMMIT. */
uint16_t channel_picker_get_selection(void);

/* Feed a knob detent. If the overlay is closed this OPENS it and does NOT move
 * the highlight, so the first spin reveals where you are rather than moving you
 * off it. */
picker_result_t channel_picker_handle_encoder(int8_t direction, uint16_t current_ch);

/* Feed a key press. Returns PICKER_IDLE when closed, so callers can pass every
 * key through unconditionally. */
picker_result_t channel_picker_handle_key(uint8_t key);

/* Call from the main loop. Returns PICKER_CANCEL exactly once on timeout. */
picker_result_t channel_picker_tick(void);

void channel_picker_draw(void);

#endif /* APP_CHANNEL_PICKER_H */
