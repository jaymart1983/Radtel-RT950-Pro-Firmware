/*
 * zone_filter.h - Zones as a filter rather than an exclusive mode
 *
 * The OEM treats zones as a MODE: the radio is either in Channel mode, where
 * the knob walks all channels and zone names are invisible, or in Zone mode,
 * where the zone name shows but the knob is confined to one zone. You have to
 * leave a zone to reach anything outside it.
 *
 * Here a zone is a filter instead. The knob always walks every channel that is
 * currently visible, in channel-number order, crossing zone boundaries freely.
 * The zone name of whatever channel you land on shows on the main display. The
 * Zone menu becomes a checklist: unticking a zone hides its channels from the
 * knob and the arrow keys, without changing which channel is tuned and without
 * changing any channel record.
 *
 * Zone membership is by 64-channel bank:
 *
 *     zone = (channel_number - 1) / 64        (channel_number is 1-based)
 *
 * This is the OEM's own arrangement -- BANK_CH_NUM is 64 in Radtel's own
 * source, and the channel plan on the radio lines up with it exactly: GMRS
 * starts at 1, Idaho repeaters at 65, Utah at 129, Nevada at 193, NOAA at 257.
 *
 * PERSISTENCE. The OEM's zone-enable state lives at SRAM 0x2000A394 and is
 * rebuilt at boot, so it cannot be borrowed. The mask is stored instead in
 * spare bytes of settings_t, which the wear-levelling layer already persists.
 *
 * A deliberate detail: a SET bit means visible. Erased flash reads 0xFF, so a
 * radio that has never written this field comes up with every zone visible,
 * which is the correct default and needs no migration step.
 */

#ifndef APP_ZONE_FILTER_H
#define APP_ZONE_FILTER_H

#include <stdint.h>
#include "drivers/flash_layout.h"

#define ZONE_FILTER_COUNT     FLASH_ZONE_MAX    /* 10 */
#define ZONE_CHANNELS_PER_ZONE 64

/* Load the mask from settings. Call once, after settings_init(). */
void zone_filter_init(void);

/* Which zone a 1-based channel number belongs to. */
uint8_t zone_of_channel(uint16_t ch_num);

/* Non-zero if the zone is ticked (its channels are visible). */
uint8_t zone_is_enabled(uint8_t zone);

/* Tick or untick a zone and persist.
 *
 * Refuses to untick the last enabled zone: an all-unticked mask would leave the
 * knob with nothing to land on and no obvious way back. Returns non-zero if the
 * change was applied. */
uint8_t zone_set_enabled(uint8_t zone, uint8_t enabled);

/* Convenience for the menu. */
uint8_t zone_toggle(uint8_t zone);

/* How many zones are currently ticked. */
uint8_t zone_enabled_count(void);

/* Non-zero if this channel is visible: it is populated AND its zone is ticked.
 * This is the single predicate the picker and the arrow keys both use. */
uint8_t zone_channel_visible(uint16_t ch_num);

/* Walk to the next visible channel from ch_num in direction (+1 / -1).
 * Wraps around. Returns 0xFFFF if no channel anywhere is visible. */
uint16_t zone_find_next_visible(uint16_t ch_num, int8_t direction);

/* Total visible channels. Used to size the picker's scrollbar. */
uint16_t zone_visible_count(void);

/* Name of the zone a channel belongs to, into buf (>= FLASH_ZONE_NAME_SIZE+1).
 * Empty string if the zone has no name programmed. */
void zone_name_for_channel(uint16_t ch_num, char *buf);

#endif /* APP_ZONE_FILTER_H */
