/*
 * zone_filter.c - Zones as a filter rather than an exclusive mode
 *
 * See zone_filter.h for the design and for why the mask is stored the way it is.
 */

#include "app/zone_filter.h"
#include "app/zone_browser.h"
#include "app/channel.h"
#include "app/settings.h"

#include <stddef.h>

/* Cached mask. Bit n set = zone n visible. */
static uint16_t enable_mask = 0xFFFF;

/* Where the mask lives inside settings_t's spare bytes. Two u8s rather than a
 * u16 so the existing settings_set_u8() API covers it and packing/endianness
 * never enter into it. */
#define ZONE_MASK_LO_OFFSET (offsetof(settings_t, _reserved) + 0)
#define ZONE_MASK_HI_OFFSET (offsetof(settings_t, _reserved) + 1)

void zone_filter_init(void)
{
    const settings_t *s = settings_get();
    uint16_t m = (uint16_t)s->_reserved[0] | ((uint16_t)s->_reserved[1] << 8);

    /* Erased flash reads 0xFFFF, which already means "every zone visible", so
     * there is nothing to migrate on a radio that has never written this. */
    enable_mask = m;

    /* Defensive: if every in-range bit is somehow clear, the knob would have
     * nothing to land on. Fall back to all-visible rather than trapping the
     * user in an empty list with no way out through the UI. */
    if ((enable_mask & ((1u << ZONE_FILTER_COUNT) - 1u)) == 0)
        enable_mask = 0xFFFF;
}

static void zone_filter_save(void)
{
    settings_set_u8(ZONE_MASK_LO_OFFSET, (uint8_t)(enable_mask & 0xFF));
    settings_set_u8(ZONE_MASK_HI_OFFSET, (uint8_t)(enable_mask >> 8));
    settings_save();
}

uint8_t zone_of_channel(uint16_t ch_num)
{
    if (ch_num == 0) return 0;                       /* 1-based; guard 0 */
    uint16_t z = (uint16_t)((ch_num - 1) / ZONE_CHANNELS_PER_ZONE);
    return (z >= ZONE_FILTER_COUNT) ? (ZONE_FILTER_COUNT - 1) : (uint8_t)z;
}

uint8_t zone_is_enabled(uint8_t zone)
{
    if (zone >= ZONE_FILTER_COUNT) return 0;
    return (enable_mask >> zone) & 1u;
}

uint8_t zone_enabled_count(void)
{
    uint8_t n = 0;
    for (uint8_t z = 0; z < ZONE_FILTER_COUNT; z++)
        if (zone_is_enabled(z)) n++;
    return n;
}

uint8_t zone_set_enabled(uint8_t zone, uint8_t enabled)
{
    if (zone >= ZONE_FILTER_COUNT) return 0;

    /* Never allow the last zone to be unticked. */
    if (!enabled && zone_is_enabled(zone) && zone_enabled_count() <= 1)
        return 0;

    uint16_t before = enable_mask;
    if (enabled) enable_mask |= (uint16_t)(1u << zone);
    else         enable_mask &= (uint16_t)~(1u << zone);

    if (enable_mask != before) zone_filter_save();
    return 1;
}

uint8_t zone_toggle(uint8_t zone)
{
    return zone_set_enabled(zone, (uint8_t)!zone_is_enabled(zone));
}

uint8_t zone_channel_visible(uint16_t ch_num)
{
    if (ch_num < 1 || ch_num > CHANNEL_COUNT) return 0;
    if (!zone_is_enabled(zone_of_channel(ch_num))) return 0;
    return channel_is_valid((uint16_t)(ch_num - 1));
}

uint16_t zone_find_next_visible(uint16_t ch_num, int8_t direction)
{
    if (direction == 0) direction = 1;

    /* Walk at most one full lap, so an all-hidden configuration terminates
     * instead of spinning forever. */
    uint16_t cur = ch_num;
    for (uint16_t step = 0; step < CHANNEL_COUNT; step++) {
        if (direction > 0)
            cur = (uint16_t)((cur >= CHANNEL_COUNT) ? 1 : cur + 1);
        else
            cur = (uint16_t)((cur <= 1) ? CHANNEL_COUNT : cur - 1);

        if (zone_channel_visible(cur)) return cur;
    }
    return 0xFFFF;
}

uint16_t zone_visible_count(void)
{
    uint16_t n = 0;
    for (uint16_t ch = 1; ch <= CHANNEL_COUNT; ch++)
        if (zone_channel_visible(ch)) n++;
    return n;
}

void zone_name_for_channel(uint16_t ch_num, char *buf)
{
    zone_read_name(zone_of_channel(ch_num), buf);
}
