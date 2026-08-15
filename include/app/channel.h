/*
 * channel.h - Memory channel management for the RT-950 Pro
 *
 * Provides load/save/delete operations for the 990 channel slots
 * stored in SPI flash, plus VFO <-> channel transfer functions.
 *
 * V0.27 OEM channel functions:
 *   Channel write @ 0x0800EFB0  (read-modify-write 4KB sector)
 *   Channel read  @ 0x0800FA58  (addr = ch_num * 32, reads 32 bytes)
 *   Bitmap scan   @ 0x0800EF68  (WL slot allocation)
 *
 * Channel record format (32 bytes packed BCD):
 *   [0-3]   RX freq (packed BCD, Hz/10)
 *   [4-7]   TX freq (packed BCD, Hz/10)
 *   [8-9]   RX CTCSS/DCS index (hi/lo into 261-entry tone table)
 *   [10-11] TX CTCSS/DCS index (hi/lo)
 *   [12]    Signal code (DTMF group 0-14)
 *   [13]    PTT-ID (0=OFF, 1=BOT, 2=EOT, 3=BOTH)
 *   [14]    Power[3:0] / Scramble[7:4]
 *   [15]    Flags: RxMod[0], TxEnable[1], ScanAdd[2], BusyLock[3],
 *           Encryption[5:4], NarrowWide[6]
 *   [16-19] Reserved
 *   [20-31] Channel name (12 bytes ASCII, space-padded)
 *
 * Flash layout: 128 channels/sector (lsrs r1, r0, 7 @ 0x0800EFBC),
 *   sector_base = sector * 4096, offset = ch_num & 127.
 *
 * System settings: 80 bytes across 5 blocks (SYSCFG/VFOCFG/EXTCFG/
 *   VFOSEL/CHCFG), 67 fields total. See flash_wearleveling.h.
 *
 * See also: flash_layout.h for complete record encoding.
 */

#ifndef APP_CHANNEL_H
#define APP_CHANNEL_H

#include <stdint.h>

/* The on-flash format is defined by flash_layout.h; defer to it rather than
 * restating it. Both headers previously defined CHANNEL_RECORD_SIZE with the
 * same value but a different spelling (0x20 vs 32), and the channel count as
 * CHANNEL_COUNT_MAX vs CHANNEL_COUNT. That was harmless only for as long as no
 * translation unit included both -- the first one to do so failed to build on
 * the redefinition. One definition of each now. */
#include "drivers/flash_layout.h"

#define CHANNEL_COUNT       CHANNEL_COUNT_MAX
#define CHANNEL_NAME_LEN    12

/* Channel record (parsed from 32-byte flash record) */
typedef struct {
    uint32_t rx_freq_hz;        /* RX frequency in Hz */
    uint32_t tx_freq_hz;        /* TX frequency in Hz (may differ for offset) */
    uint8_t  tx_power;          /* 0=low, 1=mid, 2=high */
    uint8_t  bandwidth;         /* 0=narrow, 1=wide */
    uint8_t  modulation;        /* 0=FM, 1=AM */
    uint8_t  ctcss_tx_idx;      /* 0xFF=off, 0-49=CTCSS tone index */
    uint8_t  ctcss_rx_idx;      /* 0xFF=off */
    uint8_t  dcs_tx_idx;        /* 0xFF=off, 0-103=DCS code index */
    uint8_t  dcs_rx_idx;        /* 0xFF=off */
    uint8_t  scrambler;         /* 0=off, 1=on */
    uint8_t  scan_add;          /* 1=included in scan list */
    uint8_t  squelch;           /* squelch level 0-9 */
    char     name[CHANNEL_NAME_LEN + 1]; /* Null-terminated name */
} channel_t;

/* Initialize channel subsystem */
void channel_init(void);

/* Load a channel from flash. Returns 0=success, -1=empty/invalid */
int channel_load(uint16_t ch_num, channel_t *ch);

/* Save a channel to flash */
int channel_save(uint16_t ch_num, const channel_t *ch);

/* Delete a channel (write all 0xFF) */
int channel_delete(uint16_t ch_num);

/* Check if a channel slot is populated */
uint8_t channel_is_valid(uint16_t ch_num);

/* Apply channel settings to a VFO */
void channel_to_vfo(const channel_t *ch, uint8_t vfo_idx);

/* Copy current VFO settings into a channel struct */
void channel_from_vfo(channel_t *ch, uint8_t vfo_idx);

/* Get total number of populated channels (scans all - slow) */
uint16_t channel_count_used(void);

/* Find next populated channel from start_ch in direction (+1/-1).
 * Returns channel number, or 0xFFFF if none found. */
uint16_t channel_find_next(uint16_t start_ch, int8_t direction);

/* Find next channel in scan list from start_ch. */
uint16_t channel_find_next_scannable(uint16_t start_ch, int8_t direction);

#endif /* APP_CHANNEL_H */
