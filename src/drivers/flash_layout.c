/*
 * flash_layout.c - SPI flash memory management for RT-950 Pro
 *
 * Provides channel read/write, calibration, and settings access
 * on top of the low-level SPI flash driver (spi.h).
 *
 * Flash chip: JEDEC 5E 40 16 on the measured radio -- 4 MB, not the
 * Winbond W25Q16 (2 MB) previously assumed. See flash_layout.h.
 * Channel memory: 990 channels x 32 bytes at 0x000000
 *   V0.27 channel write @ 0x0800EFB0 (read-modify-write 4KB sector)
 *   V0.27 channel read  @ 0x0800FA58 (addr = ch * 32 via lsl 5)
 *   128 channels/sector (lsrs r1, r0, 7), name at offset 0x14 (20)
 * VFO config: 3 VFOs at 0x008000 (A/B/C, 32 bytes each)
 * System settings: 0x009000 (80 bytes across 5 blocks)
 * Calibration: 0x00F000 (fixed, not wear-leveled)
 *
 * CRC over read data: CRC-16 CCITT at offset 0x7E (SYSCFG records),
 *   offsets 101-102 (CHCFG contact records).
 *
 * Address correction note: all V0.18 addresses were +0x3000 too high.
 *   V0.27 corrected offsets: SPI2_Init -0x494, all others -0x104.
 */

#include "drivers/flash_layout.h"
#include "drivers/spi.h"

#include <string.h>

/* Internal: compute flash address for a channel number -------------- */
static uint32_t channel_addr(uint16_t ch_num)
{
    return FLASH_ADDR_CHANNELS + (uint32_t)ch_num * CHANNEL_RECORD_SIZE;
}

/* ==========================================================================
 *  flash_read_channel - Read a channel record from SPI flash.
 *  Returns 0 on success, -1 if channel number is out of range.
 * ========================================================================== */

int flash_read_channel(uint16_t ch_num, channel_record_t *ch)
{
    if (ch_num >= CHANNEL_COUNT_MAX) return -1;

    uint32_t addr = channel_addr(ch_num);
    spi_flash_read(addr, (uint8_t *)ch, CHANNEL_RECORD_SIZE);
    return 0;
}

/* ==========================================================================
 *  flash_write_channel - Write a channel record to SPI flash.
 *
 *  Uses read-modify-write for the 4 KB sector containing the channel.
 *  128 channels fit per sector (128 x 32 = 4096).
 * ========================================================================== */

int flash_write_channel(uint16_t ch_num, const channel_record_t *ch)
{
    if (ch_num >= CHANNEL_COUNT_MAX) return -1;

    uint32_t addr = channel_addr(ch_num);
    uint32_t sector_base = addr & ~(FLASH_SECTOR_SIZE - 1);
    uint32_t offset_in_sector = addr - sector_base;

    /* Read entire sector into RAM */
    static uint8_t sector_buf[FLASH_SECTOR_SIZE];
    spi_flash_read(sector_base, sector_buf, FLASH_SECTOR_SIZE);

    /* Modify the channel record within the sector */
    memcpy(&sector_buf[offset_in_sector], ch, CHANNEL_RECORD_SIZE);

    /* Erase sector */
    spi_flash_erase_4k(sector_base);

    /* Write back in 256-byte pages */
    for (uint32_t off = 0; off < FLASH_SECTOR_SIZE; off += FLASH_PAGE_SIZE) {
        spi_flash_write_page(sector_base + off, &sector_buf[off], FLASH_PAGE_SIZE);
    }

    return 0;
}

/* ==========================================================================
 *  flash_read_calibration - Read calibration data from 0x00F000.
 *  Calibration is fixed (not wear-leveled). Up to FLASH_CAL_SIZE bytes.
 * ========================================================================== */

int flash_read_calibration(uint8_t *buf, uint16_t len)
{
    if (len > FLASH_CAL_SIZE) len = FLASH_CAL_SIZE;
    spi_flash_read(FLASH_ADDR_CALIBRATION, buf, len);
    return 0;
}

/* ==========================================================================
 *  flash_read_settings - Read from system settings at 0x009000.
 *  Settings span 80 bytes (5 x 16-byte blocks).
 * ========================================================================== */

int flash_read_settings(uint8_t *buf, uint16_t len)
{
    if (len > SETTINGS_TOTAL_SIZE) len = SETTINGS_TOTAL_SIZE;
    spi_flash_read(FLASH_ADDR_SETTINGS, buf, len);
    return 0;
}

/* ==========================================================================
 *  flash_write_settings - Write to system settings (sector erase + write).
 *  Settings at 0x009000 share a 4KB sector. Uses read-modify-write.
 * ========================================================================== */

int flash_write_settings(const uint8_t *buf, uint16_t len)
{
    if (len > SETTINGS_TOTAL_SIZE) len = SETTINGS_TOTAL_SIZE;

    uint32_t sector_base = FLASH_ADDR_SETTINGS & ~(FLASH_SECTOR_SIZE - 1);

    /* Read-modify-write the sector */
    static uint8_t sector_buf[FLASH_SECTOR_SIZE];
    spi_flash_read(sector_base, sector_buf, FLASH_SECTOR_SIZE);

    uint32_t offset = FLASH_ADDR_SETTINGS - sector_base;
    memcpy(&sector_buf[offset], buf, len);

    spi_flash_erase_4k(sector_base);

    for (uint16_t off = 0; off < FLASH_SECTOR_SIZE; off += FLASH_PAGE_SIZE) {
        spi_flash_write_page(sector_base + off, &sector_buf[off], FLASH_PAGE_SIZE);
    }

    return 0;
}

/* ==========================================================================
 *  Frequency encoding helpers
 * ========================================================================== */

uint32_t channel_freq_decode(const uint8_t bcd[4])
{
    /* Packed BCD -> Hz. LE digit order: byte[0] = least significant pair.
     * Each byte holds 2 BCD digits. Value represents Hz/10.
     * Example: [00,00,51,14] -> digits 14,51,00,00 -> 14510000 -> x10 = 145.1 MHz */
    uint32_t val = 0;
    for (int i = 3; i >= 0; i--) {
        uint8_t hi = (bcd[i] >> 4) & 0x0F;
        uint8_t lo = bcd[i] & 0x0F;
        if (hi > 9 || lo > 9) return 0;
        val = val * 100 + hi * 10 + lo;
    }
    return val * 10;
}

void channel_freq_encode(uint32_t hz, uint8_t bcd[4])
{
    /* Hz -> packed BCD (LE digit order). */
    if (hz == 0) { bcd[0] = bcd[1] = bcd[2] = bcd[3] = 0xFF; return; }
    uint32_t val = hz / 10;
    for (int i = 0; i < 4; i++) {
        uint8_t pair = val % 100;
        bcd[i] = ((pair / 10) << 4) | (pair % 10);
        val /= 100;
    }
}

uint32_t vfo_freq_decode(const uint8_t digits[8])
{
    /* 8 digit-per-byte -> Hz. Digits form NNN.NNNNN MHz.
     * Example: [1,4,7,2,6,5,0,0] -> 14726500 -> 147.265 MHz -> 147265000 Hz */
    uint32_t val = 0;
    for (int i = 0; i < 8; i++) {
        if (digits[i] > 9) return 0;
        val = val * 10 + digits[i];
    }
    /* val = freq in units of 10 Hz (NNN.NNNNN * 100000 = NNNNNNNNN / 10) */
    return val * 10;
}

void vfo_freq_encode(uint32_t hz, uint8_t digits[8])
{
    /* Hz -> 8 digit-per-byte. */
    uint32_t val = hz / 10;
    for (int i = 7; i >= 0; i--) {
        digits[i] = val % 10;
        val /= 10;
    }
}

/* ==========================================================================
 *  VFO access (3 records at 0x008000)
 * ========================================================================== */

int flash_read_vfo(uint8_t vfo_idx, vfo_record_t *vfo)
{
    if (vfo_idx >= VFO_COUNT) return -1;
    uint32_t addr = FLASH_ADDR_VFO + (uint32_t)vfo_idx * 32;
    spi_flash_read(addr, (uint8_t *)vfo, sizeof(vfo_record_t));
    return 0;
}

int flash_write_vfo(uint8_t vfo_idx, const vfo_record_t *vfo)
{
    if (vfo_idx >= VFO_COUNT) return -1;

    uint32_t addr = FLASH_ADDR_VFO + (uint32_t)vfo_idx * 32;
    uint32_t sector_base = addr & ~(FLASH_SECTOR_SIZE - 1);

    static uint8_t sector_buf[FLASH_SECTOR_SIZE];
    spi_flash_read(sector_base, sector_buf, FLASH_SECTOR_SIZE);

    uint32_t offset = addr - sector_base;
    memcpy(&sector_buf[offset], vfo, sizeof(vfo_record_t));

    spi_flash_erase_4k(sector_base);

    for (uint16_t off = 0; off < FLASH_SECTOR_SIZE; off += FLASH_PAGE_SIZE) {
        spi_flash_write_page(sector_base + off, &sector_buf[off], FLASH_PAGE_SIZE);
    }
    return 0;
}
