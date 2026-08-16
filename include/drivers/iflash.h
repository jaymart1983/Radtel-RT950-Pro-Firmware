/*
 * iflash.h - Internal (MCU) flash erase/program for the AT32F403A
 *
 * Distinct from drivers/spi.h, which drives the EXTERNAL W25Q16 holding
 * channels and settings. This one writes the MCU's own program flash, which is
 * what a firmware update actually modifies.
 *
 * Why this exists: the in-application updater. Radtel's own firmware does not
 * hand off to the bootloader to update itself -- the RT-900 source shows the
 * application entering MODE_FLASH_PROGRAM and writing flash directly, then
 * resetting when it is done. The bootloader is for recovery only, reachable by
 * holding the side buttons or via an SPI-flash marker; there is no soft entry
 * into it, which is confirmed by its own main(): it checks the buttons and the
 * SPI marker and nothing else.
 *
 * SAFETY -- read before changing anything here:
 *
 *   The bootloader lives at 0x08000000-0x08002FFF and MUST never be written.
 *   It is the only recovery path if an update goes wrong. Every function here
 *   refuses addresses below IFLASH_APP_BASE, and that check is not optional.
 *
 *   Code cannot erase or program the sector it is executing from. The updater
 *   must therefore run entirely from RAM, or -- as here -- write only the
 *   application region while executing from a copy of the routine placed in
 *   RAM. See iflash_program_block(), which is marked for RAM placement.
 */

#ifndef DRIVERS_IFLASH_H
#define DRIVERS_IFLASH_H

#include <stdint.h>

/* Application region. Below this is the bootloader -- never touch it. */
#define IFLASH_APP_BASE     0x08003000UL
#define IFLASH_END          0x08080000UL   /* 512 KB part */
#define IFLASH_SECTOR_SIZE  2048UL         /* AT32F403A erase granularity */

typedef enum {
    IFLASH_OK = 0,
    IFLASH_ERR_RANGE,       /* address outside the application region */
    IFLASH_ERR_ALIGN,       /* not halfword aligned */
    IFLASH_ERR_BUSY,        /* controller stuck busy */
    IFLASH_ERR_PROGRAM,     /* PGERR: programming fault */
    IFLASH_ERR_WRPRT,       /* WRPRTERR: write protected */
    IFLASH_ERR_VERIFY,      /* readback did not match */
} iflash_status_t;

/* Unlock the flash controller. Safe to call repeatedly. */
void iflash_unlock(void);

/* Re-lock. Call when finished, so stray writes cannot corrupt flash. */
void iflash_lock(void);

/* Erase the 2 KB sector containing addr. Refuses anything below the
 * application base. */
iflash_status_t iflash_erase_sector(uint32_t addr);

/* Program len bytes at addr, halfword at a time, then verify by readback.
 * addr and len must both be halfword aligned. Refuses to write below the
 * application base. Erases each sector as it is first touched. */
iflash_status_t iflash_program(uint32_t addr, const uint8_t *data, uint32_t len);

/* Reset the MCU. Used once an update completes -- the same thing the RT-900
 * does at the end of its programming loop. */
void iflash_system_reset(void) __attribute__((noreturn));

#endif /* DRIVERS_IFLASH_H */
