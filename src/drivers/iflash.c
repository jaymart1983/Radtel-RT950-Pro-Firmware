/*
 * iflash.c - Internal flash erase/program for the AT32F403A
 *
 * See iflash.h for why this exists and for the safety rules.
 *
 * Controller register map (0x40022000), the usual STM32F1-compatible layout:
 *
 *   +0x00 ACR     access control
 *   +0x04 KEYR    unlock: write 0x45670123 then 0xCDEF89AB
 *   +0x08 OPTKEYR option byte unlock
 *   +0x0C SR      status: BSY[0] PGERR[2] WRPRTERR[4] EOP[5]
 *   +0x10 CR      control: PG[0] PER[1] MER[2] OPTPG[4] STRT[6] LOCK[7]
 *   +0x14 AR      address
 */

#include "drivers/iflash.h"

#define FLASH_BASE_REG  0x40022000UL

#define FLASH_KEYR      (*(volatile uint32_t *)(FLASH_BASE_REG + 0x04))
#define FLASH_SR        (*(volatile uint32_t *)(FLASH_BASE_REG + 0x0C))
#define FLASH_CR        (*(volatile uint32_t *)(FLASH_BASE_REG + 0x10))
#define FLASH_AR        (*(volatile uint32_t *)(FLASH_BASE_REG + 0x14))

#define SR_BSY          (1UL << 0)
#define SR_PGERR        (1UL << 2)
#define SR_WRPRTERR     (1UL << 4)
#define SR_EOP          (1UL << 5)

#define CR_PG           (1UL << 0)
#define CR_PER          (1UL << 1)
#define CR_STRT         (1UL << 6)
#define CR_LOCK         (1UL << 7)

#define KEY1            0x45670123UL
#define KEY2            0xCDEF89ABUL

/* Generous bound: a sector erase is milliseconds, and this only has to stop a
 * genuinely wedged controller from hanging the radio forever. */
#define BUSY_TIMEOUT    2000000UL

static iflash_status_t wait_ready(void)
{
    uint32_t spins = 0;
    while (FLASH_SR & SR_BSY) {
        if (++spins > BUSY_TIMEOUT)
            return IFLASH_ERR_BUSY;
    }

    if (FLASH_SR & SR_WRPRTERR) {
        FLASH_SR = SR_WRPRTERR;
        return IFLASH_ERR_WRPRT;
    }
    if (FLASH_SR & SR_PGERR) {
        FLASH_SR = SR_PGERR;
        return IFLASH_ERR_PROGRAM;
    }
    if (FLASH_SR & SR_EOP)
        FLASH_SR = SR_EOP;          /* write-1-to-clear */

    return IFLASH_OK;
}

void iflash_unlock(void)
{
    if (FLASH_CR & CR_LOCK) {
        FLASH_KEYR = KEY1;
        FLASH_KEYR = KEY2;
    }
}

void iflash_lock(void)
{
    FLASH_CR |= CR_LOCK;
}

iflash_status_t iflash_erase_sector(uint32_t addr)
{
    /* The bootloader is the only way back from a bad update. Nothing below the
     * application base is erasable through this driver, ever. */
    if (addr < IFLASH_APP_BASE || addr >= IFLASH_END)
        return IFLASH_ERR_RANGE;

    iflash_status_t st = wait_ready();
    if (st != IFLASH_OK)
        return st;

    iflash_unlock();

    FLASH_CR |= CR_PER;
    FLASH_AR  = addr & ~(IFLASH_SECTOR_SIZE - 1UL);
    FLASH_CR |= CR_STRT;

    st = wait_ready();
    FLASH_CR &= ~CR_PER;
    return st;
}

iflash_status_t iflash_program(uint32_t addr, const uint8_t *data, uint32_t len)
{
    if (addr < IFLASH_APP_BASE || (addr + len) > IFLASH_END)
        return IFLASH_ERR_RANGE;
    if ((addr & 1UL) || (len & 1UL))
        return IFLASH_ERR_ALIGN;

    iflash_status_t st = wait_ready();
    if (st != IFLASH_OK)
        return st;

    iflash_unlock();

    for (uint32_t i = 0; i < len; i += 2) {
        uint32_t a = addr + i;

        /* Erase a sector the first time we touch it. Doing it here rather than
         * up front means a partial image never erases more than it writes. */
        if ((a & (IFLASH_SECTOR_SIZE - 1UL)) == 0UL) {
            st = iflash_erase_sector(a);
            if (st != IFLASH_OK)
                return st;
            iflash_unlock();
        }

        uint16_t hw = (uint16_t)data[i] | ((uint16_t)data[i + 1] << 8);

        FLASH_CR |= CR_PG;
        *(volatile uint16_t *)a = hw;
        st = wait_ready();
        FLASH_CR &= ~CR_PG;
        if (st != IFLASH_OK)
            return st;

        /* Verify immediately. A silent bad write here is exactly the failure
         * that cost this project a day: the bootloader's own updater drops the
         * last block it is given and reports success, and nothing noticed
         * because nothing read back what it wrote. */
        if (*(volatile uint16_t *)a != hw)
            return IFLASH_ERR_VERIFY;
    }

    return IFLASH_OK;
}

void iflash_system_reset(void)
{
    /* AIRCR: VECTKEY 0x05FA, SYSRESETREQ bit 2. Same call the RT-900 makes at
     * the end of its own programming loop. */
    __asm volatile ("dsb");
    *(volatile uint32_t *)0xE000ED0CUL = (0x5FAUL << 16) | (1UL << 2);
    __asm volatile ("dsb");
    for (;;)
        ;
}
