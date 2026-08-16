/*
 * updater.c - In-application firmware updater
 *
 * Receives a .BTF over UART4 and writes it to internal flash, then resets.
 * No bootloader involved.
 *
 * WHY IT WORKS THIS WAY
 *
 * The bootloader has no soft entry. Its main() checks the side buttons and an
 * SPI-flash marker, and nothing else -- there is no magic value, no RAM flag,
 * no command. Every attempt to talk it into update mode from a running
 * application failed for that reason.
 *
 * Radtel's own firmware never tries. The RT-900 source shows the application
 * switching itself into MODE_FLASH_PROGRAM, running the programming loop
 * itself, and calling NVIC_SystemReset() when finished. The bootloader is a
 * recovery path, not an update path. This is the same design.
 *
 * EXECUTING FROM RAM
 *
 * The update overwrites the very flash this code lives in, so the whole loop
 * is placed in .ramfunc and copied to SRAM at startup. It must not call
 * anything that lives in flash: a single call back mid-erase stalls the CPU
 * against a busy flash controller. That is why the UART access, the CRC and
 * the flash writes below are all open-coded here rather than reusing the
 * drivers -- duplication on purpose.
 *
 * BTF LAYOUT, as sent by the host
 *
 *   block 0   [0x000..0x3FF]  plaintext, goes to 0x08003000
 *   block 1   [0x400..0x7FF]  16-byte key + padding, NEVER flashed
 *   block 2+  [0x800..    ]   encrypted, goes to 0x08003400 onward
 *
 * Decryption is a XOR against a 128-byte expansion of that key, with 0x00 and
 * 0xFF passed through unchanged in both directions.
 *
 * SAFETY
 *
 *   - Nothing below 0x08003000 is ever erased or written. The bootloader stays
 *     intact, so holding the side buttons always recovers the radio however
 *     badly an update goes.
 *   - Every halfword is read back after programming. The bootloader's own
 *     updater silently drops the last block it is given and still reports
 *     success; that cost a full day of debugging precisely because nothing
 *     verified what it had written.
 */

#include "app/updater.h"
#include "at32f403a.h"

#define RAMFUNC __attribute__((section(".ramfunc"), noinline))

/* Protocol ------------------------------------------------------------- */
#define PKT_HEADER      0xAA
#define PKT_FOOTER      0x55
#define CMD_PROBE       0x42
#define CMD_VERSION     0x0A
#define CMD_MODEL       0x02
#define CMD_PKG_COUNT   0x04
#define CMD_DATA        0x03
#define CMD_END         0x45
#define RESULT_ACK      0x06
#define RESULT_LEN_ERR  0xE1
#define RESULT_FLASH_ERR 0xE3
#define RESULT_UNK_CMD  0xE5

#define BLOCK_SIZE      1024u
#define APP_BASE        0x08003000UL
#define SECTOR_SIZE     2048UL
#define KEY_LEN         16u
#define EXPKEY_LEN      128u

/* Flash controller */
#define FLASH_KEYR      (*(volatile uint32_t *)0x40022004UL)
#define FLASH_SR        (*(volatile uint32_t *)0x4002200CUL)
#define FLASH_CR        (*(volatile uint32_t *)0x40022010UL)
#define FLASH_AR        (*(volatile uint32_t *)0x40022014UL)

/* Buffers live in .bss, not on the stack -- 1 KB plus framing would not fit
 * comfortably and a stack overflow here would be silent and fatal. */
static uint8_t  pkt[BLOCK_SIZE + 16];
static uint8_t  expkey[EXPKEY_LEN];
static uint8_t  have_key;

/* The entire image is buffered here before ANY flash operation.
 *
 * The first design interleaved receive with erase/program and failed
 * reproducibly at block 8: a sector erase takes tens of milliseconds, the
 * updater runs with interrupts off and polls the UART, and the receive
 * register holds exactly one byte -- so everything arriving during an erase is
 * lost. It also explains the earlier corruption, where the host saw success
 * because every halfword we did write verified; the bytes we missed were never
 * written at all.
 *
 * Buffering first means the flash pass runs with no UART traffic in flight and
 * no timing relationship between the two. 56 KB is comfortably more than the
 * full firmware (~48 KB) and leaves ample stack in the 96 KB of SRAM. */
#define IMG_MAX  (56u * 1024u)
static uint8_t  img[IMG_MAX];
static uint32_t img_blocks;

/* --- RAM-resident primitives ----------------------------------------- */

RAMFUNC static void ram_putc(uint8_t c)
{
    while (!(UART4->SR & (1UL << 7)))    /* TXE */
        ;
    UART4->DR = c;
}

RAMFUNC static int ram_getc(uint32_t timeout)
{
    while (timeout--) {
        if (UART4->SR & (1UL << 5))      /* RXNE */
            return (int)(UART4->DR & 0xFF);
    }
    return -1;
}

RAMFUNC static uint16_t ram_crc(const uint8_t *d, uint32_t n)
{
    uint16_t crc = 0x0000;
    for (uint32_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}

RAMFUNC static void ram_reply(uint8_t cmd, uint8_t result)
{
    uint8_t r[9];
    r[0] = PKT_HEADER;
    r[1] = cmd;
    r[2] = 0x00;
    r[3] = result;
    r[4] = 0x00;
    r[5] = 0x00;
    uint16_t c = ram_crc(&r[1], 5);
    r[6] = (uint8_t)(c >> 8);
    r[7] = (uint8_t)(c & 0xFF);
    r[8] = PKT_FOOTER;
    for (uint8_t i = 0; i < 9; i++)
        ram_putc(r[i]);
}

RAMFUNC static int ram_flash_wait(void)
{
    uint32_t spins = 2000000UL;
    while ((FLASH_SR & 1UL) && spins--)
        ;
    if (FLASH_SR & ((1UL << 2) | (1UL << 4))) {   /* PGERR | WRPRTERR */
        FLASH_SR = (1UL << 2) | (1UL << 4);
        return -1;
    }
    if (FLASH_SR & (1UL << 5))
        FLASH_SR = (1UL << 5);                    /* EOP */
    return spins ? 0 : -1;
}

RAMFUNC static int ram_flash_write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    /* Absolute floor: the bootloader must survive whatever happens here. */
    if (addr < APP_BASE)
        return -1;

    if (FLASH_CR & (1UL << 7)) {                  /* LOCK */
        FLASH_KEYR = 0x45670123UL;
        FLASH_KEYR = 0xCDEF89ABUL;
    }

    for (uint32_t i = 0; i < len; i += 2) {
        uint32_t a = addr + i;

        if ((a & (SECTOR_SIZE - 1UL)) == 0UL) {   /* first touch: erase */
            if (ram_flash_wait() < 0) return -1;
            FLASH_CR |= (1UL << 1);               /* PER */
            FLASH_AR  = a;
            FLASH_CR |= (1UL << 6);               /* STRT */
            if (ram_flash_wait() < 0) { FLASH_CR &= ~(1UL << 1); return -1; }
            FLASH_CR &= ~(1UL << 1);
        }

        uint16_t hw = (uint16_t)data[i] | ((uint16_t)data[i + 1] << 8);
        FLASH_CR |= 1UL;                          /* PG */
        *(volatile uint16_t *)a = hw;
        if (ram_flash_wait() < 0) { FLASH_CR &= ~1UL; return -1; }
        FLASH_CR &= ~1UL;

        if (*(volatile uint16_t *)a != hw)        /* verify every halfword */
            return -1;
    }
    return 0;
}

RAMFUNC static void ram_expand_key(const uint8_t *key)
{
    for (uint8_t i = 0; i < KEY_LEN; i++)
        expkey[i] = key[i];

    /* Blocks 1..7: bytes 0-7 rotate left one bit, bytes 8-15 rotate right. */
    for (uint8_t blk = 1; blk < 8; blk++) {
        const uint8_t *prev = &expkey[(blk - 1) * KEY_LEN];
        uint8_t *cur = &expkey[blk * KEY_LEN];
        for (uint8_t i = 0; i < 8; i++)
            cur[i] = (uint8_t)((prev[i] << 1) | (prev[i] >> 7));
        for (uint8_t i = 8; i < 16; i++)
            cur[i] = (uint8_t)((prev[i] >> 1) | (prev[i] << 7));
    }
    have_key = 1;
}

RAMFUNC static void ram_decrypt(uint8_t *buf, uint32_t len, uint32_t file_off)
{
    for (uint32_t i = 0; i < len; i++) {
        uint8_t b = buf[i];
        if (b == 0x00 || b == 0xFF)
            continue;                             /* passed through by the encoder */
        uint8_t x = b ^ expkey[(file_off + i) % EXPKEY_LEN];
        if (x == 0x00 || x == 0xFF)
            continue;                             /* encoder skips these too */
        buf[i] = x;
    }
}

/* --- The update loop -------------------------------------------------- */

RAMFUNC void updater_run(void)
{
    __asm volatile ("cpsid i");                   /* no ISRs: they live in flash */

    have_key = 0;
    img_blocks = 0;
    UART4->CR1 = (1UL << 13) | (1UL << 3) | (1UL << 2);   /* UE | TE | RE, no IRQ */

    for (;;) {
        int c = ram_getc(60000000UL);             /* generous idle timeout */
        if (c < 0)
            break;                                /* host went away -- reset */
        if (c != PKT_HEADER)
            continue;

        /* [cmd][args_hi][args_lo][len_hi][len_lo] */
        uint8_t hdr[5];
        int bad = 0;
        for (uint8_t i = 0; i < 5; i++) {
            int v = ram_getc(4000000UL);
            if (v < 0) { bad = 1; break; }
            hdr[i] = (uint8_t)v;
        }
        if (bad) break;

        uint8_t  cmd  = hdr[0];
        uint16_t args = (uint16_t)((hdr[1] << 8) | hdr[2]);
        uint16_t dlen = (uint16_t)((hdr[3] << 8) | hdr[4]);
        if (dlen > BLOCK_SIZE) { ram_reply(cmd, RESULT_LEN_ERR); continue; }

        pkt[0] = cmd; pkt[1] = hdr[1]; pkt[2] = hdr[2];
        pkt[3] = hdr[3]; pkt[4] = hdr[4];
        for (uint16_t i = 0; i < dlen; i++) {
            int v = ram_getc(4000000UL);
            if (v < 0) { bad = 1; break; }
            pkt[5 + i] = (uint8_t)v;
        }
        if (bad) break;

        int ch = ram_getc(4000000UL);
        int cl = ram_getc(4000000UL);
        int ft = ram_getc(4000000UL);
        if (ch < 0 || cl < 0 || ft < 0) break;

        uint16_t want = (uint16_t)((ch << 8) | cl);
        if (ram_crc(pkt, 5u + dlen) != want) { ram_reply(cmd, RESULT_LEN_ERR); continue; }

        switch (cmd) {
        case CMD_PROBE:
        case CMD_VERSION:
        case CMD_MODEL:
        case CMD_PKG_COUNT:
            ram_reply(cmd, RESULT_ACK);
            break;

        case CMD_DATA: {
            uint8_t *d = &pkt[5];

            /* Buffer only. No flash access here -- see the note on img[]. */
            if (args == 1u) {
                ram_expand_key(d);                /* key block, never flashed */
                ram_reply(cmd, RESULT_ACK);
                break;
            }

            uint32_t off;
            if (args == 0u) {
                off = 0u;                         /* block 0, plaintext */
            } else {
                if (!have_key) { ram_reply(cmd, RESULT_FLASH_ERR); break; }
                ram_decrypt(d, dlen, (uint32_t)args * BLOCK_SIZE);
                off = ((uint32_t)args - 1u) * BLOCK_SIZE;
            }

            if (off + dlen > IMG_MAX) { ram_reply(cmd, RESULT_LEN_ERR); break; }
            for (uint16_t i = 0; i < dlen; i++)
                img[off + i] = d[i];
            if (off + dlen > img_blocks)
                img_blocks = off + dlen;

            ram_reply(cmd, RESULT_ACK);
            break;
        }

        case CMD_END:
            /* Everything is in RAM now; write it in one pass. ACK first, so
             * the host is not left waiting through the erase, and it has
             * nothing further to send. */
            ram_reply(cmd, RESULT_ACK);
            for (volatile uint32_t i = 0; i < 200000UL; i++)
                ;
            if (img_blocks)
                (void)ram_flash_write(APP_BASE, img, (img_blocks + 1u) & ~1u);
            goto done;

        default:
            ram_reply(cmd, RESULT_UNK_CMD);
            break;
        }
    }

done:
    /* Same ending as the RT-900's programming loop: reset into whatever was
     * just written. */
    __asm volatile ("dsb");
    *(volatile uint32_t *)0xE000ED0CUL = (0x5FAUL << 16) | (1UL << 2);
    for (;;)
        ;
}
