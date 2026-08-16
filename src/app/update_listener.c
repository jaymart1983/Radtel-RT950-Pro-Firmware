/*
 * update_listener.c - Soft entry into the bootloader's UART update mode
 *
 * See update_listener.h for the design and for what is unverified.
 */

#include "app/update_listener.h"

#ifndef NO_UPDATE_LISTENER

#include "at32f403a.h"
#include "debug_uart.h"
#include "drivers/lcd.h"

/* Host handshake, byte for byte as firmware_upload.py sends it. */
static const char HANDSHAKE[] = "PROGRAMBT9000U";
static const char UPDATE[]    = "UPDATE";

#define HANDSHAKE_LEN  (sizeof(HANDSHAKE) - 1)
#define UPDATE_LEN     (sizeof(UPDATE) - 1)

#define ACK_BYTE       0x06

/* Bootloader occupies 0x08000000-0x08002FFF. Word 0 is its initial SP,
 * word 1 its reset vector. */
#define BOOTLOADER_BASE 0x08000000UL

/* The two pins the bootloader samples to decide whether to enter update mode.
 * Both active low. PE5 is the top side button, PA12 the bottom one -- the pair
 * you hold while powering on. */
#define UPD_PIN_E5      (1UL << 5)
#define UPD_PIN_A12     (1UL << 12)

/* NVIC: UART4 is IRQ52. */
#define UART4_IRQn      52

static volatile uint8_t hs_matched;   /* bytes of HANDSHAKE matched so far */
static volatile uint8_t up_matched;   /* bytes of UPDATE matched so far */
static volatile uint8_t stage;        /* 0 = want handshake, 1 = want UPDATE */
static volatile uint8_t triggered;

/* Diagnostics. Distinguishes three very different failures that all present
 * identically as "the handshake did not work":
 *   rx_count == 0          -> nothing is being received at all (RX path dead)
 *   rx_count > 0, stage 0  -> bytes arrive but never match the handshake
 *   stage == 1             -> handshake matched, UPDATE did not follow
 */
static volatile uint32_t rx_count;
static volatile uint8_t  last_byte;

/* First bytes received, captured verbatim. rx_count proved the RX path works
 * (exactly 14 bytes arrive for a 14-byte handshake) while the matcher never
 * matched, so the transport is fine and the DATA is wrong. Nothing short of
 * the raw bytes will say why. */
#define RX_CAP 16
static volatile uint8_t  rx_cap[RX_CAP];
static volatile uint8_t  rx_cap_n;

uint8_t update_listener_cap_count(void) { return rx_cap_n; }

/* Read HANDSHAKE[] through the SAME symbol the matcher uses. Printing the
 * string literal from another translation unit proves nothing about this
 * array -- and 'P' matching while 'R' does not says this array is not what
 * the source appears to say. */
uint8_t update_listener_hs_byte(uint8_t i)
{
    return (i < HANDSHAKE_LEN) ? (uint8_t)HANDSHAKE[i] : 0;
}
uint8_t update_listener_hs_len(void) { return (uint8_t)HANDSHAKE_LEN; }
uint8_t update_listener_cap_byte(uint8_t i)
{
    return (i < RX_CAP) ? rx_cap[i] : 0;
}

uint32_t update_listener_rx_count(void) { return rx_count; }
uint8_t  update_listener_last_byte(void) { return last_byte; }
uint8_t  update_listener_stage(void)     { return stage; }
uint8_t  update_listener_matched(void)   { return hs_matched; }

uint8_t update_listener_triggered(void) { return triggered; }

/* Blocking single-byte TX. Used only to ACK, where a couple of character times
 * of spinning inside the ISR is harmless and far simpler than wiring up TX
 * interrupts for two bytes. */
static void uart4_putc_blocking(uint8_t c)
{
    while (!(UART4->SR & USART_SR_TXE))
        ;
    UART4->DR = c;
    while (!(UART4->SR & USART_SR_TC))
        ;
}

void update_listener_init(void)
{
    hs_matched = up_matched = stage = triggered = 0;

    /* UART4 and GPIOC clocks. dbg_init() may already have done this; both are
     * set-only so doing it twice is harmless, and this must not depend on the
     * debug build being selected. */
    CRM->APB1EN |= CRM_APB1EN_UART4EN;
    CRM->APB2EN |= CRM_APB2EN_IOPCEN;

    /* PC11 = UART4_RX, floating input. CRH bits [15:12] = 0x4 (cnf 01, mode 00).
     * PC10 (TX) is deliberately not touched -- the debug UART owns it, and
     * TX-only debug output and RX-only listening coexist on the same
     * peripheral without interfering. */
    {
        volatile uint32_t *crh = &GPIOC->CRH;
        uint32_t v = *crh;
        v &= ~(0xFUL << 12);
        v |=  (0x4UL << 12);
        *crh = v;
    }

    /* 115200 at APB1 = 60 MHz. Set unconditionally: in a non-debug build
     * nothing else has configured this UART yet. */
    UART4->BRR = 521;

    /* Clear any stale receive state BEFORE enabling the interrupt.
     *
     * The bootloader has just finished an upload over this same UART, so the
     * overrun flag is very likely set and a byte may be sitting in DR. Enabling
     * RXNEIE on top of that immediately storms the ISR. Reading SR then DR
     * clears ORE/FE/NE/PE and empties the register. */
    (void)UART4->SR;
    (void)UART4->DR;

    /* Keep whatever TX configuration is already present (the debug UART sets
     * TE|UE) and add RX plus the RX-not-empty interrupt. uart_cps_init() may
     * later overwrite CR1 wholesale, but it sets the same RX bits, so the
     * listener keeps working either way. */
    UART4->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    /* Drop any interrupt that went pending while we were setting up, then
     * enable. ICPR/ISER are one bit per IRQ and ignore zero writes, so neither
     * disturbs other interrupts. */
    *(volatile uint32_t *)(0xE000E280UL + (UART4_IRQn / 32) * 4UL)
        = (1UL << (UART4_IRQn % 32));
    *(volatile uint32_t *)(0xE000E100UL + (UART4_IRQn / 32) * 4UL)
        = (1UL << (UART4_IRQn % 32));

    dbg_puts("[UPD] update listener armed (UART4 RX)\n");
}

void update_listener_enter_bootloader(void)
{
    dbg_puts("[UPD] entering bootloader update mode\n");

    /* Flag the handover on screen before anything else -- if the radio dies
     * during the jump, the colour says how far it got. */
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, 0xF800);   /* red */

    __asm volatile ("cpsid i");

    /* Call the bootloader's UART update mode DIRECTLY, rather than driving the
     * side-button pins low and branching to its reset vector.
     *
     * The pin trick does not work: the listener matches, both ACKs go out, and
     * then the bootloader boots the application again instead of staying in
     * update mode -- its own gpio_init() runs before the button check and takes
     * those pins back, so whatever we drove is gone by the time it looks.
     *
     * Entering uart_update_mode() directly skips the button test altogether.
     * Doing so means the bootloader's own startup has not run, so its .data
     * must be placed by hand first -- addresses from the bootloader's init
     * table at 0x08002CB4 (see docs/bootloader.md):
     *
     *     0x34  bytes from flash 0x08002D68 -> SRAM 0x20000000   (.data)
     *     0x884 bytes from flash 0x08002D9C -> SRAM 0x20000034   (.bss init)
     *
     * The bootloader region is never rewritten by an application upload, so
     * these addresses are stable. If this is wrong the radio hangs and the side
     * buttons still recover it -- the same safety net as before. */

    /* Peripherals we have been using must be quiet before handing over. */
    UART4->CR1 = 0;
    *(volatile uint32_t *)0xE000E180UL = 0xFFFFFFFFUL;   /* NVIC ICER0 */
    *(volatile uint32_t *)0xE000E184UL = 0xFFFFFFFFUL;
    *(volatile uint32_t *)0xE000E188UL = 0xFFFFFFFFUL;
    *(volatile uint32_t *)0xE000E280UL = 0xFFFFFFFFUL;   /* ICPR0 */
    *(volatile uint32_t *)0xE000E284UL = 0xFFFFFFFFUL;
    *(volatile uint32_t *)0xE000E288UL = 0xFFFFFFFFUL;
    SysTick->CTRL = 0;

    /* Recreate the bootloader's initialised data. */
    {
        const volatile uint8_t *src = (const volatile uint8_t *)0x08002D68UL;
        volatile uint8_t *dst = (volatile uint8_t *)0x20000000UL;
        for (uint32_t i = 0; i < 0x34UL; i++) dst[i] = src[i];

        src = (const volatile uint8_t *)0x08002D9CUL;
        dst = (volatile uint8_t *)0x20000034UL;
        for (uint32_t i = 0; i < 0x884UL; i++) dst[i] = src[i];
    }

    SCB->VTOR = BOOTLOADER_BASE;
    __asm volatile ("msr msp, %0" : : "r" (*(volatile uint32_t *)BOOTLOADER_BASE));

    /* Call as little of the bootloader as possible.
     *
     * The first attempt replicated its main() -- gpio_init, "lcd_init",
     * uart_init -- and died partway: the screen went black (so bootloader code
     * really did run) but the update loop never drew its UPDATE banner. One of
     * those calls was also plain wrong: 0x080003F8 is lcd_gpio_init, not the
     * ST7789 init at 0x08001588, which the doc's prose and its own function
     * table disagree about.
     *
     * uart_update_mode() needs the UART, and the UART is already configured at
     * 115200 by us. gpio_init and the LCD are irrelevant to a serial update.
     * So call only uart_init -- to let the bootloader set the UART up its own
     * way -- and then the update loop. Fewer calls, fewer guessed addresses,
     * fewer ways to be wrong. */
    ((void (*)(void))(0x0800214CUL | 1UL))();   /* uart_init */

    __asm volatile ("cpsie i");

    ((void (*)(void))(0x08000224UL | 1UL))();   /* uart_update_mode, no return */

    for (;;)
        ;
}

void update_listener_feed(uint8_t c)
{
    rx_count++;
    last_byte = c;
    if (rx_cap_n < RX_CAP) rx_cap[rx_cap_n++] = c;

    if (stage == 0) {
        /* Match PROGRAMBT9000U. On a mismatch, retry the current byte as the
         * first character rather than dropping it outright, so a partial match
         * immediately followed by a real handshake still lands. */
        if (c == (uint8_t)HANDSHAKE[hs_matched]) {
            if (++hs_matched == HANDSHAKE_LEN) {
                hs_matched = 0;
                stage = 1;
                uart4_putc_blocking(ACK_BYTE);
            }
        } else {
            hs_matched = (c == (uint8_t)HANDSHAKE[0]) ? 1 : 0;
        }
        return;
    }

    /* stage 1: match UPDATE */
    if (c == (uint8_t)UPDATE[up_matched]) {
        if (++up_matched == UPDATE_LEN) {
            up_matched = 0;
            triggered = 1;
            uart4_putc_blocking(ACK_BYTE);
            update_listener_enter_bootloader();   /* does not return */
        }
    } else {
        up_matched = (c == (uint8_t)UPDATE[0]) ? 1 : 0;
        /* A stray byte after the handshake should not strand us waiting for an
         * UPDATE that never comes -- fall back to listening for a fresh
         * handshake as well. */
        if (c == (uint8_t)HANDSHAKE[0]) {
            stage = 0;
            hs_matched = 1;
        }
    }
}

#endif /* NO_UPDATE_LISTENER */
