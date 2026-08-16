/*
 * debug_uart.c - Polled UART4 debug output for boot diagnostics
 *
 * Compiled only when DEBUG_UART is defined (make DEBUG=1).
 * Uses bare register writes with no dependencies on other drivers
 * so it can be called as early as SystemInit.
 */

#ifdef DEBUG_UART

#include "at32f403a.h"
#include "rt950_pinmap.h"

/* Polled TX: spin until transmit buffer empty, then write */
static void dbg_putc(char c)
{
    while (!(UART4->SR & USART_SR_TXE))
        ;
    UART4->DR = (uint8_t)c;
}

/* Wait for transmission to fully complete */
static void dbg_flush(void)
{
    while (!(UART4->SR & USART_SR_TC))
        ;
}

void dbg_init(void)
{
    /* Enable UART4 + GPIOC clocks */
    CRM->APB1EN |= CRM_APB1EN_UART4EN;
    CRM->APB2EN |= CRM_APB2EN_IOPCEN;

    /* PC10 = AF push-pull (UART4_TX), 2 MHz */
    volatile uint32_t *crh = &CPS_TX_PORT->CRH;
    uint32_t val = *crh;
    /* PC10: CRH bits [11:8] - mode=0b10 (2MHz), cnf=0b10 (AF_PP) */
    val &= ~(0xFUL << 8);
    val |=  (0xAUL << 8);   /* 0xA = cnf:10 mode:10 */
    *crh = val;

    /* PC11 = floating input (UART4_RX) */
    /* CRH bits [15:12] - mode=0b00 (input), cnf=0b01 (floating) */
    val = *crh;
    val &= ~(0xFUL << 12);
    val |=  (0x4UL << 12);  /* 0x4 = cnf:01 mode:00 */
    *crh = val;

    /* Clear framing config left over from the bootloader, which has just been
     * using this UART for the firmware upload. Leaving CR2/CR3 alone produced
     * consistently corrupted bytes that looked like a baud error. */
    UART4->CR2 = 0;
    UART4->CR3 = 0;

    /* 115200 baud: APB1 = 60 MHz, BRR = 60000000/115200 = 521 */
    UART4->BRR = 521;
    /* Enable UART, TX only (no RX, no interrupts) */
    UART4->CR1 = USART_CR1_TE | USART_CR1_UE;

    dbg_flush();
}

void dbg_puts(const char *s)
{
    while (*s) {
        if (*s == '\n')
            dbg_putc('\r');
        dbg_putc(*s++);
    }
}

void dbg_hex8(unsigned char v)
{
    /* Arithmetic hex conversion - avoids .rodata lookup table which
     * gets corrupted by BTF encryption on larger builds. */
    unsigned char hi = v >> 4;
    unsigned char lo = v & 0xF;
    dbg_putc(hi < 10 ? '0' + hi : 'A' - 10 + hi);
    dbg_putc(lo < 10 ? '0' + lo : 'A' - 10 + lo);
}

void dbg_hex32(unsigned long v)
{
    dbg_hex8((unsigned char)(v >> 24));
    dbg_hex8((unsigned char)(v >> 16));
    dbg_hex8((unsigned char)(v >> 8));
    dbg_hex8((unsigned char)(v));
}

void dbg_newline(void)
{
    dbg_putc('\r');
    dbg_putc('\n');
}

void dbg_reg(const char *label, unsigned long val)
{
    dbg_puts(label);
    dbg_hex32(val);
    dbg_newline();
}

#endif /* DEBUG_UART */
