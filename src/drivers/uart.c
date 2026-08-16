/*
 * uart.c - Interrupt-driven UART driver for the RT-950 Pro
 *
 * V0.27 firmware UART configuration (all use identical SDK pattern):
 *   USART1 (BT)  @ fw 0x08007536 - 115200 baud, PA9/PA10, 0x40013800
 *   USART3 (GPS) @ fw 0x08013B62 - 9600 baud, PB10/PB11, 0x40004800
 *   UART4  (CPS) @ fw 0x08022762 - 115200 baud, PC10/PC11, 0x40004C00
 *   USART2 - unused in OEM firmware
 *
 * OEM SDK (shared by all 3 UARTs):
 *   USART_Init     @ fw 0x0802240C  (BRR with rounding algorithm)
 *   USART_Enable   @ fw 0x08022308  (CR1 bit 13: UEN)
 *   USART_IntEnable@ fw 0x080223D4  (0x525 = CR1 bit 5: RDBFIEN)
 *
 * BRR values (SDK rounds: temp=(clk*25)/(4*baud), BRR=(m<<4)|frac):
 *   USART1/UART4: 521 (0x0209) for 115200 @ APB2/APB1 60 MHz
 *   USART3:      6250 (0x186A) for 9600 @ APB1 60 MHz
 *
 * OEM UART ISR note: all USART/UART vector entries alias into CRC-16
 *   function body @ fw 0x08024E52 (ISR handler @ fw 0x08025294 is
 *   unreachable). OEM uses polled I/O. Our implementation upgrades to
 *   RXNE interrupts with 256-byte ring buffers (NVIC priorities: 5/5/6).
 *
 * OEM USART1 ring buffer @ 0x2000AF70: 140 bytes at +0x10,
 *   write index at +0x04, timeout at +0x06 (reset to 2000 per byte).
 *
 * See also: docs/cps-uart.md, docs/bluetooth.md
 */

#include "drivers/uart.h"
#include "app/update_listener.h"
#include "drivers/gpio.h"
#include "rt950_pinmap.h"
#include "cortex_m4.h"

/* BT ring buffer (USART1) */
static volatile uint8_t  bt_rx_buf[UART_BT_BUF_SIZE];
static volatile uint16_t bt_rx_head;
static volatile uint16_t bt_rx_tail;

/* CPS ring buffer (UART4) */
static volatile uint8_t  cps_rx_buf[UART_CPS_BUF_SIZE];
static volatile uint16_t cps_rx_head;
static volatile uint16_t cps_rx_tail;

/* USART1 ISR - Bluetooth RX */

void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE) {
        uint8_t ch = (uint8_t)(USART1->DR & 0xFF);
        uint16_t next = (bt_rx_head + 1) & (UART_BT_BUF_SIZE - 1);
        if (next != bt_rx_tail) {
            bt_rx_buf[bt_rx_head] = ch;
            bt_rx_head = next;
        }
    }
}

/* UART4 ISR - CPS/Accessory RX */

void UART4_IRQHandler(void)
{
    uint32_t sr = UART4->SR;

    /* Overrun/framing/noise/parity errors are cleared by reading SR then DR.
     * An ISR that tests only RXNE returns without touching DR, so the flag
     * stays set, the interrupt re-fires immediately, and the CPU never leaves
     * the handler -- an interrupt storm that starves everything else until the
     * watchdog resets the radio. This is easy to hit here: the bootloader
     * drives this UART hard during an upload, so ORE is usually already set by
     * the time the application enables the interrupt. */
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        (void)UART4->DR;            /* SR was read above; this clears them */
        return;
    }

    if (sr & USART_SR_RXNE) {
        uint8_t ch = (uint8_t)(UART4->DR & 0xFF);

        /* Watch for the host's "enter update mode" handshake before buffering.
         * This has to happen here, in the ISR, rather than anywhere that
         * depends on the scheduler or on CPS draining the ring buffer -- the
         * entire point is to be able to reflash a radio whose firmware is
         * broken. */
        update_listener_feed(ch);

        uint16_t next = (cps_rx_head + 1) & (UART_CPS_BUF_SIZE - 1);
        if (next != cps_rx_tail) {
            cps_rx_buf[cps_rx_head] = ch;
            cps_rx_head = next;
        }
    }
}

/* uart_gps_init - USART3 at 9600 baud on PB10 (TX) / PB11 (RX)
 *
 * OEM V0.27 references:
 *   usart3_init      @ 0x08013B20 (130 bytes)
 *   BRR = 6250       @ 0x08013B86 (9600 baud from 60 MHz APB1)
 *   PB10 AF_PP       @ 0x08013B44 (gpio_af_config)
 *   PB11 IPU         @ 0x08013B44 (gpio_af_config)
 *   USART3 base      : 0x40004800
 *
 * ISR handler is USART3_IRQHandler in gps.c.
 * NOTE: OEM uses polled I/O (ISR vectors alias into CRC-16 code).
 * Our interrupt-driven approach is an upgrade.
 */

void uart_gps_init(void)
{
    CRM->APB1EN |= CRM_APB1EN_USART3EN;
    CRM->APB2EN |= CRM_APB2EN_IOPBEN;

    gpio_config_pin(GPS_TX_PORT, GPS_TX_PIN, GPIO_MODE_OUT_2MHZ, GPIO_CNF_AF_PP);
    gpio_config_pin(GPS_RX_PORT, GPS_RX_PIN, GPIO_MODE_INPUT, GPIO_CNF_FLOATING);

    USART3->BRR = 6250;
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    NVIC_SetPriority(USART3_IRQn, 1);
    NVIC_EnableIRQ(USART3_IRQn);
}

/* uart_bt_init - USART1 at 115200 baud on PA9 (TX) / PA10 (RX)
 * V0.27 fw 0x08007536: OEM BT USART1 init */

void uart_bt_init(void)
{
    bt_rx_head = 0;
    bt_rx_tail = 0;

    CRM->APB2EN |= CRM_APB2EN_USART1EN;

    gpio_config_pin(BT_TX_PORT, BT_TX_PIN, GPIO_MODE_OUT_2MHZ, GPIO_CNF_AF_PP);
    gpio_config_pin(BT_RX_PORT, BT_RX_PIN, GPIO_MODE_INPUT, GPIO_CNF_FLOATING);

    USART1->BRR = 521;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    NVIC_SetPriority(USART1_IRQn, 1);  /* OEM: preempt=1 */
    NVIC_EnableIRQ(USART1_IRQn);
}

/* uart_acc_init - UART4 at 115200 baud on PC10 (TX) / PC11 (RX)
 * V0.27 fw 0x08022762: OEM CPS UART4 init */

void uart_acc_init(void)
{
    cps_rx_head = 0;
    cps_rx_tail = 0;

    CRM->APB1EN |= CRM_APB1EN_UART4EN;
    CRM->APB2EN |= CRM_APB2EN_IOPCEN;

    gpio_config_pin(CPS_TX_PORT, CPS_TX_PIN, GPIO_MODE_OUT_2MHZ, GPIO_CNF_AF_PP);
    gpio_config_pin(CPS_RX_PORT, CPS_RX_PIN, GPIO_MODE_INPUT, GPIO_CNF_FLOATING);

    /* Reset CR2/CR3 before configuring.
     *
     * These were never written anywhere in this codebase, so the peripheral
     * kept whatever stop-bit, clock and flow-control settings the previous
     * owner left behind. That matters here specifically: the bootloader drives
     * UART4 hard during a firmware upload and hands straight over to the
     * application, so its framing configuration was still in force. The result
     * was consistent per-byte corruption -- "tick #" always arrived as the same
     * wrong six bytes while digits and CRLF came through clean, which reads
     * like a baud error but is not one. */
    UART4->CR2 = 0;    /* 1 stop bit, no clock output, no LIN */
    UART4->CR3 = 0;    /* no flow control, no DMA, no smartcard/IrDA */

    UART4->BRR = 521;
    UART4->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    NVIC_SetPriority(UART4_IRQn, 1);   /* OEM: preempt=1 */
    NVIC_EnableIRQ(UART4_IRQn);
}

/* Blocking transmit */

void uart_send_byte(USART_TypeDef *uart, uint8_t data)
{
    while (!(uart->SR & USART_SR_TXE))
        ;
    uart->DR = data;
}

void uart_send_buf(USART_TypeDef *uart, const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
        uart_send_byte(uart, buf[i]);
    while (!(uart->SR & USART_SR_TC))
        ;
}

/* BT ring buffer API */

uint16_t uart_bt_rx_available(void)
{
    return (bt_rx_head - bt_rx_tail) & (UART_BT_BUF_SIZE - 1);
}

int16_t uart_bt_rx_read(void)
{
    if (bt_rx_head == bt_rx_tail)
        return -1;
    uint8_t ch = bt_rx_buf[bt_rx_tail];
    bt_rx_tail = (bt_rx_tail + 1) & (UART_BT_BUF_SIZE - 1);
    return ch;
}

uint16_t uart_bt_rx_read_buf(uint8_t *buf, uint16_t len)
{
    uint16_t count = 0;
    while (count < len && bt_rx_head != bt_rx_tail) {
        buf[count++] = bt_rx_buf[bt_rx_tail];
        bt_rx_tail = (bt_rx_tail + 1) & (UART_BT_BUF_SIZE - 1);
    }
    return count;
}

void uart_bt_rx_flush(void)
{
    bt_rx_tail = bt_rx_head;
}

/* CPS ring buffer API */

uint16_t uart_cps_rx_available(void)
{
    return (cps_rx_head - cps_rx_tail) & (UART_CPS_BUF_SIZE - 1);
}

int16_t uart_cps_rx_read(void)
{
    if (cps_rx_head == cps_rx_tail)
        return -1;
    uint8_t ch = cps_rx_buf[cps_rx_tail];
    cps_rx_tail = (cps_rx_tail + 1) & (UART_CPS_BUF_SIZE - 1);
    return ch;
}

uint16_t uart_cps_rx_read_buf(uint8_t *buf, uint16_t len)
{
    uint16_t count = 0;
    while (count < len && cps_rx_head != cps_rx_tail) {
        buf[count++] = cps_rx_buf[cps_rx_tail];
        cps_rx_tail = (cps_rx_tail + 1) & (UART_CPS_BUF_SIZE - 1);
    }
    return count;
}

void uart_cps_rx_flush(void)
{
    cps_rx_tail = cps_rx_head;
}

/* Legacy polling API */

uint8_t uart_recv_byte(USART_TypeDef *uart)
{
    while (!(uart->SR & USART_SR_RXNE))
        ;
    return (uint8_t)(uart->DR & 0xFF);
}

uint8_t uart_data_available(USART_TypeDef *uart)
{
    return (uart->SR & USART_SR_RXNE) ? 1 : 0;
}
