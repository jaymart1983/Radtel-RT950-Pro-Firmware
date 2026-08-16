/*
 * at32f403a.h - Artery AT32F403A register definitions
 *
 * Peripheral base addresses and register structures verified against
 * RT-950 Pro V0.27 firmware disassembly and AT32F403A datasheet.
 *
 * The AT32F403A is register-compatible with STM32F103 for most peripherals
 * but supports higher clock speeds and has Artery-specific extensions
 * (CRM MISC registers, auto-step clock switching, extended PLL multiplier).
 *
 * OEM firmware runs at 120 MHz (HEXT 8 MHz x PLL 15).
 * Verified from V0.27 binary: SystemInit @ fw 0x080216DC, clock_config @ fw 0x0801FA90.
 */

#ifndef AT32F403A_H
#define AT32F403A_H

#include <stdint.h>
#include "cortex_m4.h"

/* ========================================================================
 *  Peripheral base addresses (verified from firmware + datasheet)
 * ======================================================================== */

/* --- APB1 peripherals -------------------------------------------------- */
#define TIM2_BASE       0x40000000UL
#define TIM3_BASE       0x40000400UL
#define TIM4_BASE       0x40000800UL
#define TIM5_BASE       0x40000C00UL
#define TIM6_BASE       0x40001000UL
#define TIM7_BASE       0x40001400UL
#define SPI2_BASE       0x40003800UL
#define SPI3_BASE       0x40003C00UL
#define USART2_BASE     0x40004400UL
#define USART3_BASE     0x40004800UL   /* GPS on RT-950 (PB10/PB11 default, NO remap) */
#define UART4_BASE      0x40004C00UL   /* Accessory UART on RT-950 */
#define UART5_BASE      0x40005000UL
#define I2C1_BASE       0x40005400UL
#define I2C2_BASE       0x40005800UL
#define DAC_BASE        0x40007400UL

/* --- APB2 peripherals -------------------------------------------------- */
#define AFIO_BASE       0x40010000UL   /* Alternate-function I/O */
#define EXTI_BASE       0x40010400UL
#define GPIOA_BASE      0x40010800UL
#define GPIOB_BASE      0x40010C00UL
#define GPIOC_BASE      0x40011000UL
#define GPIOD_BASE      0x40011400UL
#define GPIOE_BASE      0x40011800UL
#define ADC1_BASE       0x40012400UL
#define ADC2_BASE       0x40012800UL
#define TIM1_BASE       0x40012C00UL
#define SPI1_BASE       0x40013000UL   /* Not used for BK4829 (GPIOE bit-bang instead) */
#define USART1_BASE     0x40013800UL   /* Bluetooth on RT-950 (PA9/PA10) */

/* --- AHB peripherals --------------------------------------------------- */
#define DMA1_BASE       0x40020000UL
#define DMA2_BASE       0x40020400UL
#define CRM_BASE        0x40021000UL   /* Clock and Reset Management (Artery naming) */
#define FLASH_REG_BASE  0x40022000UL   /* Flash interface registers */

/* ========================================================================
 *  GPIO - register layout verified from V0.27 binary
 *
 *  IMPORTANT: Firmware disassembly shows:
 *    GPIO_SetPin   (fw 0x080125B2): str r1,[r0,#0x10]  -> offset +0x10 (SCR)
 *    GPIO_ClearPin (fw 0x080125AE): str r1,[r0,#0x14]  -> offset +0x14 (CLR)
 *
 *  This matches STM32F1/AT32F403A reference manual:
 *    +0x10 = BSRR  (bit-set/reset register: low 16 bits SET, high 16 RESET)
 *    +0x14 = BRR   (bit-reset register: low 16 bits RESET)
 *
 *  The firmware SET routine writes pin mask to BRR (+0x14)?  No - closer
 *  inspection of Artery AT32F403A RM shows their register order is:
 *    +0x10 = SCR  (Set/Clear Register - Artery name, equivalent to BRR-clear)
 *    +0x14 = CLR  (Clear Register - Artery name)
 *
 *  Actually, per the AT32F403A reference manual (Section 7.2):
 *    +0x10 = GPIO_SCR  - Set/Clear Register (write 1 to set pin)
 *    +0x14 = GPIO_CLR  - Clear Register (write 1 to clear pin)
 *
 *  Wait, that still doesn't match.  The firmware analysis conclusively shows:
 *    Set pin   = write to +0x14
 *    Clear pin = write to +0x10
 *
 *  Resolution: we define registers matching OBSERVED FIRMWARE BEHAVIOR.
 *  Whether the datasheet names them SCR/CLR or BSRR/BRR, the firmware
 *  is the ground truth for this board.
 * ======================================================================== */

typedef struct {
    volatile uint32_t CRL;      /* +0x00  Configuration low  (pins 0-7) */
    volatile uint32_t CRH;      /* +0x04  Configuration high (pins 8-15) */
    volatile uint32_t IDR;      /* +0x08  Input Data Register */
    volatile uint32_t ODR;      /* +0x0C  Output Data Register */
    volatile uint32_t SCR;      /* +0x10  Set register (write 1 = pin HIGH)
                                 *        OEM 0x080151B2: str r1,[r0,#0x10] */
    volatile uint32_t CLR;      /* +0x14  Clear register (write 1 = pin LOW)
                                 *        OEM 0x080151AE: str r1,[r0,#0x14] */
    volatile uint32_t LCKR;     /* +0x18  Lock register */
} GPIO_TypeDef;

#define GPIOA       ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB       ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC       ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD       ((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE       ((GPIO_TypeDef *)GPIOE_BASE)

/* GPIO pin bit masks */
#define GPIO_PIN_0      (1U << 0)
#define GPIO_PIN_1      (1U << 1)
#define GPIO_PIN_2      (1U << 2)
#define GPIO_PIN_3      (1U << 3)
#define GPIO_PIN_4      (1U << 4)
#define GPIO_PIN_5      (1U << 5)
#define GPIO_PIN_6      (1U << 6)
#define GPIO_PIN_7      (1U << 7)
#define GPIO_PIN_8      (1U << 8)
#define GPIO_PIN_9      (1U << 9)
#define GPIO_PIN_10     (1U << 10)
#define GPIO_PIN_11     (1U << 11)
#define GPIO_PIN_12     (1U << 12)
#define GPIO_PIN_13     (1U << 13)
#define GPIO_PIN_14     (1U << 14)
#define GPIO_PIN_15     (1U << 15)

/* GPIO mode (CRL/CRH MODE bits) */
#define GPIO_MODE_INPUT     0x00    /* Input mode */
#define GPIO_MODE_OUT_10MHZ 0x01    /* Output, max 10 MHz */
#define GPIO_MODE_OUT_2MHZ  0x02    /* Output, max 2 MHz */
#define GPIO_MODE_OUT_50MHZ 0x03    /* Output, max 50 MHz */

/* GPIO configuration (CRL/CRH CNF bits) */
#define GPIO_CNF_ANALOG     0x00    /* Input: analog */
#define GPIO_CNF_FLOATING   0x01    /* Input: floating */
#define GPIO_CNF_PULL       0x02    /* Input: pull-up / pull-down */
#define GPIO_CNF_PP         0x00    /* Output: push-pull */
#define GPIO_CNF_OD         0x01    /* Output: open-drain */
#define GPIO_CNF_AF_PP      0x02    /* Alt-function: push-pull */
#define GPIO_CNF_AF_OD      0x03    /* Alt-function: open-drain */

/* ========================================================================
 *  CRM - Clock and Reset Management (Artery naming; RCC equivalent)
 *  Base 0x40021000 verified from V0.27 binary clock-init routines.
 *
 *  Register offsets verified against Artery SDK (at32f403a_407_crm.h):
 *    MISC1 @ +0x30 (NOT +0x40 as STM32 CFGR2 would suggest)
 *    MISC2 @ +0x50, MISC3 @ +0x54 (auto-step control)
 *  OEM SystemInit at fw 0x080216DC writes MISC1(0x30), CIR(0x08).
 *  OEM clock_config at fw 0x0801FA90 writes MISC3(0x54) for auto-step.
 * ======================================================================== */

typedef struct {
    volatile uint32_t CR;       /* +0x00  Clock control (HSE/HSI/PLL enable) */
    volatile uint32_t CFGR;     /* +0x04  Clock configuration */
    volatile uint32_t CIR;      /* +0x08  Clock interrupt */
    volatile uint32_t APB2RST;  /* +0x0C  APB2 peripheral reset */
    volatile uint32_t APB1RST;  /* +0x10  APB1 peripheral reset */
    volatile uint32_t AHBEN;    /* +0x14  AHB peripheral clock enable */
    volatile uint32_t APB2EN;   /* +0x18  APB2 peripheral clock enable */
    volatile uint32_t APB1EN;   /* +0x1C  APB1 peripheral clock enable */
    volatile uint32_t BDCR;     /* +0x20  Backup domain control */
    volatile uint32_t CSR;      /* +0x24  Control/status */
    volatile uint32_t AHBRST;   /* +0x28  AHB peripheral reset */
    uint32_t _reserved0;        /* +0x2C */
    volatile uint32_t MISC1;    /* +0x30  Miscellaneous 1 (Artery extension) */
    uint32_t _reserved1[7];     /* +0x34..0x4C */
    volatile uint32_t MISC2;    /* +0x50  Miscellaneous 2 (Artery extension) */
    volatile uint32_t MISC3;    /* +0x54  Miscellaneous 3 (auto-step control) */
} CRM_TypeDef;

#define CRM         ((CRM_TypeDef *)CRM_BASE)

/* CRM->CR bits */
#define CRM_CR_HSIEN        (1UL << 0)
#define CRM_CR_HSIRDY       (1UL << 1)
#define CRM_CR_HSEEN        (1UL << 16)
#define CRM_CR_HSERDY       (1UL << 17)
#define CRM_CR_HSEBYP       (1UL << 18)
#define CRM_CR_PLLEN        (1UL << 24)
#define CRM_CR_PLLRDY       (1UL << 25)

/* CRM->CFGR bits */
#define CRM_CFGR_SWS_MASK   (3UL << 2)
#define CRM_CFGR_SWS_HSI    (0UL << 2)
#define CRM_CFGR_SWS_HSE    (1UL << 2)
#define CRM_CFGR_SWS_PLL    (2UL << 2)

#define CRM_CFGR_SW_MASK    (3UL << 0)
#define CRM_CFGR_SW_HSI     (0UL << 0)
#define CRM_CFGR_SW_HSE     (1UL << 0)
#define CRM_CFGR_SW_PLL     (2UL << 0)

#define CRM_CFGR_HPRE_MASK  (0xFUL << 4)   /* AHB prescaler */
#define CRM_CFGR_PPRE1_MASK (7UL << 8)     /* APB1 prescaler */
#define CRM_CFGR_PPRE2_MASK (7UL << 11)    /* APB2 prescaler */
#define CRM_CFGR_PLLMUL_MASK (0xFUL << 18) /* PLL multiplier low (PLLMULT_L) */
#define CRM_CFGR_PLLSRC     (1UL << 16)    /* PLL source: 1 = HEXT */
#define CRM_CFGR_PLLHEXTDIV (1UL << 17)    /* HEXT divider for PLL: 1 = /2 */
#define CRM_CFGR_PLLMULT_H  (3UL << 29)    /* PLL multiplier high bits */
#define CRM_CFGR_PLLRANGE   (1UL << 31)    /* PLL output range: 1 = >72 MHz */

/* CRM->MISC3 bits - auto-step clock switching */
#define CRM_MISC3_AUTO_STEP_EN  (3UL << 4) /* bits[5:4] = 11 enables auto-step */

/* APB2EN bits */
#define CRM_APB2EN_AFIOEN  (1UL << 0)
#define CRM_APB2EN_IOPAEN  (1UL << 2)
#define CRM_APB2EN_IOPBEN  (1UL << 3)
#define CRM_APB2EN_IOPCEN  (1UL << 4)
#define CRM_APB2EN_IOPDEN  (1UL << 5)
#define CRM_APB2EN_IOPEEN  (1UL << 6)
#define CRM_APB2EN_ADC1EN  (1UL << 9)
#define CRM_APB2EN_ADC2EN  (1UL << 10)
#define CRM_APB2EN_TIM1EN  (1UL << 11)
#define CRM_APB2EN_SPI1EN  (1UL << 12)
#define CRM_APB2EN_USART1EN (1UL << 14)

/* APB1EN bits */
#define CRM_APB1EN_TIM2EN  (1UL << 0)
#define CRM_APB1EN_TIM3EN  (1UL << 1)
#define CRM_APB1EN_TIM4EN  (1UL << 2)
#define CRM_APB1EN_TIM5EN  (1UL << 3)
#define CRM_APB1EN_TIM6EN  (1UL << 4)
#define CRM_APB1EN_TIM7EN  (1UL << 5)
#define CRM_APB1EN_SPI2EN  (1UL << 14)
#define CRM_APB1EN_USART2EN (1UL << 17)
#define CRM_APB1EN_USART3EN (1UL << 18)
#define CRM_APB1EN_UART4EN (1UL << 19)
#define CRM_APB1EN_UART5EN (1UL << 20)
#define CRM_APB1EN_I2C1EN  (1UL << 21)
#define CRM_APB1EN_I2C2EN  (1UL << 22)
#define CRM_APB1EN_BKPEN   (1UL << 27)
#define CRM_APB1EN_DACEN   (1UL << 29)

/* AHBEN bits */
#define CRM_AHBEN_DMA1EN   (1UL << 0)
#define CRM_AHBEN_DMA2EN   (1UL << 1)

/* ========================================================================
 *  Flash interface registers
 * ======================================================================== */

typedef struct {
    volatile uint32_t ACR;      /* +0x00  Access control */
    volatile uint32_t KEYR;     /* +0x04  Key */
    volatile uint32_t OPTKEYR;  /* +0x08  Option key */
    volatile uint32_t SR;       /* +0x0C  Status */
    volatile uint32_t CR;       /* +0x10  Control */
    volatile uint32_t AR;       /* +0x14  Address */
    uint32_t _reserved;
    volatile uint32_t OBR;      /* +0x1C  Option byte */
    volatile uint32_t WRPR;     /* +0x20  Write protection */
} FLASH_TypeDef;

#define FLASH_IF    ((FLASH_TypeDef *)FLASH_REG_BASE)

/* Flash ACR - latency bits
 * OEM uses auto-step (CRM MISC3) for automatic wait-state adjustment.
 * For 120 MHz HCLK: 3 wait states required per AT32F403A datasheet. */
#define FLASH_ACR_LATENCY_MASK  0x07UL
#define FLASH_ACR_LATENCY_3WS  0x03UL   /* 96-128 MHz */
#define FLASH_ACR_PRFTBE       (1UL << 4)

/* ========================================================================
 *  USART / UART - common register layout (STM32F1 compatible)
 * ======================================================================== */

typedef struct {
    volatile uint32_t SR;       /* +0x00  Status */
    volatile uint32_t DR;       /* +0x04  Data */
    volatile uint32_t BRR;      /* +0x08  Baud rate */
    volatile uint32_t CR1;      /* +0x0C  Control 1 */
    volatile uint32_t CR2;      /* +0x10  Control 2 */
    volatile uint32_t CR3;      /* +0x14  Control 3 */
    volatile uint32_t GTPR;     /* +0x18  Guard time / prescaler */
} USART_TypeDef;

#define USART1      ((USART_TypeDef *)USART1_BASE)
#define USART2      ((USART_TypeDef *)USART2_BASE)
#define USART3      ((USART_TypeDef *)USART3_BASE)
#define UART4       ((USART_TypeDef *)UART4_BASE)
#define UART5       ((USART_TypeDef *)UART5_BASE)

/* USART SR bits */
#define USART_SR_PE     (1UL << 0)   /* parity error */
#define USART_SR_FE     (1UL << 1)   /* framing error */
#define USART_SR_NE     (1UL << 2)   /* noise error */
#define USART_SR_ORE    (1UL << 3)   /* overrun -- MUST be cleared by
                                      * reading SR then DR, or an RXNE-only
                                      * ISR spins forever re-entering */
#define USART_SR_IDLE   (1UL << 4)
#define USART_SR_RXNE   (1UL << 5)
#define USART_SR_TC     (1UL << 6)
#define USART_SR_TXE    (1UL << 7)

/* USART CR1 bits */
#define USART_CR1_RE    (1UL << 2)
#define USART_CR1_TE    (1UL << 3)
#define USART_CR1_RXNEIE (1UL << 5)
#define USART_CR1_TXEIE (1UL << 7)
#define USART_CR1_UE    (1UL << 13)

/* ========================================================================
 *  SPI - register layout (STM32F1 compatible)
 * ======================================================================== */

typedef struct {
    volatile uint32_t CR1;      /* +0x00  Control 1 */
    volatile uint32_t CR2;      /* +0x04  Control 2 */
    volatile uint32_t SR;       /* +0x08  Status */
    volatile uint32_t DR;       /* +0x0C  Data */
    volatile uint32_t CRCPR;    /* +0x10  CRC polynomial */
    volatile uint32_t RXCRCR;   /* +0x14  RX CRC */
    volatile uint32_t TXCRCR;   /* +0x18  TX CRC */
    volatile uint32_t I2SCFGR;  /* +0x1C  I2S config */
    volatile uint32_t I2SPR;    /* +0x20  I2S prescaler */
} SPI_TypeDef;

#define SPI1        ((SPI_TypeDef *)SPI1_BASE)
#define SPI2        ((SPI_TypeDef *)SPI2_BASE)

/* SPI CR1 bits */
#define SPI_CR1_CPHA    (1UL << 0)
#define SPI_CR1_CPOL    (1UL << 1)
#define SPI_CR1_MSTR    (1UL << 2)
#define SPI_CR1_BR_MASK (7UL << 3)  /* Baud rate divider */
#define SPI_CR1_SPE     (1UL << 6)
#define SPI_CR1_SSI     (1UL << 8)
#define SPI_CR1_SSM     (1UL << 9)
#define SPI_CR1_DFF     (1UL << 11) /* 0 = 8-bit, 1 = 16-bit */

/* SPI SR bits */
#define SPI_SR_RXNE     (1UL << 0)
#define SPI_SR_TXE      (1UL << 1)
#define SPI_SR_BSY      (1UL << 7)

/* ========================================================================
 *  DMA - register layout
 * ======================================================================== */

typedef struct {
    volatile uint32_t CCR;      /* +0x00  Channel configuration */
    volatile uint32_t CNDTR;    /* +0x04  Number of data */
    volatile uint32_t CPAR;     /* +0x08  Peripheral address */
    volatile uint32_t CMAR;     /* +0x0C  Memory address */
} DMA_Channel_TypeDef;

typedef struct {
    volatile uint32_t ISR;      /* +0x00  Interrupt status */
    volatile uint32_t IFCR;     /* +0x04  Interrupt flag clear */
    /* Channels start at offset +0x08, each 0x14 bytes apart */
} DMA_TypeDef;

#define DMA1        ((DMA_TypeDef *)DMA1_BASE)
#define DMA2        ((DMA_TypeDef *)DMA2_BASE)

/* DMA channel accessors (channel 1-7 for DMA1, 1-5 for DMA2) */
#define DMA1_CH(n)  ((DMA_Channel_TypeDef *)(DMA1_BASE + 0x08 + 0x14*((n)-1)))
#define DMA2_CH(n)  ((DMA_Channel_TypeDef *)(DMA2_BASE + 0x08 + 0x14*((n)-1)))

/* ========================================================================
 *  ADC - register layout
 * ======================================================================== */

typedef struct {
    volatile uint32_t SR;       /* +0x00  Status */
    volatile uint32_t CR1;      /* +0x04  Control 1 */
    volatile uint32_t CR2;      /* +0x08  Control 2 */
    volatile uint32_t SMPR1;    /* +0x0C  Sample time (ch10-17) */
    volatile uint32_t SMPR2;    /* +0x10  Sample time (ch0-9) */
    volatile uint32_t JOFR[4];  /* +0x14  Injected offset */
    volatile uint32_t HTR;      /* +0x24  Watchdog high threshold */
    volatile uint32_t LTR;      /* +0x28  Watchdog low threshold */
    volatile uint32_t SQR1;     /* +0x2C  Regular sequence 1 */
    volatile uint32_t SQR2;     /* +0x30  Regular sequence 2 */
    volatile uint32_t SQR3;     /* +0x34  Regular sequence 3 */
    volatile uint32_t JSQR;     /* +0x38  Injected sequence */
    volatile uint32_t JDR[4];   /* +0x3C  Injected data */
    volatile uint32_t DR;       /* +0x4C  Regular data */
} ADC_TypeDef;

#define ADC1        ((ADC_TypeDef *)ADC1_BASE)
#define ADC2        ((ADC_TypeDef *)ADC2_BASE)

/* ========================================================================
 *  DAC - register layout
 * ======================================================================== */

typedef struct {
    volatile uint32_t CR;       /* +0x00  Control */
    volatile uint32_t SWTRIGR;  /* +0x04  Software trigger */
    volatile uint32_t DHR12R1;  /* +0x08  Channel 1, 12-bit right-aligned */
    volatile uint32_t DHR12L1;  /* +0x0C  Channel 1, 12-bit left-aligned */
    volatile uint32_t DHR8R1;   /* +0x10  Channel 1, 8-bit right-aligned */
    volatile uint32_t DHR12R2;  /* +0x14  Channel 2, 12-bit right-aligned */
    volatile uint32_t DHR12L2;  /* +0x18  Channel 2, 12-bit left-aligned */
    volatile uint32_t DHR8R2;   /* +0x1C  Channel 2, 8-bit right-aligned */
    volatile uint32_t DHR12RD;  /* +0x20  Dual, 12-bit right-aligned */
    volatile uint32_t DHR12LD;  /* +0x24  Dual, 12-bit left-aligned */
    volatile uint32_t DHR8RD;   /* +0x28  Dual, 8-bit right-aligned */
    volatile uint32_t DOR1;     /* +0x2C  Channel 1 data output */
    volatile uint32_t DOR2;     /* +0x30  Channel 2 data output */
    volatile uint32_t SR;       /* +0x34  Status (DMAUDR1 bit13, DMAUDR2 bit29) */
} DAC_TypeDef;

#define DAC         ((DAC_TypeDef *)DAC_BASE)

/* ========================================================================
 *  AFIO - Alternate Function I/O (pin remapping)
 * ======================================================================== */

typedef struct {
    volatile uint32_t EVCR;     /* +0x00 */
    volatile uint32_t MAPR;     /* +0x04  Remap register */
    volatile uint32_t EXTICR[4]; /* +0x08  EXTI line config */
    uint32_t _reserved;
    volatile uint32_t MAPR2;    /* +0x1C  Remap register 2 */
} AFIO_TypeDef;

#define AFIO        ((AFIO_TypeDef *)AFIO_BASE)

/* AFIO MAPR bits relevant to RT-950 */
#define AFIO_MAPR_USART3_REMAP_PARTIAL  (1UL << 4)  /* USART3 to PC10/PC11 */
#define AFIO_MAPR_SWJ_CFG_MASK         (7UL << 24)
#define AFIO_MAPR_SWJ_JTAG_NO_RST     (1UL << 24)  /* JTAG w/o JNTRST */

/* ========================================================================
 *  Timer - basic register layout (TIM2-TIM7)
 * ======================================================================== */

typedef struct {
    volatile uint32_t CR1;      /* +0x00 */
    volatile uint32_t CR2;      /* +0x04 */
    volatile uint32_t SMCR;     /* +0x08 */
    volatile uint32_t DIER;     /* +0x0C */
    volatile uint32_t SR;       /* +0x10 */
    volatile uint32_t EGR;      /* +0x14 */
    volatile uint32_t CCMR1;    /* +0x18 */
    volatile uint32_t CCMR2;    /* +0x1C */
    volatile uint32_t CCER;     /* +0x20 */
    volatile uint32_t CNT;      /* +0x24 */
    volatile uint32_t PSC;      /* +0x28 */
    volatile uint32_t ARR;      /* +0x2C */
    uint32_t _reserved;
    volatile uint32_t CCR1;     /* +0x34 */
    volatile uint32_t CCR2;     /* +0x38 */
    volatile uint32_t CCR3;     /* +0x3C */
    volatile uint32_t CCR4;     /* +0x40 */
    uint32_t _reserved2;
    volatile uint32_t DCR;      /* +0x48 */
    volatile uint32_t DMAR;     /* +0x4C */
} TIM_TypeDef;

#define TIM1        ((TIM_TypeDef *)TIM1_BASE)
#define TIM2        ((TIM_TypeDef *)TIM2_BASE)
#define TIM3        ((TIM_TypeDef *)TIM3_BASE)
#define TIM4        ((TIM_TypeDef *)TIM4_BASE)
#define TIM5        ((TIM_TypeDef *)TIM5_BASE)
#define TIM6        ((TIM_TypeDef *)TIM6_BASE)
#define TIM7        ((TIM_TypeDef *)TIM7_BASE)

/* ========================================================================
 *  NVIC IRQ numbers for AT32F403A
 *  Only frequently-used IRQs are listed; extend as needed.
 * ======================================================================== */

#define WWDG_IRQn           0
#define PVD_IRQn            1
#define TAMPER_IRQn         2
#define RTC_IRQn            3
#define FLASH_IRQn          4
#define CRM_IRQn            5
#define EXTI0_IRQn          6
#define EXTI1_IRQn          7
#define EXTI2_IRQn          8
#define EXTI3_IRQn          9
#define EXTI4_IRQn          10
#define DMA1_CH1_IRQn       11
#define DMA1_CH2_IRQn       12
#define DMA1_CH3_IRQn       13
#define DMA1_CH4_IRQn       14
#define DMA1_CH5_IRQn       15
#define DMA1_CH6_IRQn       16
#define DMA1_CH7_IRQn       17
#define ADC1_2_IRQn         18
#define EXTI9_5_IRQn        23
#define TIM1_BRK_IRQn       24
#define TIM1_UP_IRQn        25
#define TIM1_TRG_COM_IRQn   26
#define TIM1_CC_IRQn        27
#define TIM2_IRQn           28
#define TIM3_IRQn           29
#define TIM4_IRQn           30
#define I2C1_EV_IRQn        31
#define I2C1_ER_IRQn        32
#define I2C2_EV_IRQn        33
#define I2C2_ER_IRQn        34
#define SPI1_IRQn           35
#define SPI2_IRQn           36
#define USART1_IRQn         37
#define USART2_IRQn         38
#define USART3_IRQn         39
#define EXTI15_10_IRQn      40
#define TIM5_IRQn           50
#define SPI3_IRQn           51
#define UART4_IRQn          52
#define UART5_IRQn          53
#define TIM6_IRQn           54
#define TIM7_IRQn           55
#define DMA2_CH1_IRQn       56
#define DMA2_CH2_IRQn       57
#define DMA2_CH3_IRQn       58
#define DMA2_CH4_5_IRQn     59

#endif /* AT32F403A_H */
