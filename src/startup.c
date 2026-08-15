/*
 * startup.c - Vector table and Reset_Handler for the RT-950 Pro
 *
 * Target: Artery AT32F403A (ARM Cortex-M4F @ 120 MHz)
 *
 * Vector table placed at 0x08003000 (after 12 KB OEM bootloader).
 * The OEM bootloader reads [0x08003000] as initial SP and [0x08003004]
 * as Reset_Handler, validates (SP & 0x2FFE0000 == 0x20000000), then
 * sets MSP and jumps.
 * See also: docs/bootloader.md for full bootloader reverse engineering.
 *
 * V0.27 OEM vector table (offset 0x0000, flash 0x08000000):
 *   Initial SP     = 0x20015D10 (OEM layout; we use _estack from linker)
 *   Reset_Handler  = 0x080032A1 (bootloader entry)
 *   SVCall         = 0x0801F0A7
 *   PendSV         = 0x0801BD11
 *   SysTick        = 0x080241E1 (uses double-precision FPU)
 *   HardFault      = 0x08016BC9
 *   Default handler= 0x080032BB (76 peripheral IRQs, b . loop)
 *
 * Active peripheral ISRs (only 2 of 76 are non-default):
 *   IRQ11 DMA1_CH1  @ 0x0800D191 - command dispatcher (switch on [r0+4])
 *     Cases: 0x08, 0x10-0x13, 0x15, 0x17-0x18, 0x69. RAM @ 0x2000A360.
 *   IRQ58 DMA2_CH3  @ 0x0800D1D5 - UART RX DMA completion
 *     Handles cmd 0x69, calls fw 0x0800B6D8 (buffer proc) + fw 0x080070C0
 *
 * USART1/2/3/UART4 vector entries (0x08024E95-0x08024EC3) all alias into
 * CRC-16 function body @ fw 0x08024E52 (poly 0x8408, reflected CRC-CCITT).
 * OEM UART I/O is polling-based; our implementation upgrades to IRQ-driven.
 *
 * VTOR relocation: write to 0xE000ED08 @ fw 0x0801886C
 *   Uses alignment mask 0x1FFFFF80 from literal pool @ fw 0x08018878
 *
 * ARM C runtime: __scatterload @ fw 0x0800017C, __rt_entry @ fw 0x08000280.
 * See also: tools/firmware_upload.py for BTF update protocol implementation.
 */

#include <stdint.h>
#include "debug_uart.h"

/* Symbols defined by linker.ld */
extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

/* Forward declarations */
void Reset_Handler(void);
void Default_Handler(void);
void SystemInit(void);
int  main(void);

/* Cortex-M4 system exception handlers - weak, overridable */
void NMI_Handler(void)         __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)   __attribute__((weak));
void MemManage_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)    __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)         __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)    __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)     __attribute__((weak, alias("Default_Handler")));

/* AT32F403A peripheral ISR handlers - weak, overridable */
void WWDG_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void PVD_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void TAMPER_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void RTC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void CRM_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void EXTI2_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void EXTI3_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void EXTI4_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void DMA1_CH1_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_CH2_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_CH3_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_CH4_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_CH5_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_CH6_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA1_CH7_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void ADC1_2_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_IRQHandler(void)  __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void TIM4_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void SPI2_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void USART1_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void USART2_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void USART3_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));
void RTCAlarm_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void TIM5_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void SPI3_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void UART4_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void UART5_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void TIM6_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void TIM7_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void DMA2_CH1_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA2_CH2_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA2_CH3_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void DMA2_CH4_5_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));
void SDIO2_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void SPI4_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void CAN2_TX_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void CAN2_RX0_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void CAN2_RX1_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void CAN2_SCE_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void ACC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void USB_HP2_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void USB_LP2_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void DMA2_CH6_7_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));
void DMA2_CH4_5_OVR_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

/* ========================================================================
 *  Vector table - placed at 0x08003000 by linker (.isr_vector section)
 *
 *  The AT32F403A has 76 peripheral interrupts (IRQ 0-75).
 *  We define 16 system + 76 peripheral = 92 entries to cover all IRQs.
 *
 *  V0.27 binary active peripheral ISRs (all others use default handler):
 *    IRQ11 DMA1_CH1  @ fw 0x0800D191  (command dispatcher / audio DMA)
 *    IRQ58 DMA2_CH3  @ fw 0x0800D1D5  (UART RX DMA completion)
 * ======================================================================== */

__attribute__((section(".isr_vector"), used))
const void *vector_table[] = {
    /* System exceptions */
    &_estack,                   /* [0]  Initial stack pointer (_estack from linker) */
    Reset_Handler,              /* [1]  Reset */
    NMI_Handler,                /* [2]  NMI */
    HardFault_Handler,          /* [3]  Hard Fault */
    MemManage_Handler,          /* [4]  Memory Management */
    BusFault_Handler,           /* [5]  Bus Fault */
    UsageFault_Handler,         /* [6]  Usage Fault */
    0, 0, 0, 0,                 /* [7-10] Reserved */
    SVC_Handler,                /* [11] SVCall */
    DebugMon_Handler,           /* [12] Debug Monitor */
    0,                          /* [13] Reserved */
    PendSV_Handler,             /* [14] PendSV */
    SysTick_Handler,            /* [15] SysTick */

    /* AT32F403A peripheral interrupts (IRQ 0-75) */
    WWDG_IRQHandler,            /* [16] IRQ0   Window Watchdog */
    PVD_IRQHandler,             /* [17] IRQ1   Power Voltage Detector */
    TAMPER_IRQHandler,          /* [18] IRQ2   Tamper */
    RTC_IRQHandler,             /* [19] IRQ3   RTC global */
    FLASH_IRQHandler,           /* [20] IRQ4   Flash */
    CRM_IRQHandler,             /* [21] IRQ5   CRM (Clock and Reset) */
    EXTI0_IRQHandler,           /* [22] IRQ6   EXTI Line 0 */
    EXTI1_IRQHandler,           /* [23] IRQ7   EXTI Line 1 */
    EXTI2_IRQHandler,           /* [24] IRQ8   EXTI Line 2 */
    EXTI3_IRQHandler,           /* [25] IRQ9   EXTI Line 3 */
    EXTI4_IRQHandler,           /* [26] IRQ10  EXTI Line 4 */
    DMA1_CH1_IRQHandler,        /* [27] IRQ11  DMA1 Channel 1 - OEM ACTIVE */
    DMA1_CH2_IRQHandler,        /* [28] IRQ12  DMA1 Channel 2 */
    DMA1_CH3_IRQHandler,        /* [29] IRQ13  DMA1 Channel 3 */
    DMA1_CH4_IRQHandler,        /* [30] IRQ14  DMA1 Channel 4 */
    DMA1_CH5_IRQHandler,        /* [31] IRQ15  DMA1 Channel 5 */
    DMA1_CH6_IRQHandler,        /* [32] IRQ16  DMA1 Channel 6 */
    DMA1_CH7_IRQHandler,        /* [33] IRQ17  DMA1 Channel 7 */
    ADC1_2_IRQHandler,          /* [34] IRQ18  ADC1 and ADC2 */
    0, 0, 0, 0,                 /* [35-38] IRQ19-22 USB/CAN (unused) */
    EXTI9_5_IRQHandler,         /* [39] IRQ23  EXTI Lines 5-9 */
    TIM1_BRK_IRQHandler,        /* [40] IRQ24  TIM1 Break */
    TIM1_UP_IRQHandler,         /* [41] IRQ25  TIM1 Update */
    TIM1_TRG_COM_IRQHandler,    /* [42] IRQ26  TIM1 Trigger/Commutation */
    TIM1_CC_IRQHandler,         /* [43] IRQ27  TIM1 Capture/Compare */
    TIM2_IRQHandler,            /* [44] IRQ28  TIM2 */
    TIM3_IRQHandler,            /* [45] IRQ29  TIM3 */
    TIM4_IRQHandler,            /* [46] IRQ30  TIM4 */
    I2C1_EV_IRQHandler,         /* [47] IRQ31  I2C1 Event */
    I2C1_ER_IRQHandler,         /* [48] IRQ32  I2C1 Error */
    I2C2_EV_IRQHandler,         /* [49] IRQ33  I2C2 Event */
    I2C2_ER_IRQHandler,         /* [50] IRQ34  I2C2 Error */
    SPI1_IRQHandler,            /* [51] IRQ35  SPI1 */
    SPI2_IRQHandler,            /* [52] IRQ36  SPI2 */
    USART1_IRQHandler,          /* [53] IRQ37  USART1 */
    USART2_IRQHandler,          /* [54] IRQ38  USART2 */
    USART3_IRQHandler,          /* [55] IRQ39  USART3 */
    EXTI15_10_IRQHandler,       /* [56] IRQ40  EXTI Lines 10-15 */
    RTCAlarm_IRQHandler,        /* [57] IRQ41  RTC Alarm via EXTI */
    0,                          /* [58] IRQ42  USB Wakeup */
    0, 0, 0, 0,                 /* [59-62] IRQ43-46 TIM8 variants */
    0, 0, 0,                    /* [63-65] IRQ47-49 ADC3/XMC/SDIO1 */
    TIM5_IRQHandler,            /* [66] IRQ50  TIM5 */
    SPI3_IRQHandler,            /* [67] IRQ51  SPI3 */
    UART4_IRQHandler,           /* [68] IRQ52  UART4 */
    UART5_IRQHandler,           /* [69] IRQ53  UART5 */
    TIM6_IRQHandler,            /* [70] IRQ54  TIM6 */
    TIM7_IRQHandler,            /* [71] IRQ55  TIM7 */
    DMA2_CH1_IRQHandler,        /* [72] IRQ56  DMA2 Channel 1 */
    DMA2_CH2_IRQHandler,        /* [73] IRQ57  DMA2 Channel 2 */
    DMA2_CH3_IRQHandler,        /* [74] IRQ58  DMA2 Channel 3 - OEM ACTIVE */
    DMA2_CH4_5_IRQHandler,      /* [75] IRQ59  DMA2 Channel 4-5 */
    SDIO2_IRQHandler,           /* [76] IRQ60  SDIO2 */
    I2C3_EV_IRQHandler,         /* [77] IRQ61  I2C3 Event */
    I2C3_ER_IRQHandler,         /* [78] IRQ62  I2C3 Error */
    SPI4_IRQHandler,            /* [79] IRQ63  SPI4 */
    0, 0, 0,                    /* [80-82] IRQ64-66 Reserved */
    CAN2_TX_IRQHandler,         /* [83] IRQ67  CAN2 TX */
    CAN2_RX0_IRQHandler,        /* [84] IRQ68  CAN2 RX0 */
    CAN2_RX1_IRQHandler,        /* [85] IRQ69  CAN2 RX1 */
    CAN2_SCE_IRQHandler,        /* [86] IRQ70  CAN2 SCE */
    ACC_IRQHandler,             /* [87] IRQ71  ACC */
    USB_HP2_IRQHandler,         /* [88] IRQ72  USB_HP2 */
    USB_LP2_IRQHandler,         /* [89] IRQ73  USB_LP2 */
    DMA2_CH6_7_IRQHandler,      /* [90] IRQ74  DMA2 Channel 6-7 */
    DMA2_CH4_5_OVR_IRQHandler,  /* [91] IRQ75  DMA2 Channel 4-5 overflow */
};

/* ========================================================================
 *  Model signature for bootloader firmware verification (cmd 0x02).
 *  Placed at binary offset 0x3E0 (flash 0x080033E0) by linker.
 *  The bootloader loads 16 bytes from its own 0x080003A4 and compares
 *  the first 12 bytes (strlen-based) against this signature.
 * ======================================================================== */

__attribute__((section(".model_sig"), used))
const uint8_t btf_model_signature[16] = {
    'R', 'T', '-', '9', '5', '0', ' ', ' ',
    ' ', ' ', ' ', ' ', 0x00, 0x00, 0x00, 0x00
};

/* Boot-time model signature at binary offset 0x7E0.
 * The bootloader's check_spi_model() reads from flash 0x080037E0
 * after upload (key block is not written to flash, shifting offsets).
 * Must match "RT-950      " or the bootloader erases firmware. */
__attribute__((section(".boot_model_sig"), used))
const uint8_t boot_model_signature[16] = {
    'R', 'T', '-', '9', '5', '0', ' ', ' ',
    ' ', ' ', ' ', ' ', 0x00, 0x00, 0x00, 0x00
};

/* ========================================================================
 *  Reset_Handler - C runtime initialization
 *
 *  1. Copy .data section from flash (LMA) to SRAM (VMA)
 *  2. Zero the .bss section
 *  3. Enable FPU (Cortex-M4F)
 *  4. Call SystemInit() for clock configuration
 *  5. Call main()
 *  6. Infinite loop if main() returns
 * ======================================================================== */

void Reset_Handler(void)
{
    uint32_t *src, *dst;

#ifdef DEBUG_UART
    /* Ultra-early debug: init UART4 with raw register writes.
     * No .data/.bss dependency - pure hardware register setup.
     * Sends a marker byte so we know the CPU reached Reset_Handler. */
    *(volatile uint32_t *)0x40021018 |= (1UL << 19); /* CRM APB1EN: UART4EN */
    *(volatile uint32_t *)0x40021018;                 /* read-back fence */
    *(volatile uint32_t *)0x40021014 |= (1UL << 4);  /* CRM APB2EN: IOPCEN */
    *(volatile uint32_t *)0x40021014;                 /* read-back fence */
    /* PC10 = AF push-pull 2MHz: GPIOC CRH bits[11:8] = 0xA */
    {
        volatile uint32_t *crh = (volatile uint32_t *)0x40011004;
        uint32_t v = *crh;
        v &= ~(0xFUL << 8);
        v |=  (0xAUL << 8);
        *crh = v;
    }
    *(volatile uint32_t *)0x40004C08 = 521;           /* UART4 BRR: 115200 */
    *(volatile uint32_t *)0x40004C0C = (1UL<<3)|(1UL<<13); /* CR1: TE+UE */
    /* Send "R\r\n" marker */
    while (!(*(volatile uint32_t *)0x40004C00 & (1UL<<7))) ;
    *(volatile uint32_t *)0x40004C04 = 'R';
    while (!(*(volatile uint32_t *)0x40004C00 & (1UL<<7))) ;
    *(volatile uint32_t *)0x40004C04 = '\r';
    while (!(*(volatile uint32_t *)0x40004C00 & (1UL<<7))) ;
    *(volatile uint32_t *)0x40004C04 = '\n';
    while (!(*(volatile uint32_t *)0x40004C00 & (1UL<<6))) ; /* wait TC */
#endif

    /* Copy initialized data from flash to SRAM */
    src = &_sidata;
    dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }

    /* Zero BSS */
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }

    /* Enable FPU (CP10 + CP11 full access) before any float operations */
    *(volatile uint32_t *)0xE000ED88UL |= (0xFUL << 20);

    /*
     * Clean up residual state from bootloader before enabling interrupts.
     * The bootloader may leave NVIC interrupts enabled, DMA channels
     * running, or timers active.  Disable everything first.
     */
    /* Disable ALL NVIC interrupts (ICER0..ICER2 cover IRQ 0-95) */
    *(volatile uint32_t *)0xE000E180UL = 0xFFFFFFFFUL;
    *(volatile uint32_t *)0xE000E184UL = 0xFFFFFFFFUL;
    *(volatile uint32_t *)0xE000E188UL = 0xFFFFFFFFUL;
    /* Clear ALL pending NVIC interrupts */
    *(volatile uint32_t *)0xE000E280UL = 0xFFFFFFFFUL;
    *(volatile uint32_t *)0xE000E284UL = 0xFFFFFFFFUL;
    *(volatile uint32_t *)0xE000E288UL = 0xFFFFFFFFUL;

    /* System clock configuration - sets VTOR = 0x08003000 */
    SystemInit();

    /* VTOR is now set to our vector table - safe to enable interrupts */
    __asm volatile ("cpsie i");

    /* Enter application */
    main();

    /* Should never reach here */
    while (1) {
        __asm volatile ("wfi");
    }
}

/* ========================================================================
 *  Default_Handler - Catch-all for unimplemented ISRs.
 *
 *  Reports which exception fired, then lets the watchdog reset the radio.
 *
 *  This was a bare `while (1) { bkpt #0 }`, which is the worst possible
 *  behaviour during bring-up: NMI, MemManage, BusFault, UsageFault, SVCall,
 *  PendSV and every unhandled peripheral IRQ all land here and hang in total
 *  silence — indistinguishable from a main loop that runs but does nothing.
 *
 *  IPSR holds the active exception number:
 *    2=NMI  3=HardFault  4=MemManage  5=BusFault  6=UsageFault
 *    11=SVCall  14=PendSV  15=SysTick  16+n = external IRQ n
 *
 *  `bkpt #0` is deliberately gone: with no debugger attached it escalates to a
 *  HardFault, so the fault you end up looking at is caused by the trap itself
 *  rather than by the original problem.
 * ======================================================================== */

void Default_Handler(void)
{
    uint32_t ipsr;
    __asm volatile ("mrs %0, ipsr" : "=r" (ipsr));

    /* Feed IWDG once so the message gets out before any reset. */
    *(volatile uint32_t *)0x40003000UL = 0x0000AAAAUL;

    dbg_puts("\n[FAULT] Unhandled exception\n");
    dbg_reg("[FAULT] IPSR=0x", ipsr);
    if (ipsr >= 16u)
        dbg_reg("[FAULT] external IRQ n=", ipsr - 16u);

    /* Stop feeding the watchdog: a reset beats sitting bricked. */
    while (1) {
        /* spin until IWDG fires */
    }
}

/* ========================================================================
 *  HardFault_Handler - Captures fault context for debugging.
 *
 *  Uses a naked assembly trampoline to determine which stack was active
 *  (MSP or PSP) and pass the exception stack frame to the C handler.
 *  The frame layout is: R0, R1, R2, R3, R12, LR, PC, xPSR.
 * ======================================================================== */

void HardFault_Handler_C(const uint32_t *frame);

__attribute__((naked))
void HardFault_Handler(void)
{
    __asm volatile (
        "tst lr, #4          \n"  /* Test bit 2 of EXC_RETURN */
        "ite eq               \n"
        "mrseq r0, msp        \n"  /* Main stack was active */
        "mrsne r0, psp        \n"  /* Process stack was active */
        "b HardFault_Handler_C\n"
    );
}

void HardFault_Handler_C(const uint32_t *frame)
{
    /* Feed IWDG so we stay alive long enough to print diagnostics */
    *(volatile uint32_t *)0x40003000UL = 0x0000AAAAUL;

    dbg_puts("\n[FAULT] HardFault!\n");

    /* SCB->CFSR (Configurable Fault Status Register) */
    volatile uint32_t cfsr = *(volatile uint32_t *)0xE000ED28UL;
    dbg_reg("[FAULT] CFSR=0x", cfsr);

    /* SCB->HFSR (Hard Fault Status Register) */
    dbg_reg("[FAULT] HFSR=0x", *(volatile uint32_t *)0xE000ED2CUL);

    /* SCB->BFAR (Bus Fault Address) - valid if CFSR bit 15 set */
    if (cfsr & (1UL << 15)) {
        dbg_reg("[FAULT] BFAR=0x", *(volatile uint32_t *)0xE000ED38UL);
    }

    /* SCB->MMFAR (MemManage Fault Address) - valid if CFSR bit 7 set */
    if (cfsr & (1UL << 7)) {
        dbg_reg("[FAULT] MMFAR=0x", *(volatile uint32_t *)0xE000ED34UL);
    }

    /* Exception stack frame: [R0, R1, R2, R3, R12, LR, PC, xPSR] */
    if (frame) {
        dbg_reg("[FAULT] R0 =0x", frame[0]);
        dbg_reg("[FAULT] R12=0x", frame[4]);
        dbg_reg("[FAULT] LR =0x", frame[5]);
        dbg_reg("[FAULT] PC =0x", frame[6]);
        dbg_reg("[FAULT] xPSR=0x", frame[7]);
    }

    while (1) {
        *(volatile uint32_t *)0x40003000UL = 0x0000AAAAUL;
        __asm volatile ("nop");
    }
}
