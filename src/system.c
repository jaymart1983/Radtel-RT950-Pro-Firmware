/*
 * system.c - System clock and tick initialization for the RT-950 Pro
 *
 * Target: Artery AT32F403A - 120 MHz from 8 MHz HSE crystal.
 *
 * Clock configuration verified from V0.27 binary:
 *   OEM SystemInit:   fw 0x080216DC (r2 0x080246DC)
 *   OEM clock_config: fw 0x0801FA90 (r2 0x08022A90)
 *
 *   HEXT = 8 MHz crystal (literal pool @ fw 0x0801A054)
 *   PLL  = HEXT x 15 = 120 MHz (CRM_CFG OR 0x80350000)
 *   AHB  = 120 MHz (HCLK, no prescaler)
 *   APB2 = 60 MHz  (divide by 2)
 *   APB1 = 60 MHz  (divide by 2)
 *   Timer clocks = 120 MHz (APB prescaler > 1, so doubled)
 *
 * PLL encoding (Artery-specific, verified from SDK at32f403a_407_crm.h):
 *   CRM_CFG[21:18] = PLLMULT_L = 13
 *   CRM_CFG[30:29] = PLLMULT_H = 0
 *   Full mult index = (0 << 4) | 13 = 13 -> CRM_PLL_MULT_15 -> x15
 *   CRM_CFG[31] = PLLRANGE = 1 (output > 72 MHz)
 *   CRM_CFG[16] = PLLRCS = 1 (HEXT as source)
 *   CRM_CFG[17] = PLLHEXTDIV = 0 (no HEXT division)
 *
 * Flash wait states handled automatically by auto-step (CRM MISC3).
 * The OEM firmware does NOT explicitly write FLASH_ACR during clock init.
 */

#include "at32f403a.h"
#include "debug_uart.h"

/* Global tick counter ----------------------------------------------- */
static volatile uint32_t systick_ms;

/* SysTick ISR (overrides weak default in startup.c) ----------------- */
void SysTick_Handler(void)
{
    systick_ms++;
    /* Feed IWDG (Independent Watchdog) every tick.
     * Bootloader enables IWDG before jumping to app firmware.
     * Key register = 0x40003000, reload key = 0xAAAA. */
    *(volatile uint32_t *)0x40003000UL = 0x0000AAAAUL;
}

/* ========================================================================
 *  delay_ms - Blocking delay using SysTick counter.
 * ======================================================================== */

void delay_ms(uint32_t ms)
{
    uint32_t start = systick_ms;
    while ((systick_ms - start) < ms) {
        /* Feed the watchdog while spinning.
         *
         * The bootloader enables IWDG before jumping to the application (see
         * the note in SystemInit above), so anything that blocks for longer
         * than the IWDG period resets the radio. Every HW_TEST is shaped as
         * `while (1) { ...; delay_ms(500); }` and none of them fed it, so each
         * one reset roughly once a second. That presents as a boot loop into
         * the bootloader and completely hides whatever the test was meant to
         * show — test 1 looked like a dead radio when it was actually working.
         *
         * Feeding here instead of in each test loop fixes all eleven at once,
         * and is right in principle: a bounded delay is not a hang, so it
         * should not trip the watchdog. Real lockups still reset, because they
         * are not sitting inside delay_ms(). */
        *(volatile uint32_t *)0x40003000UL = 0x0000AAAAUL;
    }
}

/* ========================================================================
 *  get_tick - Return current millisecond tick count.
 * ======================================================================== */

uint32_t get_tick(void)
{
    return systick_ms;
}

/* ========================================================================
 *  SysTick_Init - Configure SysTick for 1 ms interrupt.
 *
 *  Reload = (HCLK / 1000) - 1 = (120000000 / 1000) - 1 = 119999
 * ======================================================================== */

static void SysTick_Init(void)
{
    SysTick->LOAD = (120000000UL / 1000UL) - 1UL;
    SysTick->VAL  = 0;
    SysTick->CTRL = SYSTICK_CTRL_CLKSOURCE  /* processor clock */
                   | SYSTICK_CTRL_TICKINT    /* enable interrupt */
                   | SYSTICK_CTRL_ENABLE;    /* enable counter */
}

/* ========================================================================
 *  clock_config - Configure PLL: HEXT 8 MHz x 15 = 120 MHz.
 *
 *  Matches OEM clock_config at fw 0x0801FA90 (r2 0x08022A90).
 *
 *  OEM sequence:
 *    1. Enable HEXT, wait stable (timeout 0x3000)   - fw 0x0801FA90
 *    2. Set APB2 = /2 (bits [13:11] = 100)          - fw 0x0801FAA6
 *    3. Set APB1 = /2 (bits [10:8] = 100)           - fw 0x0801FAB2
 *    4. PLL config: CFG = (CFG & 0x1FC0FFFF) | 0x80350000  - fw 0x0801FABE
 *    5. Enable PLL, wait stable                     - fw 0x0801FACE
 *    6. Auto-step enable (MISC3[5:4] = 11)          - fw 0x0801FADC
 *    7. Switch SCLK to PLL, wait SWS                - fw 0x0801FAE4
 *    8. Auto-step disable                           - fw 0x0801FAF4
 * ======================================================================== */

static void clock_config(void)
{
    /* Step 1: Enable HEXT (8 MHz crystal) - fw 0x0801FA90 */
    CRM->CR |= CRM_CR_HSEEN;

    /* Wait for HEXT stable, timeout 0x3000 - fw 0x0801FA9A */
    uint32_t timeout = 0x3000;
    while (!(CRM->CR & CRM_CR_HSERDY) && --timeout)
        ;

    /* Step 2: APB2 prescaler = /2 - fw 0x0801FAA6 */
    /* OEM: BIC 0x3800 (clear bits[13:11]), ORR 0x2000 (set 100) */
    CRM->CFGR = (CRM->CFGR & ~CRM_CFGR_PPRE2_MASK)
              | (4UL << 11);  /* 100 = /2 -> APB2 = 60 MHz */

    /* Step 3: APB1 prescaler = /2 - fw 0x0801FAB2 */
    /* OEM: BIC 0x700 (clear bits[10:8]), ORR 0x400 (set 100) */
    CRM->CFGR = (CRM->CFGR & ~CRM_CFGR_PPRE1_MASK)
              | (4UL << 8);   /* 100 = /2 -> APB1 = 60 MHz */

    /* Step 4: PLL configuration - fw 0x0801FABE
     * OEM: AND 0x1FC0FFFF (clear PLL bits), ORR 0x80350000
     *
     * 0x80350000 decodes as:
     *   bit[31]     = 1    -> PLLRANGE (output > 72 MHz)
     *   bits[30:29] = 00   -> PLLMULT_H = 0
     *   bits[21:18] = 1101 -> PLLMULT_L = 13
     *   bit[17]     = 0    -> HEXT not divided
     *   bit[16]     = 1    -> PLL source = HEXT
     *   Full mult = (0 << 4) | 13 = 13 -> CRM_PLL_MULT_15 -> x15
     *   SYSCLK = 8 MHz x 15 = 120 MHz */
    CRM->CFGR = (CRM->CFGR & 0x1FC0FFFFUL) | 0x80350000UL;

    /* Step 5: Enable PLL - fw 0x0801FACE */
    CRM->CR |= CRM_CR_PLLEN;
    while (!(CRM->CR & CRM_CR_PLLRDY))
        ;

    /* Step 6: Auto-step enable - fw 0x0801FADC (calls sub @ fw 0x0801A06C)
     * CRM MISC3 (offset 0x54) bits[5:4] = 11
     * Auto-step gradually transitions clock frequency and adjusts flash
     * wait states automatically during the SCLK source switch. */
    CRM->MISC3 |= CRM_MISC3_AUTO_STEP_EN;

    /* Step 7: Switch SCLK to PLL - fw 0x0801FAE4 */
    CRM->CFGR = (CRM->CFGR & ~CRM_CFGR_SW_MASK) | CRM_CFGR_SW_PLL;
    while ((CRM->CFGR & CRM_CFGR_SWS_MASK) != CRM_CFGR_SWS_PLL)
        ;

    /* Step 8: Auto-step disable - fw 0x0801FAF4 (calls sub @ fw 0x0801A084) */
    CRM->MISC3 &= ~CRM_MISC3_AUTO_STEP_EN;
}

/* ========================================================================
 *  SystemInit - Reset clocks to safe state, then configure PLL.
 *
 *  Matches OEM SystemInit at fw 0x080216DC (r2 0x080246DC).
 *
 *  OEM sequence:
 *    1. Enable FPU (CPACR |= 0x00F00000)           - fw 0x080216DE
 *    2. Enable HICK (internal oscillator)           - fw 0x080216EC
 *    3. Reset CRM_CFG (AND 0xE8FF000C)             - fw 0x080216F4
 *    4. Disable HEXT, PLL, CSS                      - fw 0x08021700
 *    5. Clear PLL source/mult (AND 0x1700FFFF)     - fw 0x08021710
 *    6. Clear MISC1 fields (AND 0xFEFEFF00)        - fw 0x08021720
 *    7. Clear clock interrupts (CIR = 0x009F0000)  - fw 0x0802172A
 *    8. Call clock_config()                         - fw 0x08021730
 *    9. Set VTOR = 0x08000000                       - fw 0x08021734
 *
 *  Steps 10-11 are our additions (not part of OEM SystemInit).
 * ======================================================================== */

void SystemInit(void)
{
    /* ---- 1. Enable FPU - fw 0x080216DE ---- */
    /* CPACR (0xE000ED88) |= 0x00F00000: grant CP10/CP11 full access */
    FPU_CPACR |= (0xFUL << 20);

    /* ---- 2. Enable HICK (internal oscillator) - fw 0x080216EC ---- */
    CRM->CR |= CRM_CR_HSIEN;

    /* ---- 3. Reset CRM_CFG - fw 0x080216F4 ---- */
    /* AND 0xE8FF000C: preserves SWS[3:2] and reserved bits,
     * clears prescalers, PLL config, SW bits */
    CRM->CFGR &= 0xE8FF000CUL;

    /* ---- 4. Disable HEXT, PLL, CSS - fw 0x08021700 ---- */
    /* AND 0xFEF6FFFF: clears HSEON(16), CSSON(19), PLLON(24) */
    CRM->CR &= 0xFEF6FFFFUL;
    /* BIC bit 18: clears HSEBYP */
    CRM->CR &= ~CRM_CR_HSEBYP;

    /* ---- 5. Clear PLL source/mult in CFG - fw 0x08021710 ---- */
    /* AND 0x1700FFFF: clears PLLMULT_L[21:18], PLLSRC[16],
     * PLLHEXTDIV[17], PLLMULT_H[30:29], PLLRANGE[31] */
    CRM->CFGR &= 0x1700FFFFUL;

    /* ---- 6. Clear MISC1 fields - fw 0x08021720 ---- */
    /* AND 0xFEFEFF00 */
    CRM->MISC1 &= 0xFEFEFF00UL;

    /* ---- 7. Clear clock interrupts - fw 0x0802172A ---- */
    CRM->CIR = 0x009F0000UL;

    /* ---- 8. Configure PLL: HEXT 8 MHz x 15 = 120 MHz - fw 0x08021730 ---- */
    clock_config();

    /* ---- 9. Set VTOR ---- */
    /* OEM SystemInit writes 0x08000000 here (fw 0x08021734), then relocates
     * VTOR to the app base later (fw 0x0801886C).  We set it to our flash
     * origin immediately so that SysTick and other ISRs use our vector
     * table from the start. */
    SCB->VTOR = 0x08003000UL;

    /* ---- 10. NVIC Priority Group 4 (OEM rcc_periph_enable @ 0x08018750) ----
     * OEM sets PRIGROUP=3 → 4 preemption bits, 0 sub-priority bits.
     * Must be configured before any NVIC_SetPriority calls.
     * AIRCR requires VECTKEY (0x05FA) in upper 16 bits for write access. */
    SCB->AIRCR = SCB_AIRCR_VECTKEY | (3UL << 8);

    /* ---- 11. Enable GPIO port clocks (our addition) ---- */
    CRM->APB2EN |= CRM_APB2EN_AFIOEN
                  | CRM_APB2EN_IOPAEN
                  | CRM_APB2EN_IOPBEN
                  | CRM_APB2EN_IOPCEN
                  | CRM_APB2EN_IOPDEN
                  | CRM_APB2EN_IOPEEN;

    /* ---- 11b. Disable JTAG, keep SWD (OEM @ 0x80123D6) ----
     * OEM gpio_modes_init calls AFIO remap with 0x02000000 → SWJ_CFG=010.
     * This frees PA15, PB3, PB4 from JTAG function.
     * IOMUX_REMAP register (0x40010004) bits[26:24] = SWJ_CFG:
     *   000 = Full SWJ (JTAG+SWD)  - PA15,PB3,PB4 locked to JTAG
     *   010 = JTAG disabled, SWD only - PA15,PB3,PB4 freed as GPIO
     * Write is special: SWJ_CFG is write-only, must clear then set. */
    {
        volatile uint32_t *iomux_remap = (volatile uint32_t *)0x40010004UL;
        uint32_t val = *iomux_remap;
        val &= ~(7UL << 24);    /* clear SWJ_CFG[26:24] */
        val |=  (2UL << 24);    /* SWJ_CFG = 010: JTAG off, SWD on */
        *iomux_remap = val;
    }

    /* ---- 12. Start SysTick (1 ms tick, our addition) ---- */
    SysTick_Init();

    /* ---- 13. Debug UART (compile-time optional) ---- */
    dbg_init();
    dbg_puts("\n[DBG] SystemInit OK: HCLK=120MHz VTOR=0x08003000\n");
    dbg_reg("[DBG]   CRM->CR=0x", CRM->CR);
    dbg_reg("[DBG]   CRM->CFGR=0x", CRM->CFGR);
}
