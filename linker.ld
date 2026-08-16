/*
 * Linker script for Radtel RT-950 Pro
 * MCU: Artery AT32F403A (ARM Cortex-M4F)
 *
 * Flash layout:
 *   0x08000000 - 0x08002FFF : OEM bootloader (12 KB, read-only)
 *   0x08003000 - 0x080FFFFF : Application firmware (1012 KB)
 *
 * The OEM bootloader validates [0x08003000] as a RAM stack pointer
 * (SP & 0x2FFE0000 == 0x20000000), reads [0x08003004] as Reset_Handler,
 * sets MSP, and jumps.  No signature or CRC check.
 * See also: docs/bootloader.md for full bootloader analysis.
 *
 * Stack top 0x20017BB0 matches V0.18 firmware vector table entry [0].
 *
 * SPIM (SPI memory-mapped flash) at 0x08400000 is available but unused.
 */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08003000, LENGTH = 1012K
    SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
    /* SPIM (rx) : ORIGIN = 0x08400000, LENGTH = 16384K  - AT32 external SPI flash */
}

/* Initial stack pointer - top of SRAM.
 * V0.27 OEM uses 0x20015D10 (limited by .data section layout).
 * Our firmware uses the full 96 KB SRAM (0x20000000-0x20017FFF).
 * _estack is set near the top, leaving headroom for alignment. */
_estack = 0x20017BB0;

ENTRY(Reset_Handler)

SECTIONS
{
    /* Vector table + code in flash */
    .text :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))   /* Vector table must be first */

        /* Model signature for bootloader firmware upload (cmd 0x02).
         * Must be at binary offset 0x3E0 (absolute 0x080033E0).
         * The bootloader compares 12 bytes against its "RT-950      " string
         * during the UART upload handshake. */
        . = ORIGIN(FLASH) + 0x3E0;
        KEEP(*(.model_sig))

        /* Model signature for bootloader boot-time validation.
         * Must be at binary offset 0x7E0 (absolute 0x080037E0).
         * After upload, the key block (BTF[0x400:0x800]) is NOT written
         * to flash, so firmware offset 0x7E0 maps to flash address
         * 0x08003000 + 0x7E0 = 0x080037E0.  The bootloader's
         * check_spi_model() reads 32 bytes from this address and
         * compares against "RT-950      " before jumping to the app.
         * If the check fails, the bootloader erases the firmware and
         * displays "firmware error". */
        . = ORIGIN(FLASH) + 0x7E0;
        KEEP(*(.boot_model_sig))

        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)

        KEEP(*(.init))
        KEEP(*(.fini))

        . = ALIGN(4);
        _etext = .;
    } > FLASH

    /* Read-only data */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    /* ARM exception unwinding -- needed by some toolchains */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH

    /* Initialized data: stored in flash, copied to SRAM at startup */
    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;

        /* Code that must execute from RAM.
         *
         * The in-application updater erases and rewrites the flash it would
         * otherwise be running from, so it cannot live there. Placing it in
         * .data means startup copies it to SRAM alongside initialised data,
         * and it runs from there with no flash accesses at all. Everything it
         * calls must be in this section too -- a single call back into flash
         * mid-erase hangs the CPU. */
        *(.ramfunc)
        *(.ramfunc*)

        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > SRAM AT> FLASH

    /* Zero-initialized data */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } > SRAM

    /* Heap extends from end of BSS to bottom of stack region */
    _heap_start = _ebss;
    _heap_end   = _estack - 0x400;   /* leave 1K guard below stack */

    /DISCARD/ :
    {
        libc.a(*)
        libm.a(*)
        libgcc.a(*)
    }
}
