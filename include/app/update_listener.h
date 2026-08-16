/*
 * update_listener.h - Soft entry into the bootloader's UART update mode
 *
 * Removes the need to hold the two side buttons and pull the battery for every
 * reflash. Listens on UART4 for the OEM host handshake:
 *
 *     host -> "PROGRAMBT9000U"      radio -> 0x06
 *     host -> "UPDATE"              radio -> 0x06, then hands over to the
 *                                            bootloader's update mode
 *
 * which is exactly what firmware_upload.py sends when invoked without --ptt.
 *
 * WHY THIS IS INTERRUPT-DRIVEN AND STARTS EARLY
 *
 * The whole value of this is rescuing a radio whose firmware is broken. A
 * listener that runs as a scheduler task is useless the moment the scheduler is
 * the thing that is broken -- which is precisely the bug being chased right
 * now. So it is armed immediately after SystemInit, before any peripheral
 * bring-up, and it runs from the UART4 RX interrupt. It keeps working through a
 * hung main loop, a wedged scheduler, or a task that never returns.
 *
 * It cannot survive a fault that disables interrupts or a lockup, so the side
 * buttons remain the guaranteed recovery path. This is a convenience, not a
 * replacement.
 *
 * HOW THE HANDOVER WORKS -- AND WHAT IS UNVERIFIED
 *
 * The bootloader decides to enter update mode by reading two GPIOs at boot
 * (PE5 and PA12, both active low) and does not check any RAM flag. A system
 * reset would clear anything we drove, so faking the buttons and resetting
 * cannot work.
 *
 * Instead: drive PE5 and PA12 low as outputs, then branch to the bootloader's
 * reset vector at 0x08000000. That is a jump, not a reset, so the pin state
 * survives into the bootloader's button check. The bootloader then runs its own
 * full init -- including its SRAM setup -- rather than us calling into the
 * middle of it with uninitialised globals.
 *
 * UNVERIFIED: if the bootloader's gpio_init() reconfigures PE5/PA12 as inputs
 * before it reads them, external pull-ups win and it boots the app normally
 * instead. That would make this a no-op, not a brick. Worst realistic case is
 * a hang, and the side buttons still recover. It has not yet been tested on
 * hardware.
 */

#ifndef APP_UPDATE_LISTENER_H
#define APP_UPDATE_LISTENER_H

#include <stdint.h>

#ifdef NO_UPDATE_LISTENER
/* Control build (make NOLISTENER=1): compile the listener out entirely so it
 * can be ruled in or out as the cause of a boot failure. */
#define update_listener_init()          ((void)0)
#define update_listener_feed(c)         ((void)(c))
#define update_listener_triggered()     (0)
#else

/* Arm the listener. Call as early as possible -- before hw_init() -- so a
 * later failure cannot stop it being armed. Enables UART4 RX + RXNE interrupt.
 * Safe to call when the debug UART is in use: debug output is TX-only on PC10,
 * this listens on RX (PC11). */
void update_listener_init(void);

/* Feed one received byte. Called from UART4_IRQHandler in drivers/uart.c,
 * which owns that vector. */
void update_listener_feed(uint8_t c);

/* Non-zero once the full handshake has been seen. Mostly for diagnostics --
 * the handover happens inside the ISR, so this rarely gets observed. */
uint8_t update_listener_triggered(void);

/* Hand over to the bootloader's UART update mode. Does not return.
 * Exposed so a menu entry or a key combination can trigger it too. */
void update_listener_enter_bootloader(void) __attribute__((noreturn));

#endif /* NO_UPDATE_LISTENER */

#endif /* APP_UPDATE_LISTENER_H */
