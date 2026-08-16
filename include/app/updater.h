/*
 * updater.h - In-application firmware updater
 *
 * Receives a .BTF over UART4 and writes it to internal flash, then resets.
 * The bootloader is never involved -- it has no soft entry, and Radtel's own
 * firmware does not use it for updates either. See updater.c.
 */

#ifndef APP_UPDATER_H
#define APP_UPDATER_H

#include <stdint.h>

/* Run the update protocol. Does not return: it resets the MCU when the host
 * finishes or goes quiet. Disables interrupts, because every ISR lives in the
 * flash this is about to erase. */
void updater_run(void) __attribute__((noreturn));

#endif /* APP_UPDATER_H */
