# Bring-up ladder

Twelve images, in the order to flash them. Each rung adds one subsystem, so a
failure tells you *which* layer broke rather than just "the firmware doesn't
work". Rungs 1–11 are the repo's own `HW_TEST` targets; rung 12 is the real
firmware.

All are built with `DEBUG=1`, so every one prints to UART4 on PC10 — the same
pins as the programming cable.

```bash
tools/flash_and_watch.sh images/01-blinky.BTF
```

That releases the serial port, refuses to upload if anything else holds it,
uploads, then captures the boot trace to `logs/` while showing it live.

**You should not need the side buttons.** Every image here carries the update
listener, so a running radio hands itself to the bootloader when it sees the
host handshake. Just leave the radio powered on and run the command.

Two things to know about that:

* The listener is armed at the very top of `main()` and runs from the UART4 RX
  interrupt, so it works even when the firmware is otherwise broken — a hung
  main loop or a wedged scheduler does not stop it.
* It cannot survive a fault that disables interrupts. **The side buttons remain
  the guaranteed path**: hold the bottom two while powering on, then run the
  same command. The script detects which mode the radio is in and adapts.

The handover itself is **not yet verified on hardware**. If it does not work it
degrades to a normal boot rather than bricking anything.

| # | image | proves | pass looks like |
|---|---|---|---|
| 01 | `blinky` | code runs at all; clocks, reset vector, power latch | LED blinks |
| 02 | `uart-echo` | UART4 both directions | typed characters echo back |
| 03 | `lcd-pattern` | display driver and panel | clean test pattern, no garbling |
| 04 | `keypad-encoder` | key matrix and knob | `[KEY]` / `[ENC]` lines per press and detent |
| 05 | `dac-tone` | audio path and amplifier | audible tone |
| 06 | `spi-flash-id` | config storage on SPI2 | Winbond JEDEC id, expect `EF 40 15` |
| 07 | `bk4829-id` | both transceivers | chip id from each of the two chips |
| 08 | `si4732-rev` | broadcast receiver | firmware revision reported |
| 09 | `adc-monitor` | battery and RSSI sensing | plausible voltage |
| 10 | `gps` | GPS UART and NMEA parse | NMEA sentences |
| 11 | `full-diagnostic` | everything above in one run | all subsystems report |
| 12 | `full-firmware` | scheduler, UI, our features | see below |

## What rung 12 answers

The known failure is that the radio boots — splash renders, audio tests play —
but the main screen is garbled and input is dead. Everything still working runs
inside `main()` during init; everything dead is driven by a **scheduler task**.

The trace separates those cases immediately:

| trace | meaning |
|---|---|
| no `[DBG] app_init complete` | init hangs — the last line printed names the culprit |
| no `[SCHED] run: tasks=12` | never reached the scheduler |
| `[SCHED] run:` then silence | loop entered but no task ever dispatches — suspect the tick |
| `[SCHED] hb t=` repeating | scheduler is healthy; the fault is inside individual tasks |
| `[FAULT] ...` | an exception fired; IPSR names which |

`[KBD_DIAG]` prints raw keypad GPIO every 2 s for the first 10 samples, and
`[KEY] press:` appears whenever a key is actually detected — so rung 12 also
distinguishes "keypad not scanned" from "keypad scanned but reads nothing".

## Recovery

Any rung can be undone. The bootloader lives at `0x08000000`, below anything
these images write, so it always survives:

```bash
tools/flash_and_watch.sh "binary/RT_950Pro_V0.27_260203/RT_950Pro_V0.27_260203.BTF"
```

Channel data is never at risk — it lives in external SPI flash, which firmware
upload does not touch.

## Note on the fault handler

`Default_Handler` used to be a silent `while (1) { bkpt #0 }`, so any unhandled
exception hung the radio with no output — indistinguishable from a main loop
that runs but does nothing. It now prints the IPSR exception number and lets the
watchdog reset. If a rung dies with `[FAULT]`, that number says exactly which
exception fired.
