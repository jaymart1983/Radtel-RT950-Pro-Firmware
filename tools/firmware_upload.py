#!/usr/bin/env python3
"""
firmware_upload.py - Firmware update tool for the Radtel RT-950 Pro

Uploads BTF firmware files to the radio using the bootloader protocol
(reverse engineered from RT-950_EnUPDATE.exe and bootloader disassembly).

Two-phase protocol:
  Phase 1: PROGRAMBT9000U handshake + "UPDATE" to enter bootloader mode
  Phase 2: 0xAA-framed packets for firmware transfer

The BTF file is sent as-is (still encrypted). The MCU bootloader decrypts
it internally using the key embedded at BTF offset 0x400.

Alternative update paths (not implemented here):
  - BLE: mobile app sends same protocol over Bluetooth characteristics,
    128-byte chunks, with CRC verify (cmd 0x06) every 8 chunks and
    overall CRC (cmd 0x14). Targets SPI flash region 0x001C0000.
  - SPI flash OTA: bootloader checks magic 0xA55A at SPI addr 0x300000,
    reads length from 0x300004, verifies "Ver" at 0x300700, CRC checks
    firmware at 0x300100, then programs to internal flash on next boot.

Usage:
    python3 firmware_upload.py upload /dev/ttyUSB0 firmware.BTF
    python3 firmware_upload.py upload /dev/ttyUSB0 firmware.BTF --ptt
    python3 firmware_upload.py verify /dev/ttyUSB0 firmware.BTF
    python3 firmware_upload.py probe /dev/ttyUSB0

Requires: pyserial (pip install pyserial)
"""

import argparse
import math
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("Error: pyserial required. Install with: pip install pyserial",
          file=sys.stderr)
    sys.exit(1)


# Protocol constants
HANDSHAKE_STRING = b"PROGRAMBT9000U"
UPDATE_STRING    = b"UPDATE"
ACK_BYTE         = 0x06
HEADER           = 0xAA

# Data-block transport tuning. See send_data() for why these exist.
DATA_BLOCK_RETRIES = 5      # attempts per block before giving up
DATA_BLOCK_DELAY   = 0.012  # settle time after each accepted block
DATA_RETRY_DELAY   = 0.150  # longer pause after a failure, to let flash finish
FOOTER           = 0x55

# Bootloader commands
CMD_PROBE         = 0x42   # Initial probe (returns error but confirms boot mode)
CMD_VERSION       = 0x0A   # Version handshake ("BOOTLOADER_V3")
CMD_MODEL         = 0x02   # Model signature verification
CMD_PKG_COUNT     = 0x04   # Announce total package count
CMD_DATA          = 0x03   # Send data packet (1024 bytes)
CMD_END           = 0x45   # End / finalize update

# BTF layout
BTF_MODEL_OFFSET  = 0x3E0  # Model signature (32 bytes)
BTF_MODEL_SIZE    = 32
BTF_KEY_OFFSET    = 0x400  # Encryption key seed (16 bytes)
BTF_KEY_SIZE      = 16
DATA_BLOCK_SIZE   = 1024   # Bytes per data packet

# Response codes
RESULT_ACK        = 0x06
RESULT_LEN_ERR    = 0xE1
RESULT_VERIFY_ERR = 0xE2
RESULT_FLASH_ERR  = 0xE3
RESULT_UNK_CMD    = 0xE5
RESULT_MODEL_ERR  = 0xE6

RESULT_NAMES = {
    0x06: "ACK",
    0xE1: "Wrong data length",
    0xE2: "Data verification error",
    0xE3: "Flash write error",
    0xE5: "Ready (cmd not applicable in update mode)",
    0xE6: "Model mismatch",
}

# Probe flood parameters: after UPDATE, the bootloader polls UART for a
# limited window. We must send probes rapidly to catch the window.
PROBE_FLOOD_INTERVAL = 0.050  # 50ms between probe attempts
PROBE_FLOOD_TIMEOUT  = 15.0   # max seconds to keep trying

DEFAULT_BAUD    = 115200
DEFAULT_TIMEOUT = 3.0
VERSION_STRING  = b"BOOTLOADER_V3"


def crc_ccitt(data: bytes) -> int:
    """CRC-CCITT (poly 0x1021, init 0x0000) over data bytes."""
    crc = 0x0000
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def build_packet(cmd: int, args: int = 0, data: bytes = b"") -> bytes:
    """Build an 0xAA-framed bootloader packet.

    Format: [0xAA, cmd, args_hi, args_lo, len_hi, len_lo, data..., crc_hi, crc_lo, 0x55]
    CRC covers bytes [1..5+len(data)] (cmd through end of data).
    """
    args_hi = (args >> 8) & 0xFF
    args_lo = args & 0xFF
    dlen = len(data)
    len_hi = (dlen >> 8) & 0xFF
    len_lo = dlen & 0xFF

    payload = bytes([cmd, args_hi, args_lo, len_hi, len_lo]) + data
    crc = crc_ccitt(payload)

    return bytes([HEADER]) + payload + struct.pack(">H", crc) + bytes([FOOTER])


def parse_response(resp: bytes) -> dict:
    """Parse a bootloader response packet.

    Response format: [0xAA, cmd, 0x00, result, 0x00, 0x00, crc_hi, crc_lo, 0x55]
    Returns dict with cmd, result, raw, or None if invalid.
    """
    if not resp or len(resp) < 9:
        return None
    if resp[0] != HEADER:
        return None

    footer_idx = resp.rfind(bytes([FOOTER]))
    if footer_idx < 0:
        return None

    return {
        "cmd": resp[1],
        "result": resp[3],
        "result_name": RESULT_NAMES.get(resp[3], f"0x{resp[3]:02X}"),
        "raw": resp,
    }


def serial_monitor(port: str, baud: int = DEFAULT_BAUD, timestamp: bool = True):
    """Monitor serial port output. Prints incoming data as text.
    Ctrl+C to exit.

    Every print here flushes. Without that, piping the monitor to a file or
    another process gets Python's default 8 KB block buffering, and a boot
    trace of a few hundred bytes never reaches the far end -- it sits in the
    buffer until the process exits, and is lost entirely if the monitor is
    killed by a signal. That failure looks exactly like a radio that is not
    printing anything, which is a very expensive thing to misdiagnose.
    """

    print(f"Monitoring {port} @ {baud} baud (Ctrl+C to stop)", flush=True)
    print("-" * 60, flush=True)

    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
    )
    ser.reset_input_buffer()

    line_buf = ""
    t0 = time.monotonic()
    try:
        while True:
            data = ser.read(256)
            if not data:
                continue
            text = data.decode('ascii', errors='replace')
            for ch in text:
                if ch == '\n':
                    if timestamp:
                        elapsed = time.monotonic() - t0
                        prefix = f"[{elapsed:8.3f}] "
                    else:
                        prefix = ""
                    print(f"{prefix}{line_buf}", flush=True)
                    line_buf = ""
                elif ch == '\r':
                    continue
                else:
                    line_buf += ch
    except KeyboardInterrupt:
        pass
    finally:
        # Flush any partial line, so the last thing printed before a hang is
        # not swallowed -- that line is usually the most informative one.
        if line_buf:
            print(line_buf, flush=True)
        print("\n--- monitor stopped ---", flush=True)
        ser.close()


class FirmwareUploader:
    """Handles the complete firmware upload protocol."""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD,
                 timeout: float = DEFAULT_TIMEOUT, verbose: bool = False):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.verbose = verbose
        self.ser = None

    def log(self, msg: str):
        if self.verbose:
            print(f"  [DBG] {msg}")

    def open(self):
        """Open serial port."""
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_raw(self, data: bytes):
        """Send raw bytes."""
        self.log(f"TX ({len(data)}): {data[:32].hex()}" +
                 ("..." if len(data) > 32 else ""))
        self.ser.write(data)
        self.ser.flush()

    def recv_raw(self, count: int = 64, timeout: float = None) -> bytes:
        """Read raw bytes with optional timeout override."""
        old_timeout = self.ser.timeout
        if timeout is not None:
            self.ser.timeout = timeout
        try:
            data = self.ser.read(count)
            self.log(f"RX ({len(data)}): {data.hex()}" if data else "RX: (empty)")
            return data
        finally:
            self.ser.timeout = old_timeout

    def recv_until_footer(self, timeout: float = 5.0) -> bytes:
        """Read bytes until 0x55 footer or timeout."""
        old_timeout = self.ser.timeout
        self.ser.timeout = timeout
        buf = bytearray()
        start = time.time()
        try:
            while time.time() - start < timeout:
                b = self.ser.read(1)
                if not b:
                    break
                buf.extend(b)
                if b[0] == FOOTER and len(buf) >= 9:
                    break
            self.log(f"RX ({len(buf)}): {bytes(buf).hex()}" if buf else "RX: (empty)")
            return bytes(buf)
        finally:
            self.ser.timeout = old_timeout

    def _wait_ack(self, what: str, timeout: float = 3.0) -> bool:
        """Scan the incoming stream for an ACK rather than demanding it first.

        A running radio may be emitting debug output on this same UART, so the
        byte immediately after the handshake is frequently unrelated. Requiring
        resp[0] == ACK made a perfectly good handshake look like a failure --
        the reported "got: b6" was simply the first byte of a debug line.
        """
        deadline = time.time() + timeout
        seen = bytearray()
        while time.time() < deadline:
            chunk = self.recv_raw(64, timeout=0.3)
            if chunk:
                seen.extend(chunk)
                if ACK_BYTE in chunk:
                    if len(seen) > 1:
                        print(f"  {what} ACK found after {len(seen) - 1} bytes "
                              f"of other output (debug traffic on the same UART)")
                    else:
                        print(f"  {what} ACK received")
                    return True
        print(f"  ERROR: No ACK to {what} "
              f"(saw: {bytes(seen[:24]).hex() if seen else 'nothing'})")
        return False

    def handshake(self) -> bool:
        """Phase 1: PROGRAMBT9000U + UPDATE handshake."""
        print("Phase 1: Entering update mode...")

        # Clear anything already buffered, so a debug line sent before we
        # started is not mistaken for a reply.
        self.ser.reset_input_buffer()

        self.send_raw(HANDSHAKE_STRING)
        if not self._wait_ack("Handshake"):
            return False

        self.send_raw(UPDATE_STRING)
        if not self._wait_ack("UPDATE"):
            return False

        print("  Radio entering bootloader mode")
        return True

    def send_command(self, cmd: int, args: int = 0, data: bytes = b"",
                     expect_ack: bool = True, timeout: float = 5.0) -> dict:
        """Send a bootloader command and read response."""
        pkt = build_packet(cmd, args, data)
        self.send_raw(pkt)

        if not expect_ack:
            return {"result": 0, "result_name": "no-ack-expected"}

        resp_bytes = self.recv_until_footer(timeout=timeout)
        if not resp_bytes:
            return None

        # OEM tool just checks for 0xAA header presence
        if resp_bytes[0] == HEADER:
            parsed = parse_response(resp_bytes)
            if parsed:
                return parsed
            # Minimal parse: just header present
            return {
                "cmd": cmd,
                "result": resp_bytes[3] if len(resp_bytes) > 3 else 0xFF,
                "result_name": RESULT_NAMES.get(
                    resp_bytes[3] if len(resp_bytes) > 3 else 0xFF,
                    "unknown"),
                "raw": resp_bytes,
            }

        return None

    def probe(self, flood: bool = False) -> bool:
        """Send cmd 0x42 probe. Bootloader responds with 0xAA-framed packet
        to confirm it's running.

        In direct mode (buttons held), the bootloader is already in
        uart_update_mode() and responds 0xE5 (cmd not applicable).

        In upgrade mode (after PROGRAMBT9000U + UPDATE), the MCU resets
        and the bootloader polls UART for a limited window. We must flood
        probes rapidly to catch this window (flood=True).
        """
        if flood:
            return self._probe_flood()

        print("  Probing bootloader (cmd 0x42)...")
        resp = self.send_command(CMD_PROBE, timeout=1.0)
        if resp is None:
            print("  ERROR: No response to probe")
            return False
        print(f"  Bootloader responding: {resp['result_name']}")
        return True

    def _probe_flood(self) -> bool:
        """Rapidly send probe packets to catch the bootloader's UART
        polling window after MCU reset.

        The OEM tool sleeps 80ms after UPDATE ACK then sends a probe.
        We use a tight loop with short read timeouts to maximize our
        chance of hitting the narrow window (~200ms).
        """
        print("  Probing bootloader (rapid flood)...")
        pkt = build_packet(CMD_PROBE)
        start = time.time()
        attempt = 0
        old_timeout = self.ser.timeout

        # Brief delay for MCU to reset and bootloader UART to initialize
        time.sleep(0.030)
        self.ser.reset_input_buffer()
        self.ser.timeout = 0.015  # 15ms read timeout for tight loop

        try:
            while time.time() - start < PROBE_FLOOD_TIMEOUT:
                self.ser.write(pkt)
                self.ser.flush()
                attempt += 1

                # Read immediately - no sleep between write and read
                resp = self.ser.read(64)
                if resp and len(resp) >= 9 and resp[0] == HEADER:
                    elapsed = time.time() - start
                    parsed = parse_response(resp)
                    result_name = parsed["result_name"] if parsed else "?"
                    self.log(f"TX ({len(pkt)}): {pkt.hex()}")
                    self.log(f"RX ({len(resp)}): {resp.hex()}")
                    print(f"  Bootloader responded after {elapsed:.3f}s "
                          f"({attempt} probes): {result_name}")
                    # Wait for bootloader to finish flash_erase_sector(),
                    # transition to uart_update_mode(), and enter receive loop.
                    # Flash erase can take 500ms+ on AT32F403A.
                    # Consume any stale responses before returning.
                    time.sleep(0.200)
                    self.ser.timeout = 0.500
                    while True:
                        stale = self.ser.read(64)
                        if stale:
                            self.log(f"Stale ({len(stale)}): {stale.hex()}")
                        else:
                            break
                    self.ser.reset_input_buffer()
                    return True
        finally:
            self.ser.timeout = old_timeout

        print(f"  ERROR: No bootloader response after {attempt} probes "
              f"({PROBE_FLOOD_TIMEOUT}s)")
        return False

    def version_handshake(self) -> bool:
        """Send version identification (cmd 0x0A, "BOOTLOADER_V3")."""
        print("  Version handshake (cmd 0x0A)...")
        resp = self.send_command(CMD_VERSION, data=VERSION_STRING)
        if resp is None:
            print("  ERROR: No response to version handshake")
            return False
        if resp["result"] != RESULT_ACK:
            print(f"  ERROR: Version handshake failed: {resp['result_name']}")
            return False
        print("  Version handshake OK")
        return True

    def model_verify(self, btf_data: bytes) -> bool:
        """Send model verification (cmd 0x02, 32 bytes from BTF@0x3E0)."""
        print("  Model verification (cmd 0x02)...")
        model_sig = btf_data[BTF_MODEL_OFFSET:BTF_MODEL_OFFSET + BTF_MODEL_SIZE]
        model_str = model_sig[:12].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"  Model string: \"{model_str}\"")

        resp = self.send_command(CMD_MODEL, data=model_sig)
        if resp is None:
            print("  ERROR: No response to model verification")
            return False
        if resp["result"] != RESULT_ACK:
            print(f"  ERROR: Model verification failed: {resp['result_name']}")
            return False
        print("  Model verification OK")
        return True

    def send_package_count(self, total_packages: int) -> bool:
        """Send total package count (cmd 0x04)."""
        print(f"  Package count: {total_packages} (cmd 0x04)...")
        count_data = struct.pack(">H", total_packages - 1)
        resp = self.send_command(CMD_PKG_COUNT, data=count_data)
        if resp is None:
            print("  ERROR: No response to package count")
            return False
        if resp["result"] != RESULT_ACK:
            print(f"  ERROR: Package count rejected: {resp['result_name']}")
            return False
        print("  Package count accepted")
        return True

    def send_data(self, btf_data: bytes) -> bool:
        """Send firmware data in 1024-byte blocks (cmd 0x03)."""
        total_packages = math.ceil(len(btf_data) / DATA_BLOCK_SIZE)
        print(f"\nPhase 2: Uploading {len(btf_data)} bytes in {total_packages} blocks...")

        for seq in range(total_packages):
            offset = seq * DATA_BLOCK_SIZE
            block = btf_data[offset:offset + DATA_BLOCK_SIZE]

            # Zero-pad last block if needed
            if len(block) < DATA_BLOCK_SIZE:
                block = block + b'\x00' * (DATA_BLOCK_SIZE - len(block))

            # Retry with resync.
            #
            # Without this, uploads failed at a DIFFERENT block each attempt
            # ("Wrong data length" at 16, then 21). That error is the bootloader
            # saying the frame it received did not match its length field, i.e.
            # bytes went missing -- it is a transport fault, not bad data. The
            # radio drops incoming bytes while it is busy erasing/writing a
            # flash page, and with a 1029-byte packet already streaming there is
            # nothing to stop the frame being chopped.
            #
            # Re-sending a block is safe: the sequence number is explicit in the
            # packet (args=seq), so the bootloader writes the same destination
            # regardless of how many times it is sent.
            resp = None
            for attempt in range(DATA_BLOCK_RETRIES):
                if attempt:
                    # Drop whatever partial frame desynced us, and give the
                    # radio time to finish its flash write before trying again.
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    time.sleep(DATA_RETRY_DELAY)

                resp = self.send_command(CMD_DATA, args=seq, data=block,
                                         timeout=10.0)
                if resp is not None and resp["result"] == RESULT_ACK:
                    break

                why = "no response" if resp is None else resp["result_name"]
                print(f"\n  block {seq}: {why}"
                      f" (attempt {attempt + 1}/{DATA_BLOCK_RETRIES})")

            if resp is None:
                print(f"\n  ERROR: No response to data block {seq}"
                      f" after {DATA_BLOCK_RETRIES} attempts")
                return False
            if resp["result"] != RESULT_ACK:
                print(f"\n  ERROR: Block {seq} rejected: {resp['result_name']}"
                      f" after {DATA_BLOCK_RETRIES} attempts")
                return False

            # Breathing room so the next packet does not arrive while the radio
            # is still committing this one.
            time.sleep(DATA_BLOCK_DELAY)

            # Progress bar
            pct = (seq + 1) * 100 // total_packages
            bar_len = 40
            filled = bar_len * (seq + 1) // total_packages
            bar = '#' * filled + '-' * (bar_len - filled)
            print(f"\r  [{bar}] {pct:3d}% ({seq+1}/{total_packages})", end='', flush=True)

        print()  # newline after progress bar
        return True

    def finalize(self) -> bool:
        """Send end command (cmd 0x45)."""
        print("  Finalizing update (cmd 0x45)...")
        resp = self.send_command(CMD_END, timeout=10.0)
        if resp is None:
            print("  ERROR: No response to end command")
            return False
        if resp["result"] != RESULT_ACK:
            print(f"  WARNING: End command response: {resp['result_name']}")
        print("  Update finalized")
        return True

    def upload(self, btf_path: str, ptt_mode: bool = False) -> bool:
        """Full firmware upload sequence."""
        # Read and validate BTF file
        print(f"Reading BTF file: {btf_path}")
        with open(btf_path, 'rb') as f:
            btf_data = f.read()

        if len(btf_data) < BTF_MODEL_OFFSET + BTF_MODEL_SIZE:
            print(f"ERROR: BTF file too small ({len(btf_data)} bytes)")
            return False

        total_packages = math.ceil(len(btf_data) / DATA_BLOCK_SIZE)
        model_sig = btf_data[BTF_MODEL_OFFSET:BTF_MODEL_OFFSET + BTF_MODEL_SIZE]
        model_str = model_sig[:12].rstrip(b'\x00').decode('ascii', errors='replace')
        key_seed = btf_data[BTF_KEY_OFFSET:BTF_KEY_OFFSET + BTF_KEY_SIZE]

        print(f"  File size: {len(btf_data)} bytes ({total_packages} blocks)")
        print(f"  Model: \"{model_str}\"")
        print(f"  Key seed: {key_seed.hex()}")
        print()

        self.open()
        try:
            # Phase 1: Enter bootloader mode
            if ptt_mode:
                print("Direct bootloader mode: radio should be in bootloader "
                      "(hold side buttons at power-on)")
                time.sleep(0.5)
                self.ser.reset_input_buffer()
            else:
                if not self.handshake():
                    return False

            # Phase 2: Bootloader protocol
            # In upgrade mode (non-PTT), we must flood probes rapidly
            # to catch the bootloader's brief UART polling window.
            use_flood = not ptt_mode
            if not self.probe(flood=use_flood):
                if ptt_mode:
                    print("WARNING: Probe failed, attempting to continue...")
                else:
                    print("ERROR: Could not reach bootloader")
                    return False

            if not self.version_handshake():
                return False

            if not self.model_verify(btf_data):
                return False

            if not self.send_package_count(total_packages):
                return False

            if not self.send_data(btf_data):
                return False

            if not self.finalize():
                return False

            print("\nFirmware upload complete!")
            print("Radio should reboot with new firmware.")
            return True

        finally:
            self.close()


def validate_btf(btf_path: str):
    """Validate a BTF file without uploading."""
    print(f"Validating BTF file: {btf_path}")
    with open(btf_path, 'rb') as f:
        data = f.read()

    print(f"  File size: {len(data)} bytes")
    total_packages = math.ceil(len(data) / DATA_BLOCK_SIZE)
    print(f"  Data blocks: {total_packages}")

    if len(data) < BTF_MODEL_OFFSET + BTF_MODEL_SIZE:
        print("  ERROR: File too small (missing model signature)")
        return

    # Vector table check
    sp = struct.unpack_from('<I', data, 0)[0]
    reset = struct.unpack_from('<I', data, 4)[0]
    print(f"\n  Vector table:")
    print(f"    Initial SP: 0x{sp:08X}", end='')
    if 0x20000000 <= sp <= 0x2FFDFFFF:
        print(" (valid SRAM)")
    else:
        print(" (WARNING: outside SRAM range)")

    print(f"    Reset vector: 0x{reset:08X}", end='')
    if 0x08003000 <= reset <= 0x080FFFFF:
        print(" (valid app flash)")
    else:
        print(f" (WARNING: outside expected range)")

    # Model signature
    model_sig = data[BTF_MODEL_OFFSET:BTF_MODEL_OFFSET + BTF_MODEL_SIZE]
    model_str = model_sig[:12].rstrip(b'\x00').decode('ascii', errors='replace')
    print(f"\n  Model signature @ 0x{BTF_MODEL_OFFSET:04X}:")
    print(f"    String: \"{model_str}\"")
    print(f"    Full 32 bytes: {model_sig.hex()}")

    # Encryption key
    key_seed = data[BTF_KEY_OFFSET:BTF_KEY_OFFSET + BTF_KEY_SIZE]
    print(f"\n  Encryption key @ 0x{BTF_KEY_OFFSET:04X}:")
    print(f"    Seed: {key_seed.hex()}")
    print(f"    All zero: {'YES (unencrypted)' if key_seed == bytes(16) else 'NO (encrypted)'}")

    # Check for encryption marker at 0x800
    if len(data) > 0x800:
        sample = data[0x800:0x810]
        print(f"\n  First encrypted bytes @ 0x800: {sample.hex()}")

    print(f"\n  BTF file appears valid for model \"{model_str}\"")


def probe_bootloader(port: str, baud: int = DEFAULT_BAUD, verbose: bool = False,
                     upgrade: bool = False):
    """Probe if radio is in bootloader mode.

    If upgrade=True, first try the PROGRAMBT9000U + UPDATE handshake to
    enter bootloader from normal firmware, then probe with rapid flooding.
    """
    uploader = FirmwareUploader(port, baud, verbose=verbose)
    uploader.open()
    try:
        print(f"Probing bootloader on {port}...")

        if upgrade:
            print("\n--- Upgrade mode: entering bootloader via firmware ---")
            if not uploader.handshake():
                print("Firmware handshake failed. Is the radio on and connected?")
                return
            print("Entered bootloader via upgrade handshake.\n")

        # In upgrade mode, flood probes to catch bootloader's UART window.
        # In direct mode, a single probe suffices.
        if uploader.probe(flood=upgrade):
            print("Radio is in bootloader mode!")
            if uploader.version_handshake():
                print("Bootloader version confirmed: V3")
            return

        if upgrade:
            print("No bootloader response after upgrade handshake.")
            print("The radio may need a power cycle. Try holding the bottom")
            print("two side buttons while powering on for direct bootloader entry.")
        else:
            print("No bootloader response. Try --upgrade or hold "
                  "side buttons while powering on.")
    finally:
        uploader.close()


def main():
    parser = argparse.ArgumentParser(
        description="Radtel RT-950 Pro firmware upload tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Upload firmware (normal mode - radio sends UPDATE command):
    %(prog)s upload /dev/ttyUSB0 RT-950_V027.BTF

  Upload firmware (direct bootloader - hold side buttons at power-on):
    %(prog)s upload /dev/ttyUSB0 RT-950_V027.BTF --ptt

  Upload and monitor debug output after reboot:
    %(prog)s upload /dev/ttyUSB0 firmware.BTF --ptt -m

  Monitor debug UART output (radio already running):
    %(prog)s monitor /dev/ttyUSB0

  Validate a BTF file without uploading:
    %(prog)s verify firmware.BTF

  Check if radio is in bootloader mode (direct):
    %(prog)s probe /dev/ttyUSB0

  Enter bootloader via upgrade handshake and probe:
    %(prog)s probe /dev/ttyUSB0 --upgrade
""")

    subparsers = parser.add_subparsers(dest="command", help="Command")

    # upload
    p_upload = subparsers.add_parser("upload", help="Upload firmware to radio")
    p_upload.add_argument("port", help="Serial port (e.g., /dev/ttyUSB0)")
    p_upload.add_argument("btf", help="BTF firmware file to upload")
    p_upload.add_argument("--ptt", action="store_true",
                          help="Direct bootloader mode: radio already in bootloader "
                               "(hold bottom two side buttons while powering on)")
    p_upload.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                          help=f"Baud rate (default: {DEFAULT_BAUD})")
    p_upload.add_argument("-v", "--verbose", action="store_true",
                          help="Verbose output (show raw bytes)")
    p_upload.add_argument("-m", "--monitor", action="store_true",
                          help="Monitor debug UART output after upload completes")

    # verify
    p_verify = subparsers.add_parser("verify", help="Validate BTF file without uploading")
    p_verify.add_argument("btf", help="BTF firmware file to validate")

    # probe
    p_probe = subparsers.add_parser("probe", help="Check if radio is in bootloader mode")
    p_probe.add_argument("port", help="Serial port (e.g., /dev/ttyUSB0)")
    p_probe.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                          help=f"Baud rate (default: {DEFAULT_BAUD})")
    p_probe.add_argument("--upgrade", action="store_true",
                          help="Enter bootloader via upgrade handshake (radio must be on)")
    p_probe.add_argument("-v", "--verbose", action="store_true",
                          help="Verbose output")

    # monitor
    p_monitor = subparsers.add_parser("monitor",
                                       help="Monitor debug UART output from radio")
    p_monitor.add_argument("port", help="Serial port (e.g., /dev/ttyUSB0)")
    p_monitor.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                            help=f"Baud rate (default: {DEFAULT_BAUD})")
    p_monitor.add_argument("--no-timestamp", action="store_true",
                            help="Disable elapsed-time timestamps")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        sys.exit(1)

    if args.command == "upload":
        uploader = FirmwareUploader(
            args.port, args.baud, verbose=args.verbose)
        success = uploader.upload(args.btf, ptt_mode=args.ptt)
        if success and args.monitor:
            print()
            time.sleep(0.5)  # brief pause for radio reboot
            serial_monitor(args.port, args.baud)
        sys.exit(0 if success else 1)

    elif args.command == "verify":
        validate_btf(args.btf)

    elif args.command == "probe":
        probe_bootloader(args.port, args.baud, verbose=args.verbose,
                         upgrade=args.upgrade)

    elif args.command == "monitor":
        serial_monitor(args.port, args.baud,
                       timestamp=not args.no_timestamp)


if __name__ == "__main__":
    main()
