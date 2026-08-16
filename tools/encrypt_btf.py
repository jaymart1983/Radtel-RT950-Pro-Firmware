#!/usr/bin/env python3
"""
encrypt_btf.py - BTF firmware encryption/decryption tool for the Radtel RT-950 Pro

Encrypts raw firmware binaries into .BTF update format, or decrypts .BTF files
back to raw binary for analysis.

Encryption algorithm (reverse-engineered from OEM firmware analysis):
  1. A 16-byte encryption key is expanded to 128 bytes:
     - Block 0 = original 16-byte key
     - Blocks 1-7: per-byte bit rotation from previous block
       * Bytes 0-7: left-rotate by 1 bit
       * Bytes 8-15: right-rotate by 1 bit
  2. The first 0x800 bytes (vector table + bootloader signature) are plaintext.
  3. From offset 0x800 onwards, each byte is XORed with expanded_key[offset % 128].
  4. Bytes with value 0x00 or 0xFF are preserved (not encrypted).

Usage:
    python3 encrypt_btf.py firmware.bin output.BTF --key <hex_key>
    python3 encrypt_btf.py --decrypt firmware.BTF output.bin --key <hex_key>
    python3 encrypt_btf.py --decrypt firmware.BTF output.bin --auto
"""

import argparse
import sys
import os


# Offset where encryption begins
ENCRYPT_START = 0x800

# Offset where the 16-byte key is stored in BTF files
KEY_OFFSET = 0x400
KEY_LENGTH = 16

# Block size for BTF upload protocol
BLOCK_SIZE = 0x400  # 1024 bytes

# Expanded key length
EXPANDED_KEY_LEN = 128

# Known keys for reference/validation
KNOWN_KEYS = {
    "71CAEFACD047EF83EFD2141A3512A638": "V0.15",
    "DC24DEF7CF4F3FED91BCC88BB0613A51": "V0.18",
    "3C0F640BB03230BB97AF8029C4AD794D": "V0.21",
    "7E807B1761A4EBC6FC3A8DD33752F305": "V0.27",
}


def rol8(byte: int) -> int:
    """Left-rotate a byte by 1 bit: bit 7 wraps to bit 0."""
    return ((byte << 1) | (byte >> 7)) & 0xFF


def ror8(byte: int) -> int:
    """Right-rotate a byte by 1 bit: bit 0 wraps to bit 7."""
    return ((byte >> 1) | (byte << 7)) & 0xFF


def expand_key(key_bytes: bytes) -> bytes:
    """
    Expand a 16-byte key to 128 bytes using per-byte bit rotation.

    Algorithm (verified via roundtrip testing against OEM .BTF files):
      - Block 0 = original 16-byte key
      - For blocks 1-7, each byte is individually bit-rotated from the
        corresponding byte in the PREVIOUS block:
          * First 8 bytes: left-rotate each byte by 1 bit
          * Last 8 bytes: right-rotate each byte by 1 bit
    """
    assert len(key_bytes) == 16, "Key must be exactly 16 bytes"

    blocks = [key_bytes]

    for _ in range(7):
        prev = blocks[-1]
        new_block = bytes(
            rol8(prev[i]) if i < 8 else ror8(prev[i])
            for i in range(16)
        )
        blocks.append(new_block)

    return b''.join(blocks)


def encrypt_decrypt(data: bytearray, expanded_key: bytes) -> bytearray:
    """
    XOR data from ENCRYPT_START onwards with the expanded key.
    Bytes with value 0x00 or 0xFF are preserved unchanged.
    """
    result = bytearray(data)
    key_len = len(expanded_key)

    for i in range(ENCRYPT_START, len(result)):
        if result[i] == 0x00 or result[i] == 0xFF:
            continue
        key_idx = i % key_len
        xored = result[i] ^ expanded_key[key_idx]
        if xored == 0x00 or xored == 0xFF:
            continue  # preserve: XOR result must not be 0x00 or 0xFF
        result[i] = xored

    return result


def extract_key(data: bytes) -> bytes:
    """Extract the 16-byte encryption key from offset 0x400 in a BTF file."""
    if len(data) < KEY_OFFSET + KEY_LENGTH:
        print(f"Error: File too small ({len(data)} bytes) to contain key at "
              f"offset 0x{KEY_OFFSET:X}", file=sys.stderr)
        sys.exit(1)
    return data[KEY_OFFSET:KEY_OFFSET + KEY_LENGTH]


def pad_to_block(firmware: bytearray) -> bytearray:
    """Pad the firmware up to a whole number of 1024-byte upload blocks.

    The final block of an upload does not land correctly on the radio. Flashing
    a 9168-byte image gave byte-perfect content for the first 8 KB and garbage
    in the tail -- confirmed by having the firmware checksum its own flash a
    kilobyte at a time and comparing against the .bin:

        BLK 0..7  match exactly
        BLK 8     53907 expected, 93901 actual   <- last, partial block

    That tail is where late .rodata lives, so the symptoms were font glyphs
    rendering as garbage and string constants reading as random bytes, while
    everything earlier in the image worked perfectly.

    Padding with 0xFF -- erased-flash value -- makes every block full, so no
    partial-block case arises at all.
    """
    rem = len(firmware) % BLOCK_SIZE
    if rem:
        firmware = bytearray(firmware) + bytearray(b"\xFF" * (BLOCK_SIZE - rem))

    # Then add one whole sacrificial block.
    #
    # Padding to a block boundary was not enough: with the image at exactly ten
    # full blocks the final one STILL did not land (BLK 8 read 106141 against
    # 53907 expected). The bootloader does not commit the last block it is
    # given. Appending a block of erased-value padding means the discarded
    # block contains nothing the firmware needs.
    firmware = bytearray(firmware) + bytearray(b"\xFF" * BLOCK_SIZE)
    return firmware


def build_btf(firmware: bytearray, key_bytes: bytes, expanded_key: bytes,
              key_block: bytes = None) -> bytearray:
    """
    Build a .BTF file from a raw firmware binary.

    BTF layout (1024-byte blocks for bootloader upload):
      Block 0 [0x000:0x400] - Plaintext: vectors, model signature
      Block 1 [0x400:0x800] - Key block: 16-byte key + padding (not flashed)
      Block 2+ [0x800:...]  - Encrypted firmware data (from binary[0x400:])

    The key block is inserted between binary[0:0x400] and binary[0x400:],
    so BTF size = firmware size + 1024.

    If key_block is provided (1024 bytes from original BTF), it is used as-is
    for exact round-trip. Otherwise a new key block is generated.
    """
    # Block 0: first 1K of firmware (plaintext, includes vectors + model sig)
    block0 = bytearray(firmware[:BLOCK_SIZE])
    if len(block0) < BLOCK_SIZE:
        block0.extend(bytes(BLOCK_SIZE - len(block0)))

    if key_block is not None:
        # Use preserved key block for exact round-trip
        kb = bytearray(key_block[:BLOCK_SIZE])
    else:
        # Generate key block: 16-byte seed + expanded key
        kb = bytearray(BLOCK_SIZE)
        kb[:KEY_LENGTH] = key_bytes
        exp_len = min(len(expanded_key), BLOCK_SIZE - KEY_LENGTH)
        kb[KEY_LENGTH:KEY_LENGTH + exp_len] = expanded_key[:exp_len]

    # Remaining firmware data (will be encrypted)
    rest = bytearray(firmware[BLOCK_SIZE:])

    # Assemble BTF: block0 + kb + rest
    btf = block0 + kb + rest

    # Encrypt from offset 0x800 onwards
    btf = encrypt_decrypt(btf, expanded_key)
    return btf


def extract_firmware(btf: bytearray, expanded_key: bytes) -> tuple:
    """
    Extract raw firmware binary from a .BTF file.

    Reverses the BTF layout: decrypts, then removes the 1024-byte key block
    to reconstruct the original firmware binary.

    Returns (firmware, key_block) where key_block is the original 1024 bytes
    at offset 0x400-0x800 (preserved for exact round-trip re-encryption).
    """
    # Save original key block before decryption
    key_block = bytes(btf[BLOCK_SIZE:ENCRYPT_START])

    # Decrypt from offset 0x800 onwards
    decrypted = encrypt_decrypt(btf, expanded_key)

    # Remove key block: firmware = btf[0:0x400] + btf[0x800:]
    firmware = decrypted[:BLOCK_SIZE] + decrypted[ENCRYPT_START:]
    return firmware, key_block


def main():
    parser = argparse.ArgumentParser(
        description="RT-950 Pro BTF firmware encryption/decryption tool"
    )
    parser.add_argument("input", help="Input file (.bin for encrypt, .BTF for decrypt)")
    parser.add_argument("output", help="Output file (.BTF for encrypt, .bin for decrypt)")
    parser.add_argument(
        "--key",
        help="Encryption key as 32 hex characters (16 bytes)."
    )
    parser.add_argument(
        "--auto",
        action="store_true",
        help="Auto-extract key from BTF header at offset 0x400 (decrypt only)."
    )
    parser.add_argument(
        "--decrypt",
        action="store_true",
        help="Decrypt a .BTF file to raw binary (same XOR operation)."
    )
    parser.add_argument(
        "--keyblock",
        help="Key block file (1024 bytes). Decrypt: save to this path. "
             "Encrypt: load from this path for exact round-trip."
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Print details about the operation."
    )

    args = parser.parse_args()

    # Read input first (needed for --auto)
    if not os.path.exists(args.input):
        print(f"Error: Input file not found: {args.input}", file=sys.stderr)
        sys.exit(1)

    with open(args.input, "rb") as f:
        data = bytearray(f.read())

    # Encrypt direction only: pad the raw image so every upload block is full.
    # See pad_to_block() -- a partial final block does not land on the radio.
    if not args.decrypt:
        before = len(data)
        data = pad_to_block(data)
        if len(data) != before:
            print(f"Padded {before} -> {len(data)} bytes "
                  f"({len(data)//BLOCK_SIZE} whole {BLOCK_SIZE}-byte blocks)")

    # Resolve key
    if args.auto:
        if not args.decrypt:
            print("Error: --auto can only be used with --decrypt", file=sys.stderr)
            sys.exit(1)
        key_bytes = extract_key(data)
    elif args.key:
        try:
            key_bytes = bytes.fromhex(args.key)
        except ValueError:
            print(f"Error: Invalid hex key: {args.key}", file=sys.stderr)
            sys.exit(1)
        if len(key_bytes) != KEY_LENGTH:
            print(f"Error: Key must be {KEY_LENGTH} bytes (32 hex chars), "
                  f"got {len(key_bytes)}", file=sys.stderr)
            sys.exit(1)
    else:
        print("Error: Provide --key <hex> or --auto (with --decrypt)", file=sys.stderr)
        sys.exit(1)

    key_hex = key_bytes.hex().upper()
    version = KNOWN_KEYS.get(key_hex, None)

    # Expand key
    expanded_key = expand_key(key_bytes)

    if args.verbose:
        print(f"Input:        {args.input}")
        print(f"Output:       {args.output}")
        key_label = f"{key_hex} ({version})" if version else key_hex
        print(f"Key:          {key_label}")
        print(f"Expanded key: {expanded_key[:32].hex().upper()}...")
        print(f"Operation:    {'Decrypt' if args.decrypt else 'Encrypt'}")
        print(f"Input size:   {len(data)} bytes (0x{len(data):X})")
    elif version:
        print(f"Key: {key_hex} ({version})")

    if args.decrypt:
        # Decrypt BTF to raw firmware binary
        if len(data) < ENCRYPT_START:
            print("Warning: BTF file smaller than expected header size.",
                  file=sys.stderr)
        result, orig_keyblock = extract_firmware(data, expanded_key)

        # Save key block for exact round-trip re-encryption
        kb_path = args.keyblock or (args.output + ".keyblock")
        with open(kb_path, "wb") as f:
            f.write(orig_keyblock)
        if args.verbose:
            print(f"Key block:    saved to {kb_path} ({len(orig_keyblock)} bytes)")

        # Validate vector table
        if len(result) >= 8:
            sp = int.from_bytes(result[0:4], "little")
            pc = int.from_bytes(result[4:8], "little")
            if args.verbose:
                print(f"Vector SP:    0x{sp:08X}")
                print(f"Vector PC:    0x{pc:08X}")
                print(f"Output size:  {len(result)} bytes (BTF - {BLOCK_SIZE} key block)")
            if not (0x20000000 <= sp <= 0x20018000):
                print(f"Warning: Initial SP 0x{sp:08X} looks invalid for AT32F403A",
                      file=sys.stderr)
            if not (0x08000000 <= pc <= 0x08100000):
                print(f"Warning: Reset vector 0x{pc:08X} looks invalid for AT32F403A",
                      file=sys.stderr)
    else:
        # Encrypt raw firmware binary to BTF
        if len(data) < BLOCK_SIZE:
            print(f"Warning: Firmware smaller than {BLOCK_SIZE} bytes; "
                  "block 0 will be zero-padded.", file=sys.stderr)

        # Load preserved key block for exact round-trip if available
        saved_kb = None
        kb_path = args.keyblock or (args.input + ".keyblock")
        if os.path.exists(kb_path):
            with open(kb_path, "rb") as f:
                saved_kb = f.read()
            if len(saved_kb) != BLOCK_SIZE:
                print(f"Warning: Key block file is {len(saved_kb)} bytes, "
                      f"expected {BLOCK_SIZE}. Ignoring.", file=sys.stderr)
                saved_kb = None
            elif args.verbose:
                print(f"Key block:    loaded from {kb_path}")

        result = build_btf(data, key_bytes, expanded_key, key_block=saved_kb)

        # Verify key block was embedded
        embedded_key = result[KEY_OFFSET:KEY_OFFSET + KEY_LENGTH]
        if embedded_key != key_bytes:
            print("Error: Key block verification failed!", file=sys.stderr)
            sys.exit(1)

        if args.verbose:
            model = result[0x3E0:0x3EC]
            print(f"Model sig:    {model}")
            print(f"Key at 0x400: {embedded_key.hex().upper()}")
            print(f"Output size:  {len(result)} bytes (firmware + {BLOCK_SIZE} key block)")

    # Write output
    with open(args.output, "wb") as f:
        f.write(result)

    op = "Decrypted" if args.decrypt else "Encrypted"
    print(f"{op} {len(result)} bytes -> {args.output}")


if __name__ == "__main__":
    main()
