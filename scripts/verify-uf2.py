#!/usr/bin/env python3
"""Validate the actual RP2040 UF2 output, not just the presence of a file."""
import hashlib
from pathlib import Path
import struct
import sys


def verify(path):
    data = path.read_bytes()
    if not data or len(data) % 512:
        raise ValueError("UF2 must contain complete 512-byte blocks")
    count = len(data) // 512
    seen = set()
    ranges = []
    for offset in range(0, len(data), 512):
        block = data[offset:offset + 512]
        magic0, magic1, flags, address, size, number, total, family = struct.unpack_from("<8I", block)
        if (magic0, magic1, struct.unpack_from("<I", block, 508)[0]) != (0x0A324655, 0x9E5D5157, 0x0AB16F30):
            raise ValueError("Invalid UF2 block magic")
        if not flags & 0x2000 or flags & 1 or family != 0xE48BFF56:
            raise ValueError("Expected RP2040 flash blocks with a family ID")
        if not 0 < size <= 476 or address < 0x10000000 or address + size > 0x10200000:
            raise ValueError("Payload lies outside the board's 2 MiB flash")
        if total != count or number in seen or not 0 <= number < count:
            raise ValueError("Inconsistent UF2 block numbering")
        seen.add(number)
        ranges.append((address, address + size))
    ranges.sort()
    if ranges[0][0] != 0x10000000 or any(b[0] < a[1] for a, b in zip(ranges, ranges[1:])):
        raise ValueError("Missing flash start or overlapping payload blocks")
    print(f"Verified {path.name}: {count} RP2040 blocks, {len(data)} bytes")
    print(f"SHA256 {hashlib.sha256(data).hexdigest()}")


if __name__ == "__main__":
    try:
        verify(Path(sys.argv[1]))
    except (IndexError, OSError, ValueError) as error:
        sys.exit(f"UF2 validation failed: {error}")
