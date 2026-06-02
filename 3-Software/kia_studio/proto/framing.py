"""Byte-level framing primitives: COBS + CRC32.

Mirror of firmware components/common/cobs.c and crc32.c.
CRC32 is the standard reflected CRC-32 (poly 0xEDB88320), == firmware CRC32-ISO == zlib.crc32.
"""
from __future__ import annotations

import zlib

DELIMITER = 0x00


def crc32(data: bytes) -> int:
    """CRC32-ISO over `data`, returned as unsigned 32-bit."""
    return zlib.crc32(data) & 0xFFFFFFFF


def cobs_encode(data: bytes) -> bytes:
    """COBS-encode `data` and append the 0x00 frame delimiter."""
    out = bytearray([0])      # placeholder for first code byte
    code_idx = 0
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
    out[code_idx] = code
    out.append(DELIMITER)
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Decode a COBS frame that includes its trailing 0x00 delimiter.

    Returns b"" on malformed input (missing/early delimiter, zero code byte).
    """
    if len(data) < 2 or data[-1] != DELIMITER:
        return b""
    out = bytearray()
    i = 0
    end = len(data) - 1
    while i < end:
        code = data[i]
        if code == 0:
            return b""
        i += 1
        for _ in range(code - 1):
            if i >= end:
                return bytes(out)
            out.append(data[i])
            i += 1
        if code != 0xFF and i < end:
            out.append(0)
    return bytes(out)
