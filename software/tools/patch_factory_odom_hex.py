#!/usr/bin/env python3
"""Patch only the factory firmware odom timer constant in an Intel HEX image.

This is an offline file transformation. It never opens a serial port or flashes
the board. The source image is pinned by SHA-256 and is never overwritten.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import sys


EXPECTED_INPUT_SHA256 = "1e27ea266e772d075b633549091f64befa5f5ae4f446224878ea50f8e788d2a7"
ODOM_TIMER_ADDRESS = 0x0800B790
OLD_TIMER_NS = 90_000_000
NEW_TIMER_NS = 20_000_000
OLD_BYTES = OLD_TIMER_NS.to_bytes(8, "little")
NEW_BYTES = NEW_TIMER_NS.to_bytes(8, "little")
DEFAULT_INPUT = Path.home() / "Downloads/microROS_STM32-FW_V1.1.2.hex"
DEFAULT_OUTPUT = Path(__file__).resolve().parents[2] / "field_data/microROS_STM32-FW_V1.1.2-odom20ms.hex"


class HexError(ValueError):
    pass


def decode_hex(lines: list[str]) -> tuple[dict[int, int], list[tuple[int, int, int]]]:
    """Return address->byte and (line index, address, record type) metadata."""
    memory: dict[int, int] = {}
    records: list[tuple[int, int, int]] = []
    base = 0
    eof_seen = False
    for line_index, raw in enumerate(lines):
        line = raw.strip()
        if not line.startswith(":"):
            raise HexError(f"line {line_index + 1}: missing ':' record marker")
        try:
            payload = bytes.fromhex(line[1:])
        except ValueError as exc:
            raise HexError(f"line {line_index + 1}: invalid hexadecimal record") from exc
        if len(payload) < 5 or (len(payload) != payload[0] + 5):
            raise HexError(f"line {line_index + 1}: inconsistent record length")
        if sum(payload) & 0xFF:
            raise HexError(f"line {line_index + 1}: checksum mismatch")
        count = payload[0]
        offset = int.from_bytes(payload[1:3], "big")
        record_type = payload[3]
        data = payload[4 : 4 + count]
        if eof_seen:
            raise HexError(f"line {line_index + 1}: record follows EOF")
        records.append((line_index, offset, record_type))

        if record_type == 0x00:
            absolute = base + offset
            for i, value in enumerate(data):
                address = absolute + i
                if address in memory:
                    raise HexError(f"line {line_index + 1}: overlapping address 0x{address:08X}")
                memory[address] = value
        elif record_type == 0x01:
            if count or offset:
                raise HexError(f"line {line_index + 1}: malformed EOF record")
            eof_seen = True
        elif record_type == 0x02:
            if count != 2 or offset:
                raise HexError(f"line {line_index + 1}: malformed segment address record")
            base = int.from_bytes(data, "big") << 4
        elif record_type == 0x04:
            if count != 2 or offset:
                raise HexError(f"line {line_index + 1}: malformed linear address record")
            base = int.from_bytes(data, "big") << 16
        elif record_type in (0x03, 0x05):
            if count != 4 or offset:
                raise HexError(f"line {line_index + 1}: malformed start address record")
        else:
            raise HexError(f"line {line_index + 1}: unsupported record type 0x{record_type:02X}")

    if not eof_seen:
        raise HexError("missing EOF record")
    return memory, records


def checksum(record_without_checksum: bytes) -> int:
    return (-sum(record_without_checksum)) & 0xFF


def patch_lines(lines: list[str], memory: dict[int, int], records: list[tuple[int, int, int]]) -> list[str]:
    # Resolve every target byte to exactly one HEX data record and byte offset.
    locations: dict[int, tuple[int, int]] = {}
    base = 0
    for line_index, offset, record_type in records:
        payload = bytes.fromhex(lines[line_index].strip()[1:])
        count = payload[0]
        data = payload[4 : 4 + count]
        if record_type == 0x02:
            base = int.from_bytes(data, "big") << 4
        elif record_type == 0x04:
            base = int.from_bytes(data, "big") << 16
        elif record_type == 0x00:
            for data_offset in range(count):
                address = base + offset + data_offset
                if ODOM_TIMER_ADDRESS <= address < ODOM_TIMER_ADDRESS + len(OLD_BYTES):
                    if address in locations:
                        raise HexError(f"target address 0x{address:08X} occurs more than once")
                    locations[address] = (line_index, data_offset)

    expected_addresses = set(range(ODOM_TIMER_ADDRESS, ODOM_TIMER_ADDRESS + len(OLD_BYTES)))
    if set(locations) != expected_addresses:
        missing = sorted(expected_addresses - set(locations))
        raise HexError(f"target bytes are not fully represented in data records: {missing}")

    patched = list(lines)
    affected_lines: set[int] = set()
    for i, new_value in enumerate(NEW_BYTES):
        address = ODOM_TIMER_ADDRESS + i
        line_index, byte_index = locations[address]
        original_line = patched[line_index]
        body_end = len(original_line.rstrip("\r\n"))
        hex_start = 9 + 2 * byte_index  # ':' + count/address/type (8 hex chars)
        patched[line_index] = (
            original_line[:hex_start]
            + f"{new_value:02X}"
            + original_line[hex_start + 2 : body_end]
            + original_line[body_end:]
        )
        affected_lines.add(line_index)

    for line_index in affected_lines:
        line = patched[line_index]
        record_text = line.strip()
        payload = bytes.fromhex(record_text[1:])
        new_payload = payload[:-1] + bytes([checksum(payload[:-1])])
        ending = "\r\n" if line.endswith("\r\n") else "\n" if line.endswith("\n") else ""
        patched[line_index] = ":" + new_payload.hex().upper() + ending
    return patched


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("output", nargs="?", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    source_bytes = args.input.read_bytes()
    source_sha = hashlib.sha256(source_bytes).hexdigest()
    if source_sha != EXPECTED_INPUT_SHA256:
        raise HexError(f"input SHA-256 mismatch: expected {EXPECTED_INPUT_SHA256}, got {source_sha}")
    if args.input.resolve() == args.output.resolve():
        raise HexError("refusing to overwrite the original input")
    if args.output.exists():
        raise HexError(f"refusing to overwrite existing output: {args.output}")

    try:
        source_text = source_bytes.decode("ascii")
    except UnicodeDecodeError as exc:
        raise HexError("input is not ASCII Intel HEX") from exc
    lines = source_text.splitlines(keepends=True)
    source_image, source_records = decode_hex(lines)

    actual_old = bytes(source_image.get(ODOM_TIMER_ADDRESS + i, -1) for i in range(8))
    if actual_old != OLD_BYTES:
        raise HexError(
            f"unexpected bytes at 0x{ODOM_TIMER_ADDRESS:08X}: "
            f"expected {OLD_BYTES.hex(' ')}, got {actual_old.hex(' ')}"
        )
    # Require the exact 8-byte value to occur once in the decoded contiguous image.
    matches = []
    for address in source_image:
        if all(source_image.get(address + i) == OLD_BYTES[i] for i in range(8)):
            matches.append(address)
    if matches != [ODOM_TIMER_ADDRESS]:
        raise HexError(f"expected one unique timer constant at target, found {[hex(x) for x in matches]}")

    output_lines = patch_lines(lines, source_image, source_records)
    output_image, _ = decode_hex(output_lines)
    expected_new = NEW_TIMER_NS.to_bytes(8, "little")
    actual_new = bytes(output_image.get(ODOM_TIMER_ADDRESS + i, -1) for i in range(8))
    if actual_new != expected_new:
        raise HexError(f"patched bytes do not equal expected 20,000,000 ns: {actual_new.hex(' ')}")
    differences = {
        address
        for address in source_image.keys() | output_image.keys()
        if source_image.get(address) != output_image.get(address)
    }
    expected_differences = set(range(ODOM_TIMER_ADDRESS, ODOM_TIMER_ADDRESS + 8))
    if not differences or not differences <= expected_differences:
        raise HexError(f"unexpected image differences outside the 8-byte timer word: {[hex(x) for x in sorted(differences)]}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("".join(output_lines), encoding="ascii", newline="")
    output_sha = hashlib.sha256(args.output.read_bytes()).hexdigest()
    print(f"Input:      {args.input}")
    print(f"Input SHA:  {source_sha}")
    print(f"Patched:    {args.output}")
    print(f"Output SHA: {output_sha}")
    print(f"Address:    0x{ODOM_TIMER_ADDRESS:08X}: {OLD_BYTES.hex(' ')} -> {expected_new.hex(' ')}")
    print(f"Verified:   all record checksums valid; {len(differences)} changed byte(s), all within the 8-byte timer word")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, HexError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(2)
