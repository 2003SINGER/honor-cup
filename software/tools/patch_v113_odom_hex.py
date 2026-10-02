#!/usr/bin/env python3
"""Create offline odometry-rate variants of the archived M3 Pro V1.1.3 HEX.

This tool is intentionally pinned to one known integrated factory image. It
does not connect to a robot, use a programmer, or overwrite the source image.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import sys

from patch_factory_odom_hex import HexError, checksum, decode_hex


EXPECTED_INPUT_SHA256 = "e0db7805de6691b6b5d83c8b348f1477553c9547524cd69d6fc7673c4fcc1cc0"
OLD_TIMER_NS = 90_000_000
ADJACENT_LIDAR_TIMER_NS = 140_000_000
TIMER_WORD_BYTES = 8
ARCHIVE_DIR = Path(__file__).resolve().parents[2] / "field_data/firmware_archive/2026-10-02"
DEFAULT_INPUT = ARCHIVE_DIR / "microROS_STM32-FW_V1.1.3.hex"

REQUIRED_FIRMWARE_STRINGS = (
    b"Start YB_Node",
    b"YB_Node",
    b"odom_raw",
    b"imu/data_raw",
    b"scan0",
    b"scan1",
    b"battery",
    b"cmd_vel",
    b"arm6_joints",
)


def matches_at(memory: dict[int, int], address: int, value: bytes) -> bool:
    return all(memory.get(address + i) == byte for i, byte in enumerate(value))


def find_all(memory: dict[int, int], value: bytes) -> list[int]:
    return [address for address in memory if matches_at(memory, address, value)]


def image_contains(memory: dict[int, int], value: bytes) -> bool:
    return bool(find_all(memory, value))


def locate_odom_timer(memory: dict[int, int]) -> int:
    old = OLD_TIMER_NS.to_bytes(TIMER_WORD_BYTES, "little")
    adjacent = ADJACENT_LIDAR_TIMER_NS.to_bytes(TIMER_WORD_BYTES, "little")
    locations = find_all(memory, old)
    if len(locations) != 1:
        raise HexError(f"expected one unique 90 ms timer word, found {[hex(x) for x in locations]}")
    address = locations[0]
    if not matches_at(memory, address + TIMER_WORD_BYTES, adjacent):
        raise HexError(
            f"90 ms word at 0x{address:08X} is not followed by the expected 140 ms timer word"
        )
    missing = [s.decode("ascii") for s in REQUIRED_FIRMWARE_STRINGS if not image_contains(memory, s)]
    if missing:
        raise HexError(f"image is missing expected integrated-firmware strings: {missing}")
    return address


def patch_timer_lines(
    lines: list[str], records: list[tuple[int, int, int]], address: int, new_value: bytes
) -> list[str]:
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
                byte_address = base + offset + data_offset
                if address <= byte_address < address + len(new_value):
                    if byte_address in locations:
                        raise HexError(f"target address 0x{byte_address:08X} occurs more than once")
                    locations[byte_address] = (line_index, data_offset)

    expected = set(range(address, address + len(new_value)))
    if locations.keys() != expected:
        raise HexError(f"timer bytes are not fully represented in HEX data records: {sorted(expected - locations.keys())}")

    patched = list(lines)
    changed_lines: set[int] = set()
    for index, new_byte in enumerate(new_value):
        line_index, byte_index = locations[address + index]
        original = patched[line_index]
        content_end = len(original.rstrip("\r\n"))
        hex_start = 9 + 2 * byte_index
        patched[line_index] = (
            original[:hex_start]
            + f"{new_byte:02X}"
            + original[hex_start + 2 : content_end]
            + original[content_end:]
        )
        changed_lines.add(line_index)

    for line_index in changed_lines:
        original = patched[line_index]
        ending = "\r\n" if original.endswith("\r\n") else "\n" if original.endswith("\n") else ""
        payload = bytes.fromhex(original.strip()[1:])
        patched[line_index] = ":" + (payload[:-1] + bytes([checksum(payload[:-1])])).hex().upper() + ending
    return patched


def create_variant(source_bytes: bytes, hz: int) -> tuple[bytes, int, int]:
    if hz not in (20, 50):
        raise HexError("supported rates are 20 or 50 Hz")
    source_sha = hashlib.sha256(source_bytes).hexdigest()
    if source_sha != EXPECTED_INPUT_SHA256:
        raise HexError(f"V1.1.3 source SHA-256 mismatch: got {source_sha}")
    try:
        source_text = source_bytes.decode("ascii")
    except UnicodeDecodeError as exc:
        raise HexError("input is not ASCII Intel HEX") from exc
    lines = source_text.splitlines(keepends=True)
    source_image, records = decode_hex(lines)
    address = locate_odom_timer(source_image)
    old_word = OLD_TIMER_NS.to_bytes(TIMER_WORD_BYTES, "little")
    new_ns = 1_000_000_000 // hz
    new_word = new_ns.to_bytes(TIMER_WORD_BYTES, "little")
    if not matches_at(source_image, address, old_word):
        raise HexError("timer source word changed after location")

    output_lines = patch_timer_lines(lines, records, address, new_word)
    output_image, _ = decode_hex(output_lines)
    if source_image.keys() != output_image.keys():
        raise HexError("patched HEX changed the mapped address set")
    diffs = {a for a in source_image if source_image[a] != output_image[a]}
    allowed = set(range(address, address + TIMER_WORD_BYTES))
    if not diffs or not diffs <= allowed:
        raise HexError(f"unexpected image changes outside timer word: {[hex(a) for a in sorted(diffs)]}")
    if not matches_at(output_image, address, new_word):
        raise HexError("patched timer does not equal the requested interval")
    return "".join(output_lines).encode("ascii"), address, len(diffs)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--hz", type=int, choices=(20, 50), required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    source_bytes = args.input.read_bytes()
    output = args.output or ARCHIVE_DIR / f"microROS_STM32-FW_V1.1.3_odom_{args.hz}Hz.hex"
    if args.input.resolve() == output.resolve():
        raise HexError("refusing to overwrite the original input")
    if output.exists():
        raise HexError(f"refusing to overwrite existing output: {output}")

    candidate, address, changed_bytes = create_variant(source_bytes, args.hz)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(candidate)
    interval_ms = 1000 // args.hz
    print(f"Source:      {args.input}")
    print(f"Source SHA:  {hashlib.sha256(source_bytes).hexdigest()}")
    print(f"Candidate:   {output}")
    print(f"Output SHA:  {hashlib.sha256(candidate).hexdigest()}")
    print(f"Timer:       0x{address:08X}, 90 ms -> {interval_ms} ms ({args.hz} Hz)")
    print(f"Verified:    checksums valid; same address map; {changed_bytes} changed byte(s), timer word only")
    print("Status:      offline candidate only; not programmed or runtime-validated")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, HexError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(2)
