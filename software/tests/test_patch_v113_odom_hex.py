from __future__ import annotations

import hashlib
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

import patch_v113_odom_hex as patcher
from patch_factory_odom_hex import decode_hex


def _record(address: int, record_type: int, data: bytes = b"") -> str:
    payload = bytes([len(data)]) + address.to_bytes(2, "big") + bytes([record_type]) + data
    checksum = (-sum(payload)) & 0xFF
    return ":" + (payload + bytes([checksum])).hex().upper() + "\n"


def _fixture_hex() -> bytes:
    memory = bytearray(b"\xFF" * 256)
    timer = (90_000_000).to_bytes(8, "little") + (140_000_000).to_bytes(8, "little")
    memory[0x40 : 0x40 + len(timer)] = timer
    memory[0x50 : 0x54] = (40_000_000).to_bytes(4, "little")
    cursor = 0x80
    for text in patcher.REQUIRED_FIRMWARE_STRINGS:
        memory[cursor : cursor + len(text)] = text
        cursor += len(text) + 1
    lines = [_record(0, 4, (0x0800).to_bytes(2, "big"))]
    for offset in range(0, len(memory), 16):
        lines.append(_record(offset, 0, bytes(memory[offset : offset + 16])))
    lines.append(_record(0, 1))
    return "".join(lines).encode("ascii")


@pytest.mark.parametrize(
    "hz,expected_ns",
    [
        (20, 50_000_000),
        (25, 40_000_000),
        (30, 33_333_333),
        (50, 20_000_000),
    ],
)
def test_create_variant_changes_only_unique_timer_word(monkeypatch, hz, expected_ns):
    source = _fixture_hex()
    monkeypatch.setattr(patcher, "EXPECTED_INPUT_SHA256", hashlib.sha256(source).hexdigest())

    result, address, changed_byte_count = patcher.create_variant(source, hz)
    before, _ = decode_hex(source.decode("ascii").splitlines(keepends=True))
    after, records = decode_hex(result.decode("ascii").splitlines(keepends=True))

    assert address == 0x08000040
    assert before.keys() == after.keys()
    assert [a for a in before if before[a] != after[a]] == sorted(
        a for a in range(address, address + 8) if before[a] != after[a]
    )
    assert bytes(after[address + i] for i in range(8)) == expected_ns.to_bytes(8, "little")
    assert changed_byte_count == sum(before[a] != after[a] for a in before)
    assert len(records) == len(decode_hex(source.decode("ascii").splitlines(keepends=True))[1])


def test_create_variant_rejects_non_pinned_source():
    with pytest.raises(patcher.HexError, match="SHA-256 mismatch"):
        patcher.create_variant(_fixture_hex(), 20)


def test_locate_odom_timer_rejects_duplicate_90ms_values():
    source = _fixture_hex()
    memory, _ = decode_hex(source.decode("ascii").splitlines(keepends=True))
    duplicate_at = max(memory) + 8
    timer = (90_000_000).to_bytes(8, "little") + (140_000_000).to_bytes(8, "little")
    memory.update({duplicate_at + i: byte for i, byte in enumerate(timer)})
    with pytest.raises(patcher.HexError, match="unique 90 ms"):
        patcher.locate_odom_timer(memory)


def test_create_variant_rejects_unsupported_rate(monkeypatch):
    source = _fixture_hex()
    monkeypatch.setattr(patcher, "EXPECTED_INPUT_SHA256", hashlib.sha256(source).hexdigest())
    with pytest.raises(patcher.HexError, match="supported rates"):
        patcher.create_variant(source, 60)


@pytest.mark.parametrize(
    "odom_hz,imu_hz,lidar_ms,expected_odom_ns,expected_imu_ns,expected_lidar_ns",
    [
        (30, 60, None, 33_333_333, 16_666_666, None),
        (25, 50, None, 40_000_000, 20_000_000, None),
        (30, 30, 70, 33_333_333, 33_333_333, 70_000_000),
        (30, 30, 140, 33_333_333, 33_333_333, 140_000_000),
    ],
)
def test_create_combined_variant_changes_only_guarded_timer_words(
    monkeypatch, odom_hz, imu_hz, lidar_ms, expected_odom_ns, expected_imu_ns, expected_lidar_ns
):
    source = _fixture_hex()
    monkeypatch.setattr(patcher, "EXPECTED_INPUT_SHA256", hashlib.sha256(source).hexdigest())
    monkeypatch.setattr(patcher, "IMU_TIMER_ADDRESS", 0x08000050)

    result, address, changed_byte_count = patcher.create_variant(source, odom_hz, imu_hz, lidar_ms)
    before, _ = decode_hex(source.decode("ascii").splitlines(keepends=True))
    after, _ = decode_hex(result.decode("ascii").splitlines(keepends=True))
    changed = {a for a in before if before[a] != after[a]}

    assert address == 0x08000040
    allowed = set(range(0x08000040, 0x08000048)) | set(range(0x08000050, 0x08000054))
    if expected_lidar_ns is not None:
        allowed |= set(range(0x08000048, 0x08000050))
    assert changed <= allowed
    assert bytes(after[0x08000040 + i] for i in range(8)) == expected_odom_ns.to_bytes(8, "little")
    assert bytes(after[0x08000050 + i] for i in range(4)) == expected_imu_ns.to_bytes(4, "little")
    if expected_lidar_ns is not None:
        assert bytes(after[0x08000048 + i] for i in range(8)) == expected_lidar_ns.to_bytes(8, "little")
    assert changed_byte_count == len(changed)


def test_combined_variant_rejects_duplicate_imu_timer(monkeypatch):
    source = _fixture_hex()
    monkeypatch.setattr(patcher, "EXPECTED_INPUT_SHA256", hashlib.sha256(source).hexdigest())
    monkeypatch.setattr(patcher, "IMU_TIMER_ADDRESS", 0x08000050)
    memory, _ = decode_hex(source.decode("ascii").splitlines(keepends=True))
    duplicate_at = max(memory) + 8
    memory.update({duplicate_at + i: b for i, b in enumerate((40_000_000).to_bytes(4, "little"))})
    # Rebuild a valid HEX image with a duplicate 40 ms literal.
    rows = [_record(0, 4, (0x0800).to_bytes(2, "big"))]
    end = max(memory) - 0x08000000 + 1
    raw = bytearray(b"\xFF" * end)
    for address, byte in memory.items():
        raw[address - 0x08000000] = byte
    for offset in range(0, len(raw), 16):
        rows.append(_record(offset, 0, raw[offset : offset + 16]))
    rows.append(_record(0, 1))
    duplicate_source = "".join(rows).encode("ascii")
    monkeypatch.setattr(patcher, "EXPECTED_INPUT_SHA256", hashlib.sha256(duplicate_source).hexdigest())

    with pytest.raises(patcher.HexError, match="inferred 40 ms IMU timer"):
        patcher.create_variant(duplicate_source, 30, 60)


def test_lidar_combined_variant_rejects_duplicate_140ms_timer(monkeypatch):
    source = _fixture_hex()
    lines = source.decode("ascii").splitlines(keepends=True)
    lines.insert(-1, _record(0x100, 0, (140_000_000).to_bytes(8, "little")))
    duplicate_source = "".join(lines).encode("ascii")
    monkeypatch.setattr(patcher, "EXPECTED_INPUT_SHA256", hashlib.sha256(duplicate_source).hexdigest())
    monkeypatch.setattr(patcher, "IMU_TIMER_ADDRESS", 0x08000050)

    with pytest.raises(patcher.HexError, match="unique adjacent 140 ms lidar timer"):
        patcher.create_variant(duplicate_source, 30, 30, 70)
