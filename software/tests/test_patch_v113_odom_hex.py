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
    cursor = 0x80
    for text in patcher.REQUIRED_FIRMWARE_STRINGS:
        memory[cursor : cursor + len(text)] = text
        cursor += len(text) + 1
    lines = [_record(0, 4, (0x0800).to_bytes(2, "big"))]
    for offset in range(0, len(memory), 16):
        lines.append(_record(offset, 0, bytes(memory[offset : offset + 16])))
    lines.append(_record(0, 1))
    return "".join(lines).encode("ascii")


@pytest.mark.parametrize("hz,expected_ns", [(20, 50_000_000), (50, 20_000_000)])
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
        patcher.create_variant(source, 25)
