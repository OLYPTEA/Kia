"""Offline protocol-core tests — no hardware, no Qt. Run: pytest  (or python test_proto.py)."""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kia_studio.proto import (  # noqa: E402
    OP, RSP, build_frame, parse_frame, decode_status, FrameStreamer,
    cobs_encode, cobs_decode, crc32, KiaClient, LoopbackTransport,
)
from kia_studio.proto.opcodes import Fault  # noqa: E402


# --- COBS ---------------------------------------------------------------
def test_cobs_roundtrip():
    for payload in (b"", b"\x00", b"\x00\x00", b"hello", bytes(range(256)),
                    b"\xff" * 600, b"\x00" * 300):
        enc = cobs_encode(payload)
        assert enc[-1] == 0x00
        assert 0x00 not in enc[:-1]              # no interior zeros
        assert cobs_decode(enc) == payload


def test_cobs_decode_malformed():
    assert cobs_decode(b"") == b""
    assert cobs_decode(b"\x01") == b""           # no delimiter


# --- CRC32 --------------------------------------------------------------
def test_crc32_known_vector():
    assert crc32(b"123456789") == 0xCBF43926     # standard CRC-32 check value


# --- frames -------------------------------------------------------------
def test_frame_roundtrip_set_xyz():
    body = struct.pack("<5f", 150.0, 0.0, 120.0, -30.0, 2.0)
    frame = build_frame(OP.SET_XYZ, 7, body)
    decoded = cobs_decode(frame)
    op, seq, rx_body = parse_frame(decoded)
    assert op == OP.SET_XYZ and seq == 7 and rx_body == body


def test_frame_crc_corruption_rejected():
    frame = bytearray(cobs_decode(build_frame(OP.PING, 1)))
    frame[-1] ^= 0xFF                            # corrupt
    assert parse_frame(bytes(frame)) is None


# --- status payload -----------------------------------------------------
def _make_status(cur=1234, fault=Fault.NONE):
    return (struct.pack("<4f", 1.0, 2.0, 3.0, 4.0) + struct.pack("<f", 5.0)
            + struct.pack("<3f", 150.0, 0.0, 120.0) + struct.pack("<f", -30.0)
            + struct.pack("<I", cur) + bytes([fault, 2, 1, 0]))


def test_decode_status():
    st = decode_status(_make_status(cur=4096, fault=Fault.OVERCURRENT))
    assert st.joints == (1.0, 2.0, 3.0, 4.0)
    assert st.grip == 5.0 and st.xyz == (150.0, 0.0, 120.0) and st.pitch == -30.0
    assert st.current_ma == 4096
    assert st.fault_enum is Fault.OVERCURRENT and st.in_fault
    assert st.idle is True


# --- streamer + client end-to-end --------------------------------------
def test_client_tx_and_telemetry():
    t = LoopbackTransport()
    c = KiaClient(t)
    got = []
    c.on_telemetry = got.append

    seq = c.set_joint(2, 45.0, 1.5)
    op, rx_seq, body = parse_frame(cobs_decode(bytes(t.tx_log)))
    assert op == OP.SET_JOINT and rx_seq == seq
    assert struct.unpack("<Bff", body) == (2, 45.0, 1.5)

    # device pushes telemetry split across two chunks
    tlm = build_frame(RSP.TELEMETRY, 0, _make_status())
    t.inject(tlm[:5]); t.inject(tlm[5:])
    assert len(got) == 1
    assert c.last_telemetry.current_ma == 1234


def test_streamer_mixed_text_and_binary():
    s = FrameStreamer()
    frames, lines = s.feed(b"PONG fw=1.0.0\r\n")
    assert not frames and lines == ["PONG fw=1.0.0"]
    tlm = build_frame(RSP.TELEMETRY, 0, _make_status())
    frames, lines = s.feed(tlm)
    assert len(frames) == 1 and frames[0][0] == RSP.TELEMETRY


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn(); print(f"  ok  {fn.__name__}")
    print(f"\n{len(fns)} passed")
