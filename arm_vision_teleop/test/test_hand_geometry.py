import math
import socket

import pytest

from arm_vision_teleop.hand_geometry import ClutchMapping, Ema, clamp_box, compute_features
from arm_vision_teleop.stream_protocol import ProtocolError, pack_frame, recv_frame


def make_hand(cx=0.5, cy=0.5, s=0.1, pinch=1.0, fist=False):
    """Synthetic 21-landmark hand, upright, centred at (cx, cy), wrist->MCP = s.

    Square image is assumed (width == height) in the tests below.
    """
    lm = [(cx, cy)] * 21
    lm = list(lm)
    lm[0] = (cx, cy + s / 2)                          # wrist
    for i, dx in zip((5, 9, 13, 17), (-0.03, 0.0, 0.03, 0.06)):
        lm[i] = (cx + dx, cy - s / 2)                 # MCPs, size = s
    tip_dist = 0.8 if fist else 2.0                   # tip-to-wrist in units of s
    for tip, mcp in ((12, 9), (16, 13), (20, 17)):
        lm[tip] = (lm[mcp][0], lm[0][1] - tip_dist * s)
    lm[8] = (cx, cy - 1.2 * s)                        # index tip
    lm[4] = (cx + pinch * s, cy - 1.2 * s)            # thumb tip, `pinch` sizes from index
    return lm


def test_features_basic():
    f = compute_features(make_hand(), 480, 480)
    assert f.size == pytest.approx(0.1)
    assert f.openness == 1.0   # pinch 1.0 > pinch_open 0.9
    assert f.fist is False


def test_pinch_closes_gripper_and_fist_detected():
    assert compute_features(make_hand(pinch=0.1), 480, 480).openness == 0.0
    mid = compute_features(make_hand(pinch=0.575), 480, 480).openness
    assert mid == pytest.approx(0.5)
    assert compute_features(make_hand(fist=True), 480, 480).fist is True


def test_features_scale_invariant():
    a = compute_features(make_hand(s=0.1, pinch=0.5), 480, 480)
    b = compute_features(make_hand(s=0.2, pinch=0.5), 480, 480)
    assert a.openness == pytest.approx(b.openness)
    assert b.size == pytest.approx(2 * a.size)


def test_bad_landmarks():
    with pytest.raises(ValueError):
        compute_features([(0, 0)] * 5, 640, 480)
    with pytest.raises(ValueError):
        compute_features([(0.5, 0.5)] * 21, 640, 480)


def test_ema():
    e = Ema(0.5)
    assert e((0.0, 2.0)) == (0.0, 2.0)
    assert e((1.0, 0.0)) == (0.5, 1.0)
    e.reset()
    assert e((4.0, 4.0)) == (4.0, 4.0)


def test_clutch_mapping():
    m = ClutchMapping(gain_lateral=0.3, gain_vertical=0.2, gain_depth=0.1)
    assert not m.engaged
    m.engage((0.5, 0.5, 0.1), (0.2, 0.0, 0.1))
    assert m.target((0.5, 0.5, 0.1)) == pytest.approx([0.2, 0.0, 0.1])  # no motion
    t = m.target((0.6, 0.4, 0.1 * math.e))
    assert t[1] == pytest.approx(-0.03)     # lateral: right in image -> -y (default sign)
    assert t[2] == pytest.approx(0.1 + 0.02)  # hand up -> tip up
    assert t[0] == pytest.approx(0.2 + 0.1)   # hand e-times bigger -> +depth gain
    m.release()
    assert not m.engaged


def test_clamp_box():
    assert clamp_box([1, -1, 0.5], [0, 0, 0], [0.5, 0.5, 0.5]) == [0.5, 0, 0.5]


def test_stream_protocol_roundtrip():
    a, b = socket.socketpair()
    try:
        a.sendall(pack_frame(b'jpegdata') + pack_frame(b'x' * 100000))
        assert recv_frame(b) == b'jpegdata'
        assert recv_frame(b) == b'x' * 100000
        a.sendall(b'GET / HTTP/1.1\r\n')
        with pytest.raises(ProtocolError):
            recv_frame(b)
    finally:
        a.close()
        b.close()
