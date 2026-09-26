"""Deterministic checks of braking and seam speeds on fixed geometry."""

import math
import pytest

from m3pro_nav.motion_primitive import MotionPrimitive
from m3pro_nav.pose import Pose2D
from m3pro_nav.speed_profile import SpeedProfile


def straight(x0, x1, *, vmax=0.7, vend=0.0):
    return MotionPrimitive(
        kind='STRAIGHT', start_pose=Pose2D(x0, 0.0, math.pi / 2),
        p0=(x0, 0.0), p1=(x1, 0.0), length=abs(x1 - x0),
        v_max=vmax, v_end=vend)


def test_profile_brakes_to_unknown_boundary_stop():
    moving = straight(0.0, 0.4)
    stop = MotionPrimitive(kind='STOP', start_pose=Pose2D(0.4, 0.0, math.pi / 2),
                           p0=(0.4, 0.0), duration=0.2)
    profile = SpeedProfile([moving, stop], a_acc=1.0, a_dec=1.0)
    samples = [profile.sample(profile.duration * i / 200) for i in range(201)]
    assert profile.length == pytest.approx(0.4)
    assert profile.end_speed == 0.0
    assert max(p.speed for p in samples) <= moving.v_max + 1e-9
    assert max(abs(p.acceleration) for p in samples) <= 1.0 + 1e-9
    assert all(a.progress_s <= b.progress_s + 1e-9 for a, b in zip(samples, samples[1:]))
    assert profile.sample(profile.duration).progress_s == pytest.approx(0.4)
    assert profile.sample(profile.duration).speed == 0.0


def test_profile_stops_before_direction_reversal():
    outbound = straight(0.0, 0.2)
    inbound = straight(0.2, 0.0)
    profile = SpeedProfile([outbound, inbound], a_acc=1.0, a_dec=1.0)
    assert profile.length == pytest.approx(0.4)
    assert profile.end_speed == 0.0
    samples = [profile.sample(profile.duration * i / 400) for i in range(401)]
    seam = min(samples, key=lambda p: abs(p.progress_s - 0.2))
    assert abs(seam.progress_s - 0.2) < 1e-4
    assert seam.speed < 0.015


def test_profile_rejects_unbrakeable_or_moving_stop():
    with pytest.raises(ValueError, match='cannot brake'):
        SpeedProfile([straight(0.0, 0.1, vend=0.0)], start_speed=0.7)
    bad = straight(0.0, 0.4, vend=0.2)
    stop = MotionPrimitive(kind='STOP', start_pose=Pose2D(0.4, 0.0, 0.0),
                           p0=(0.4, 0.0), duration=0.2)
    with pytest.raises(ValueError, match='before STOP'):
        SpeedProfile([bad, stop])


def test_short_following_segment_lowers_earlier_seam_speed():
    first = straight(0.0, 0.4, vend=0.7)
    second = straight(0.4, 0.45, vend=0.0)
    profile = SpeedProfile([first, second], a_acc=1.0, a_dec=1.0)
    # The 5 cm final segment can brake only from sqrt(2*a*d).
    assert profile.boundary_speeds[1] == pytest.approx(math.sqrt(0.1))
    assert profile.boundary_speeds[2] == 0.0


def test_root_bootstrap_does_not_demand_unreachable_cruise_speed():
    bootstrap = straight(0.0, 0.2, vend=0.7)
    profile = SpeedProfile([bootstrap], a_acc=1.0, a_dec=1.0)
    assert profile.boundary_speeds[1] == pytest.approx(math.sqrt(0.4))


def test_reversal_seam_requires_explicit_zero_speed():
    with pytest.raises(ValueError, match='before reversal'):
        SpeedProfile([straight(0.0, 0.2, vend=0.1),
                      straight(0.2, 0.0, vend=0.0)])
