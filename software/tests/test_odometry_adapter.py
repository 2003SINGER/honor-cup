"""ROS-independent checks for the real feedback boundary."""

import math
from types import SimpleNamespace as NS

import pytest

from m3pro_nav.odometry_adapter import OdometryMonitor, odometry_from_msg


def odom(*, yaw=math.pi / 2, vx=0.04, vy=0.0, sec=12, nanosec=0,
         frame='odom', child='base_footprint'):
    q = NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    return NS(header=NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nanosec)),
              child_frame_id=child,
              pose=NS(pose=NS(position=NS(x=1.0, y=2.0), orientation=q)),
              twist=NS(twist=NS(linear=NS(x=vx, y=vy), angular=NS(z=0.01))))


def adapt(msg):
    return odometry_from_msg(msg, expected_odom_frame='odom',
                             expected_base_frame='base_footprint')


def test_odometry_twist_is_rotated_from_child_frame_into_pose_frame():
    sample = adapt(odom())
    assert (sample.pose.x, sample.pose.y, sample.pose.yaw) == pytest.approx(
        (1.0, 2.0, math.pi / 2))
    assert (sample.vx_world, sample.vy_world, sample.wz) == pytest.approx(
        (0.0, 0.04, 0.01), abs=1e-12)


@pytest.mark.parametrize('change', [
    {'frame': ''}, {'frame': 'map'}, {'child': ''}, {'child': 'base_link'},
    {'vx': math.nan}, {'yaw': math.nan}, {'sec': -1}, {'nanosec': 1_000_000_000},
])
def test_odometry_rejects_unverified_frames_or_invalid_values(change):
    with pytest.raises(ValueError):
        adapt(odom(**change))


def test_odometry_receipt_age_and_source_stamp_both_guard_control():
    monitor = OdometryMonitor(max_age=0.5)
    first = adapt(odom(sec=12))
    assert monitor.accept(first, now=20.2, received=20.0,
                          expected_odom_frame='odom',
                          expected_base_frame='base_footprint') is first
    with pytest.raises(ValueError, match='advance'):
        monitor.accept(first, now=20.3, received=20.3,
                       expected_odom_frame='odom',
                       expected_base_frame='base_footprint')
    later = adapt(odom(sec=13))
    with pytest.raises(ValueError, match='stale'):
        monitor.accept(later, now=21.0, received=20.0,
                       expected_odom_frame='odom',
                       expected_base_frame='base_footprint')
