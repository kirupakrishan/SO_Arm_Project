import math

import numpy as np
import pytest

from arm_vision_teleop.arm_geometry import (
    joint_angle, map_axes, palm_normal, pick_side, reach_from_elbow, unwrap_near,
    wrist_flex, wrist_roll,
)

# Camera-aligned frame: x = image right, y = image down, z = away from camera.
UP = np.array([0.0, -1.0, 0.0])
TO_CAM = np.array([0.0, 0.0, -1.0])
RIGHT = np.array([1.0, 0.0, 0.0])


def right_hand_palm_to_camera(flex=0.0):
    """User's right hand in a mirrored image, fingers up, palm facing the camera.

    In the mirrored image the thumb/index side is on the image LEFT.
    ``flex`` bends the hand toward the palm (i.e. toward the camera).
    """
    hand_dir = math.cos(flex) * UP + math.sin(flex) * TO_CAM
    across = RIGHT                          # index (left) -> pinky (right)
    lm = [np.zeros(3) for _ in range(21)]
    lm[9] = 0.08 * hand_dir                 # middle MCP
    lm[5] = 0.08 * hand_dir - 0.03 * across  # index MCP (image left)
    lm[17] = 0.07 * hand_dir + 0.03 * across  # pinky MCP (image right)
    return lm, hand_dir


def test_joint_angle():
    assert joint_angle([0, 1, 0], [0, 0, 0], [1, 0, 0]) == pytest.approx(math.pi / 2)
    assert joint_angle([0, 1, 0], [0, 0, 0], [0, -1, 0]) == pytest.approx(math.pi)


def test_palm_normal_points_out_of_palm():
    lm, _ = right_hand_palm_to_camera()
    n = palm_normal(lm, 'Right', mirrored=True)
    assert n == pytest.approx(TO_CAM, abs=1e-6)
    # same geometry labelled as the other hand (or un-mirrored) flips it
    assert palm_normal(lm, 'Left', mirrored=True) == pytest.approx(-TO_CAM, abs=1e-6)
    assert palm_normal(lm, 'Right', mirrored=False) == pytest.approx(-TO_CAM, abs=1e-6)


@pytest.mark.parametrize('flex', [0.0, 0.5, -0.6])
def test_wrist_flex_signed(flex):
    lm, hand_dir = right_hand_palm_to_camera(flex)
    n = palm_normal(lm, 'Right')
    assert wrist_flex(UP, hand_dir, n) == pytest.approx(flex, abs=1e-6)


def test_wrist_roll():
    # palm facing the camera, forearm vertical -> 0
    assert wrist_roll(UP, TO_CAM) == pytest.approx(0.0, abs=1e-9)
    # palm rotated a quarter turn about the forearm -> +-90 deg
    assert abs(wrist_roll(UP, RIGHT)) == pytest.approx(math.pi / 2)
    assert wrist_roll(UP, RIGHT) == pytest.approx(-wrist_roll(UP, -RIGHT))
    # forearm pointing at the camera: falls back to "palm up" as zero
    assert wrist_roll(TO_CAM, UP) == pytest.approx(0.0, abs=1e-9)


def test_map_axes_default_robot_frame():
    # forearm pointing at the camera -> robot forward (+x); up -> +z; image left -> +y
    m = ['-z', '-x', '-y']
    assert map_axes(TO_CAM, m) == pytest.approx([1, 0, 0])
    assert map_axes(UP, m) == pytest.approx([0, 0, 1])
    assert map_axes(-RIGHT, m) == pytest.approx([0, 1, 0])


def test_reach_from_elbow():
    a, b = math.radians(60), math.radians(170)
    assert reach_from_elbow(math.radians(30), 0.1, 0.3, a, b) == 0.1
    assert reach_from_elbow(math.radians(180), 0.1, 0.3, a, b) == 0.3
    assert reach_from_elbow(math.radians(115), 0.1, 0.3, a, b) == pytest.approx(0.2)


def test_pick_side_and_unwrap():
    pose = [(0.0, 0.0)] * 33
    pose[15], pose[16] = (0.2, 0.5), (0.8, 0.5)
    assert pick_side(pose, (0.75, 0.45)) == 'right'
    assert pick_side(pose, (0.1, 0.6)) == 'left'
    assert unwrap_near(-3.1, 3.1) == pytest.approx(-3.1 + 2 * math.pi)
    assert unwrap_near(0.5, None) == 0.5
