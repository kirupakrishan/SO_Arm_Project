import math

import pytest

from lerobot_real.mapping import ALL_JOINTS, lerobot_to_urdf, step_towards, urdf_to_lerobot


@pytest.mark.parametrize('name', ALL_JOINTS)
def test_round_trip(name):
    for rad in (-0.1, 0.0, 0.5, 1.2):
        val = urdf_to_lerobot(name, rad, sign=-1.0, offset_deg=3.0)
        assert lerobot_to_urdf(name, val, sign=-1.0, offset_deg=3.0) == pytest.approx(rad)


def test_degrees_and_clamp():
    assert urdf_to_lerobot('elbow_flex', math.radians(45)) == pytest.approx(45.0)
    assert urdf_to_lerobot('elbow_flex', 10.0) == pytest.approx(math.degrees(1.69))


def test_gripper_range():
    assert urdf_to_lerobot('gripper', -0.174533) == pytest.approx(0.0)
    assert urdf_to_lerobot('gripper', 1.74533) == pytest.approx(100.0)


def test_step_towards():
    assert step_towards(0.0, 10.0, 3.0) == 3.0
    assert step_towards(0.0, -10.0, 3.0) == -3.0
    assert step_towards(0.0, 2.0, 3.0) == 2.0
