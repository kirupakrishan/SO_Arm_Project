import os

import pytest

from arm_joy_teleop.joy_mapping import (
    ALL_JOINTS, EdgeDetector, apply_deadzone, axis, button, button_pair, channel,
    damped_least_squares, load_urdf_limits, step_towards,
)

URDF = os.path.join(os.path.dirname(__file__), '..', '..', 'arm_movement', 'Viz_asset',
                    'SO101', 'so101_new_calib.urdf')


def test_axis_and_button_out_of_range_are_zero():
    assert axis([0.5], 3) == 0.0
    assert axis([0.5], -1) == 0.0
    assert axis([float('nan')], 0) == 0.0
    assert axis([2.0], 0) == 1.0
    assert button([1], 5) is False
    assert button([0, 1], 1) is True


def test_deadzone_rescales():
    assert apply_deadzone(0.05, 0.1) == 0.0
    assert apply_deadzone(1.0, 0.1) == pytest.approx(1.0)
    assert apply_deadzone(-0.55, 0.1) == pytest.approx(-0.5)


def test_channel_axis_scale_and_buttons():
    axes, buttons = [0.0, -1.0], [0, 1, 0]
    assert channel(axes, buttons, {'axis': 1, 'scale': -1.0}) == 1.0
    assert button_pair(buttons, 1, 2) == 1.0
    assert channel(axes, buttons, {'button_pos': 2, 'button_neg': 1}) == -1.0
    # axis + button saturate at 1
    assert channel([1.0], [1], {'axis': 0, 'button_pos': 0}) == 1.0


def test_edge_detector_fires_once():
    e = EdgeDetector()
    assert e.pressed([1], 0) is True
    assert e.pressed([1], 0) is False
    assert e.pressed([0], 0) is False
    assert e.pressed([1], 0) is True
    assert e.pressed([], 0) is False  # stale input counts as released


def test_step_towards():
    assert step_towards(0.0, 1.0, 0.25) == 0.25
    assert step_towards(0.9, 1.0, 0.25) == 1.0
    assert step_towards(0.0, -1.0, 0.25) == -0.25


@pytest.mark.skipif(not os.path.exists(URDF), reason='arm_movement URDF not found')
def test_urdf_limits():
    lim = load_urdf_limits(URDF)
    assert set(lim) == set(ALL_JOINTS)
    assert lim['shoulder_pan'] == pytest.approx((-1.91986, 1.91986))
    assert lim['gripper'] == pytest.approx((-0.174533, 1.74533))


def test_damped_least_squares_matches_inverse_when_well_conditioned():
    J = [[2.0, 0.0], [0.0, 4.0]]
    q = damped_least_squares(J, [2.0, 4.0], 1e-6)
    assert q == pytest.approx([1.0, 1.0], abs=1e-6)
