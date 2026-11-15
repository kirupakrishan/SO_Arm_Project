"""Pure conversion helpers between URDF joint space (radians) and LeRobot units.

Kept free of ROS / LeRobot imports so it can be unit tested without hardware.

LeRobot SO-101 with ``use_degrees=True``:
  * body joints are in degrees, 0 = middle of the calibrated range
    (this matches ``so101_new_calib.urdf``, whose zero pose is the same).
  * gripper is always RANGE_0_100 (0 = closed end of range, 100 = open end).
"""

import math

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
GRIPPER = 'gripper'
ALL_JOINTS = ARM_JOINTS + [GRIPPER]

# Limits copied from Viz_asset/SO101/so101_new_calib.urdf (radians).
URDF_LIMITS = {
    'shoulder_pan': (-1.91986, 1.91986),
    'shoulder_lift': (-1.74533, 1.74533),
    'elbow_flex': (-1.69, 1.69),
    'wrist_flex': (-1.65806, 1.65806),
    'wrist_roll': (-2.74385, 2.84121),
    'gripper': (-0.174533, 1.74533),
}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def urdf_to_lerobot(name, rad, sign=1.0, offset_deg=0.0,
                    gripper_closed_rad=-0.174533, gripper_open_rad=1.74533):
    """URDF joint angle (rad) -> LeRobot command value (deg, or 0..100 for gripper)."""
    lo, hi = URDF_LIMITS[name]
    rad = clamp(rad, lo, hi)
    if name == GRIPPER:
        pct = (rad - gripper_closed_rad) / (gripper_open_rad - gripper_closed_rad) * 100.0
        return clamp(pct, 0.0, 100.0)
    return sign * math.degrees(rad) + offset_deg


def lerobot_to_urdf(name, value, sign=1.0, offset_deg=0.0,
                    gripper_closed_rad=-0.174533, gripper_open_rad=1.74533):
    """LeRobot reading (deg, or 0..100 for gripper) -> URDF joint angle (rad)."""
    if name == GRIPPER:
        return gripper_closed_rad + (value / 100.0) * (gripper_open_rad - gripper_closed_rad)
    return math.radians((value - offset_deg) * sign)


def step_towards(current, target, max_step):
    """Move ``current`` toward ``target`` by at most ``max_step`` (rate limiter)."""
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + math.copysign(max_step, delta)
