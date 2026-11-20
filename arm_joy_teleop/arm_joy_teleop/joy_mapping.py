"""Pure helpers: gamepad (sensor_msgs/Joy axes/buttons) -> velocity commands.

No ROS imports, so this can be unit tested without a joystick or ROS running.

Index conventions follow the ROS 2 Humble ``joy`` package (SDL2 game-controller
mapping, same for Xbox / PlayStation / most USB pads):

  axes:    0 left-X   1 left-Y   2 right-X   3 right-Y   4 LT   5 RT
           (stick left = +1, stick up = +1)
  buttons: 0 A/cross  1 B/circle  2 X/square  3 Y/triangle  4 back  5 guide
           6 start  7 L-stick  8 R-stick  9 LB  10 RB
           11 dpad-up  12 dpad-down  13 dpad-left  14 dpad-right

Every index is configurable in config/gamepad.yaml; -1 means "not mapped".
"""

import math
import xml.etree.ElementTree as ET

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
GRIPPER = 'gripper'
ALL_JOINTS = ARM_JOINTS + [GRIPPER]


def axis(axes, idx):
    """Axis value, or 0.0 if unmapped (-1) or the pad doesn't have that axis."""
    if idx is None or idx < 0 or idx >= len(axes):
        return 0.0
    v = float(axes[idx])
    return 0.0 if math.isnan(v) else max(-1.0, min(1.0, v))


def button(buttons, idx):
    """True if the button is pressed; False if unmapped or out of range."""
    if idx is None or idx < 0 or idx >= len(buttons):
        return False
    return bool(buttons[idx])


def button_pair(buttons, pos_idx, neg_idx):
    """+1 / -1 / 0 from a pair of buttons (e.g. dpad left/right)."""
    return float(button(buttons, pos_idx)) - float(button(buttons, neg_idx))


def apply_deadzone(v, deadzone):
    """Zero small stick values and rescale the rest so output still spans 0..1."""
    if abs(v) <= deadzone:
        return 0.0
    return math.copysign((abs(v) - deadzone) / (1.0 - deadzone), v)


def channel(axes, buttons, spec, deadzone=0.0):
    """Evaluate one input channel in [-1, 1].

    ``spec`` is a dict with optional keys:
      axis (int), scale (float, sign flips direction), button_pos, button_neg (int).
    Axis and buttons are summed then clamped, so either can drive the channel.
    """
    v = apply_deadzone(axis(axes, spec.get('axis', -1)), deadzone) * spec.get('scale', 1.0)
    v += button_pair(buttons, spec.get('button_pos', -1), spec.get('button_neg', -1))
    return max(-1.0, min(1.0, v))


class EdgeDetector:
    """Reports a button only on the press transition (released -> pressed)."""

    def __init__(self):
        self._prev = {}

    def pressed(self, buttons, idx):
        now = button(buttons, idx)
        was = self._prev.get(idx, False)
        self._prev[idx] = now
        return now and not was


def step_towards(current, target, max_step):
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + math.copysign(max_step, delta)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def load_urdf_limits(urdf_path, joints=ALL_JOINTS):
    """{joint: (lower, upper)} for the given revolute joints of a URDF file."""
    root = ET.parse(urdf_path).getroot()
    limits = {}
    for j in root.findall('joint'):  # direct children only (skips <transmission>)
        name = j.get('name')
        lim = j.find('limit')
        if name in joints and lim is not None:
            limits[name] = (float(lim.get('lower')), float(lim.get('upper')))
    missing = [n for n in joints if n not in limits]
    if missing:
        raise ValueError(f'URDF {urdf_path} has no limits for joints: {missing}')
    return limits


def damped_least_squares(J, v, damping):
    """qdot = J^T (J J^T + damping^2 I)^-1 v  (J is m x n, nested lists or numpy)."""
    import numpy as np
    J = np.asarray(J, dtype=float)
    m = J.shape[0]
    return J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(m), np.asarray(v, float))
