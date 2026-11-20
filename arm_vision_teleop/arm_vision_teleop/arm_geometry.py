"""Pure math for elbow-anchored arm teleop (mode:=arm). numpy only, no ROS/MediaPipe.

Your elbow is the base. From MediaPipe's 3D *world* landmarks (meters, camera-aligned
axes: x = image right, y = image down, z = away from the camera) we extract:

  forearm direction   elbow -> wrist unit vector       -> where the gripper tip points
  elbow angle         shoulder-elbow-wrist angle       -> how far the tip reaches
  wrist flex          hand bend vs forearm, + = toward the palm   -> robot wrist_flex
  wrist roll          palm rotation about the forearm, 0 = palm facing the camera
                                                       -> robot wrist_roll

A rigid forearm with a fixed elbow only has 2 positional DOF (its direction), so the
third (reach) comes from the elbow bend: straight arm = far, bent = near.
"""

import math

import numpy as np

# MediaPipe pose landmark indices: (shoulder, elbow, wrist) for each body side.
POSE_SIDES = {'left': (11, 13, 15), 'right': (12, 14, 16)}
# MediaPipe hand landmark indices.
H_WRIST, H_INDEX_MCP, H_MIDDLE_MCP, H_PINKY_MCP = 0, 5, 9, 17

AXIS_NAMES = {'x': 0, 'y': 1, 'z': 2}


def unit(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError('zero-length vector')
    return v / n


def joint_angle(a, b, c):
    """Angle at b (radians) between b->a and b->c; pi = straight."""
    u, w = unit(np.subtract(a, b)), unit(np.subtract(c, b))
    return math.acos(max(-1.0, min(1.0, float(np.dot(u, w)))))


def palm_normal(hand_world, handedness, mirrored=True):
    """Unit normal pointing OUT of the palm, from 3D hand world landmarks.

    ``handedness`` is MediaPipe's label ('Left'/'Right'), which describes the real
    hand when the image was mirrored (selfie view) before detection.
    """
    w = np.asarray(hand_world[H_WRIST], float)
    idx = np.asarray(hand_world[H_INDEX_MCP], float) - w
    pky = np.asarray(hand_world[H_PINKY_MCP], float) - w
    n = np.cross(pky, idx)                      # out of the palm for a right hand, mirrored
    if (handedness == 'Left') == mirrored:
        n = -n
    return unit(n)


def _perp(v, axis):
    """Component of v perpendicular to unit ``axis``."""
    return v - np.dot(v, axis) * axis


def wrist_flex(forearm_dir, hand_dir, normal):
    """Signed hand bend vs forearm (rad): + toward the palm (flexion), - extension."""
    f = unit(forearm_dir)
    p = unit(_perp(np.asarray(normal, float), f))   # palm side, perpendicular to forearm
    h = unit(hand_dir)
    return math.atan2(float(np.dot(h, p)), float(np.dot(h, f)))


def wrist_roll(forearm_dir, normal):
    """Palm rotation about the forearm (rad). 0 = palm facing the camera.

    Falls back to "palm facing up" as the zero when the forearm points at the camera
    (the camera direction is then parallel to the forearm and can't define a zero).
    """
    f = unit(forearm_dir)
    n = _perp(np.asarray(normal, float), f)
    ref = _perp(np.array([0.0, 0.0, -1.0]), f)      # toward the camera
    if np.linalg.norm(ref) < 0.3:
        ref = _perp(np.array([0.0, -1.0, 0.0]), f)  # up
    n, ref = unit(n), unit(ref)
    return math.atan2(float(np.dot(f, np.cross(ref, n))), float(np.dot(ref, n)))


def map_axes(v, axis_map):
    """Re-express a camera-frame vector in the robot frame.

    ``axis_map``: for robot x, y, z, which camera axis feeds it, e.g. ['-z', '-x', '-y']
    = robot x <- toward the camera, robot y <- image left, robot z <- up.
    """
    out = []
    for spec in axis_map:
        sign = -1.0 if spec.startswith('-') else 1.0
        out.append(sign * float(v[AXIS_NAMES[spec.lstrip('+-')]]))
    return np.array(out)


def reach_from_elbow(angle, reach_min, reach_max, bent_angle, straight_angle):
    """Elbow angle (rad) -> tip distance from the robot's shoulder (m), linear + clamped."""
    t = (angle - bent_angle) / (straight_angle - bent_angle)
    return reach_min + max(0.0, min(1.0, t)) * (reach_max - reach_min)


def pick_side(pose_2d, hand_wrist_2d):
    """Which pose arm ('left'/'right') belongs to the tracked hand: nearest wrist."""
    def d(side):
        x, y = pose_2d[POSE_SIDES[side][2]]
        return (x - hand_wrist_2d[0]) ** 2 + (y - hand_wrist_2d[1]) ** 2
    return min(POSE_SIDES, key=d)


def unwrap_near(angle, reference):
    """Shift ``angle`` by 2*pi so it's within pi of ``reference`` (for smoothing)."""
    if reference is None:
        return angle
    return reference + (angle - reference + math.pi) % (2 * math.pi) - math.pi
