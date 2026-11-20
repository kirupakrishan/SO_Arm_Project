"""Pure helpers: MediaPipe hand landmarks -> teleop features, and the clutch mapping.

No ROS / MediaPipe / OpenCV imports, so this is unit-testable anywhere.

MediaPipe hand landmark indices:
    0 wrist | thumb 1-4 | index 5-8 | middle 9-12 | ring 13-16 | pinky 17-20
    (per finger: MCP, PIP, DIP, TIP)
"""

import math
from dataclasses import dataclass

WRIST, THUMB_TIP, INDEX_TIP = 0, 4, 8
MIDDLE_MCP = 9
PALM = (0, 5, 9, 13, 17)
# (tip, mcp) for the fingers that aren't used for pinching: used to detect a fist.
CURL_FINGERS = ((12, 9), (16, 13), (20, 17))


@dataclass
class HandFeatures:
    u: float          # palm centre, 0 (left) .. 1 (right) of the image
    v: float          # palm centre, 0 (top) .. 1 (bottom)
    size: float       # wrist -> middle-finger base, as a fraction of image height
    openness: float   # gripper command: 0 = pinched (closed) .. 1 = spread (open)
    fist: bool        # middle, ring and pinky curled -> pause / clutch


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def compute_features(landmarks, width, height,
                     pinch_closed=0.25, pinch_open=0.9, curl_ratio=1.3):
    """``landmarks``: 21 (x, y) pairs in MediaPipe's normalized image coords.

    Distances are computed in pixels-scaled-by-height so they don't depend on
    the image aspect ratio, then normalized by hand size so they don't depend
    on how far the hand is from the camera.
    """
    if len(landmarks) != 21:
        raise ValueError(f'expected 21 landmarks, got {len(landmarks)}')
    aspect = width / float(height)
    pts = [(x * aspect, y) for x, y in landmarks]

    size = _dist(pts[WRIST], pts[MIDDLE_MCP])
    if size < 1e-6:
        raise ValueError('degenerate hand (zero size)')

    u = sum(landmarks[i][0] for i in PALM) / len(PALM)
    v = sum(landmarks[i][1] for i in PALM) / len(PALM)

    pinch = _dist(pts[THUMB_TIP], pts[INDEX_TIP]) / size
    openness = (pinch - pinch_closed) / (pinch_open - pinch_closed)
    openness = max(0.0, min(1.0, openness))

    curled = [_dist(pts[tip], pts[WRIST]) < curl_ratio * _dist(pts[mcp], pts[WRIST])
              for tip, mcp in CURL_FINGERS]
    return HandFeatures(u=u, v=v, size=size, openness=openness, fist=all(curled))


class Ema:
    """Exponential moving average over a tuple of floats; reset() on hand loss."""

    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None

    def reset(self):
        self.value = None

    def __call__(self, sample):
        if self.value is None:
            self.value = tuple(sample)
        else:
            a = self.alpha
            self.value = tuple(a * s + (1 - a) * p for s, p in zip(sample, self.value))
        return self.value


@dataclass
class ClutchMapping:
    """Relative ("mouse-lift") mapping from hand motion to a gripper-tip target.

    When the hand engages, its position and the robot tip are latched as
    references. Afterwards the tip target is the tip reference plus the hand's
    displacement, scaled. Making a fist (or losing the hand) disengages, so you
    can reposition your hand without moving the robot, like lifting a mouse.

    Robot axes (world frame of the URDF):
      lateral  <- image u  (meters per full image width)
      vertical <- image v  (meters per full image height)
      depth    <- log(hand size / reference size)  (meters per e-fold of size,
                  i.e. bringing the hand ~2.7x closer to the camera)
    Each robot axis is chosen by index (0=x, 1=y, 2=z) and sign.
    """

    gain_lateral: float = 0.30
    gain_vertical: float = 0.30
    gain_depth: float = 0.15
    lateral_axis: int = 1
    lateral_sign: float = -1.0
    vertical_axis: int = 2
    vertical_sign: float = -1.0   # image v grows downward; robot z grows upward
    depth_axis: int = 0
    depth_sign: float = 1.0

    def __post_init__(self):
        self.hand_ref = None
        self.tip_ref = None

    @property
    def engaged(self):
        return self.hand_ref is not None

    def engage(self, hand, tip):
        """``hand``: (u, v, size). ``tip``: current robot tip (x, y, z)."""
        self.hand_ref = tuple(hand)
        self.tip_ref = list(tip)

    def release(self):
        self.hand_ref = None
        self.tip_ref = None

    def target(self, hand):
        u, v, size = hand
        u0, v0, s0 = self.hand_ref
        out = list(self.tip_ref)
        out[self.lateral_axis] += self.lateral_sign * self.gain_lateral * (u - u0)
        out[self.vertical_axis] += self.vertical_sign * self.gain_vertical * (v - v0)
        out[self.depth_axis] += self.depth_sign * self.gain_depth * math.log(size / s0)
        return out


def clamp_box(p, lo, hi):
    return [max(l, min(h, x)) for x, l, h in zip(p, lo, hi)]
