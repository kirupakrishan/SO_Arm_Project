"""Rest-pose handling shared by follower_bridge (on exit) and the go_to_rest script.

The rest pose is stored in LeRobot units (degrees for the arm joints, 0..100 for
the gripper), i.e. raw arm space, so it may lie outside the URDF limits (the
SO-101 rests folded against its hard stops).
"""

import os
import signal
import time

ALL_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']

# Folded pose measured on the "vellayan" arm, kept a few degrees off the hard
# stops so the motors aren't pushing into them. Record your own with:
#   ros2 run lerobot_real go_to_rest --record
DEFAULT_REST_POSE = {
    'shoulder_pan': 0.0,
    'shoulder_lift': -100.0,
    'elbow_flex': 95.0,
    'wrist_flex': 70.0,
    'wrist_roll': 0.0,
    'gripper': 5.0,
}
DEFAULT_REST_FILE = os.path.expanduser('~/.config/lerobot_real/rest_pose.yaml')


def load_rest_pose(path=DEFAULT_REST_FILE):
    """Rest pose from ``path`` if it exists (written by --record), else the default."""
    pose = dict(DEFAULT_REST_POSE)
    path = os.path.expanduser(path) if path else path
    if path and os.path.isfile(path):
        import yaml
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        pose.update({k: float(v) for k, v in data.items() if k in pose})
    return pose


def save_rest_pose(pose, path=DEFAULT_REST_FILE):
    import yaml
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write('# SO-101 rest pose in LeRobot units (deg; gripper 0..100)\n')
        yaml.safe_dump({j: round(float(pose[j]), 2) for j in ALL_JOINTS}, f, sort_keys=False)


class RestAborted(Exception):
    pass


def move_to_pose(bus, target, speed_deg_s=30.0, gripper_pct_s=50.0, rate_hz=30.0,
                 settle_timeout_s=2.0, tolerance_deg=6.0, log=print):
    """Smoothly move a connected, calibrated (degree-mode) SO-101 bus to ``target``.

    All joints are interpolated together (smoothstep), so they start and stop at the
    same time. A 2nd Ctrl+C or a SIGTERM aborts the motion by raising RestAborted.
    Returns True if every joint ended within ``tolerance_deg`` of the target.
    """
    t_armed = time.monotonic() + 1.0

    def _abort(signum, _frame):
        # The Ctrl+C that triggered this shutdown is often delivered twice (terminal
        # + ros2 launch forwarding it when non-interactive, or a double key press).
        # Ignore SIGINTs in the first second; a deliberate later Ctrl+C aborts.
        if signum == signal.SIGINT and time.monotonic() < t_armed:
            return
        raise RestAborted(signal.Signals(signum).name)

    old = {s: signal.signal(s, _abort) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        start = {j: float(v) for j, v in bus.sync_read('Present_Position').items() if j in target}
        dur = 0.5
        for j in start:
            spd = gripper_pct_s if j == 'gripper' else speed_deg_s
            dur = max(dur, abs(target[j] - start[j]) / spd)
        log(f'Moving to rest over {dur:.1f} s (Ctrl+C again to abort -> torque off)')

        dt = 1.0 / rate_hz
        t0 = time.monotonic()
        while True:
            s = min(1.0, (time.monotonic() - t0) / dur)
            a = s * s * (3 - 2 * s)  # smoothstep: zero velocity at both ends
            bus.sync_write('Goal_Position',
                           {j: start[j] + a * (target[j] - start[j]) for j in start})
            if s >= 1.0:
                break
            time.sleep(dt)

        # Let the position loop settle (P gain is low, gravity causes some sag).
        deadline = time.monotonic() + settle_timeout_s
        while True:
            present = bus.sync_read('Present_Position')
            err = {j: abs(present[j] - target[j]) for j in start if j != 'gripper'}
            if max(err.values()) <= tolerance_deg or time.monotonic() > deadline:
                break
            time.sleep(0.1)
        worst = max(err, key=err.get)
        if err[worst] > tolerance_deg:
            log(f'Rest reached approximately ({worst} off by {err[worst]:.1f} deg).')
            return False
        log('Rest pose reached.')
        return True
    finally:
        for s, h in old.items():
            signal.signal(s, h)
