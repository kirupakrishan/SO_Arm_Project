"""Safety script: disable torque on every SO-101 motor (arm goes limp).

    ros2 run lerobot_real torque_off                 # all motors, /dev/ttyACM0
    ros2 run lerobot_real torque_off --delay 3       # 3 s countdown to grab the arm
    ros2 run lerobot_real torque_off --port /dev/ttyACM1 --motors gripper

Does not need ROS. If ``lerobot`` isn't importable (system python), it re-runs
itself with the lerobot conda env's interpreter.
"""

import argparse
import os
import subprocess
import sys
import time

DEFAULT_ENV = os.path.expanduser('~/miniforge3/envs/lerobot')
MOTORS = {  # name: motor id (STS3215), same as lerobot SO101Follower
    'shoulder_pan': 1, 'shoulder_lift': 2, 'elbow_flex': 3,
    'wrist_flex': 4, 'wrist_roll': 5, 'gripper': 6,
}


def _reexec_in_lerobot_env(env):
    py = os.path.join(env, 'bin', 'python')
    if os.environ.get('_TORQUE_OFF_REEXEC') or not os.path.exists(py):
        sys.exit(f'[torque_off] cannot import lerobot and no usable env at {env} '
                 '(use --lerobot-env).')
    new_env = dict(os.environ, _TORQUE_OFF_REEXEC='1')
    new_env['LD_LIBRARY_PATH'] = os.path.join(env, 'lib') + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    os.execve(py, [py, os.path.abspath(__file__)] + sys.argv[1:], new_env)


def _bridge_running():
    r = subprocess.run(['pgrep', '-f', 'lib/lerobot_real/(follower_bridge|real_to_sim)'],
                       capture_output=True, text=True)
    me = {str(os.getpid()), str(os.getppid())}
    return [p for p in r.stdout.split() if p not in me]


def main():
    ap = argparse.ArgumentParser(description='Disable torque on the SO-101 arm.')
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--motors', nargs='+', choices=list(MOTORS), default=list(MOTORS))
    ap.add_argument('--delay', type=float, default=0.0,
                    help='countdown in seconds before releasing (time to support the arm)')
    ap.add_argument('--force', action='store_true',
                    help='run even if follower_bridge is running (it may re-command the arm)')
    ap.add_argument('--lerobot-env', default=DEFAULT_ENV)
    args = ap.parse_args()

    try:
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus
    except ImportError:
        _reexec_in_lerobot_env(args.lerobot_env)

    pids = _bridge_running()
    if pids and not args.force:
        sys.exit(f'[torque_off] a lerobot_real node is running (pid {", ".join(pids)}). '
                 'Stop it first (Ctrl+C its launch, which also disables torque), '
                 'or pass --force.')

    print(f'[torque_off] WARNING: {", ".join(args.motors)} will go limp - support the arm!')
    for t in range(int(args.delay), 0, -1):
        print(f'  releasing in {t}...', flush=True)
        time.sleep(1)

    bus = FeetechMotorsBus(
        port=args.port,
        motors={n: Motor(i, 'sts3215', MotorNormMode.RANGE_M100_100) for n, i in MOTORS.items()},
    )
    try:
        bus.connect(handshake=False)  # don't abort if one motor is unresponsive
    except Exception as e:
        sys.exit(f'[torque_off] cannot open {args.port}: {e}')

    failed = []
    try:
        for m in args.motors:  # one at a time so a single bad motor can't stop the rest
            try:
                bus.disable_torque(m, num_retry=3)
                state = bus.read('Torque_Enable', m, normalize=False, num_retry=3)
                ok = int(state) == 0
            except Exception as e:
                ok, state = False, e
            print(f'  {m:<14} {"torque OFF" if ok else f"FAILED ({state})"}')
            if not ok:
                failed.append(m)
    finally:
        bus.disconnect(disable_torque=False)

    if failed:
        sys.exit(f'[torque_off] could not confirm torque off for: {", ".join(failed)} '
                 '- unplug the arm power supply.')
    print('[torque_off] all requested motors are limp.')


if __name__ == '__main__':
    main()
