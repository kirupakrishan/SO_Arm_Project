"""Move the SO-101 to its rest pose, then turn torque off.

    ros2 run lerobot_real go_to_rest             # move to rest, then go limp
    ros2 run lerobot_real go_to_rest --hold      # stay powered at rest
    ros2 run lerobot_real go_to_rest --record    # save the arm's current pose as rest
                                                 # (place it by hand, torque off)

Does not need ROS. If ``lerobot`` isn't importable (system python), it re-runs
itself with the lerobot conda env's interpreter.
"""

import argparse
import os
import subprocess
import sys

DEFAULT_ENV = os.path.expanduser('~/miniforge3/envs/lerobot')


def _reexec_in_lerobot_env(env):
    py = os.path.join(env, 'bin', 'python')
    if os.environ.get('_GO_TO_REST_REEXEC') or not os.path.exists(py):
        sys.exit(f'[go_to_rest] cannot import lerobot and no usable env at {env} '
                 '(use --lerobot-env).')
    new_env = dict(os.environ, _GO_TO_REST_REEXEC='1')
    new_env['LD_LIBRARY_PATH'] = os.path.join(env, 'lib') + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    # Run as a module so `lerobot_real.rest` stays importable.
    os.execve(py, [py, '-m', 'lerobot_real.go_to_rest'] + sys.argv[1:], new_env)


def main():
    ap = argparse.ArgumentParser(description='Move the SO-101 to its rest pose.')
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--robot-id', default='vellayan')
    ap.add_argument('--speed', type=float, default=30.0, help='deg/s for the slowest-path joint')
    ap.add_argument('--hold', action='store_true', help='keep torque on at rest')
    ap.add_argument('--record', action='store_true',
                    help='save the current (hand-placed) pose as the rest pose and exit')
    ap.add_argument('--rest-file', default=None, help='rest pose yaml (default ~/.config/lerobot_real/rest_pose.yaml)')
    ap.add_argument('--force', action='store_true', help='run even if follower_bridge is running')
    ap.add_argument('--lerobot-env', default=DEFAULT_ENV)
    args = ap.parse_args()

    try:
        from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
    except ImportError:
        _reexec_in_lerobot_env(args.lerobot_env)

    from lerobot_real.rest import (DEFAULT_REST_FILE, RestAborted, load_rest_pose,
                                   move_to_pose, save_rest_pose)
    rest_file = args.rest_file or DEFAULT_REST_FILE

    r = subprocess.run(['pgrep', '-f', 'lib/lerobot_real/(follower_bridge|real_to_sim)'],
                       capture_output=True, text=True)
    pids = [p for p in r.stdout.split() if p not in {str(os.getpid()), str(os.getppid())}]
    if pids and not args.force:
        sys.exit(f'[go_to_rest] a lerobot_real node is running (pid {", ".join(pids)}); '
                 'stop it first (it goes to rest on exit) or pass --force.')

    robot = SO101Follower(SO101FollowerConfig(
        port=args.port, id=args.robot_id, use_degrees=True,
        disable_torque_on_disconnect=not args.hold))
    if not robot.calibration:
        sys.exit(f'[go_to_rest] no calibration at {robot.calibration_fpath}')
    try:
        robot.connect(calibrate=False)
    except Exception as e:
        sys.exit(f'[go_to_rest] cannot connect on {args.port}: {e}')
    if not robot.is_calibrated:
        robot.bus.write_calibration(robot.calibration)

    try:
        if args.record:
            robot.bus.disable_torque()
            pose = robot.bus.sync_read('Present_Position')
            save_rest_pose(pose, rest_file)
            print(f'[go_to_rest] saved rest pose to {rest_file}:')
            for j, v in pose.items():
                print(f'  {j:<14} {v:8.2f}')
            return

        target = load_rest_pose(rest_file)
        print(f'[go_to_rest] target ({"file" if os.path.isfile(rest_file) else "default"}): '
              + ', '.join(f'{j}={v:.0f}' for j, v in target.items()))
        # In case it was limp: hold the current pose first so enabling torque can't jump.
        robot.bus.sync_write('Goal_Position', robot.bus.sync_read('Present_Position'))
        robot.bus.enable_torque()
        move_to_pose(robot.bus, target, speed_deg_s=args.speed,
                     log=lambda m: print(f'[go_to_rest] {m}', flush=True))
    except (RestAborted, KeyboardInterrupt) as e:
        print(f'[go_to_rest] aborted ({e or "Ctrl+C"}) - turning torque off', flush=True)
        robot.config.disable_torque_on_disconnect = True
        robot.bus.disconnect(disable_torque=True)
        sys.exit(1)
    finally:
        if robot.bus.is_connected:
            print('[go_to_rest] ' + ('holding at rest (torque on).' if args.hold
                                     else 'torque off - arm is resting.'))
            robot.bus.disconnect(disable_torque=not args.hold)


if __name__ == '__main__':
    main()
