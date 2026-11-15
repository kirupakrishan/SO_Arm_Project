"""Shared helpers for ROS nodes that talk to the real SO-101 through LeRobot.

Used by follower_bridge (sim -> real) and real_to_sim (real -> sim).
Requires the lerobot conda env interpreter (the launch files handle that).
"""

import time
from dataclasses import dataclass

from sensor_msgs.msg import JointState

from lerobot_real.mapping import ALL_JOINTS, URDF_LIMITS, clamp, lerobot_to_urdf, urdf_to_lerobot

try:
    from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
except ImportError as e:  # pragma: no cover - depends on runtime env
    raise SystemExit(
        f'[lerobot_real] cannot import lerobot ({e}).\n'
        'Run this node with the lerobot conda env python, e.g. via a lerobot_real launch file.'
    )


@dataclass
class ArmParams:
    port: str
    robot_id: str
    calibration_dir: str
    connect_attempts: int
    sign: dict
    offset: dict
    grip_closed: float
    grip_open: float

    def to_lerobot(self, name, rad):
        return urdf_to_lerobot(name, rad, self.sign[name], self.offset[name],
                               self.grip_closed, self.grip_open)

    def to_urdf(self, name, value):
        return lerobot_to_urdf(name, value, self.sign[name], self.offset[name],
                               self.grip_closed, self.grip_open)


def declare_arm_params(node) -> ArmParams:
    """Declare the parameters common to every node that drives/reads the arm."""
    p = node.declare_parameter
    signs = list(p('joint_signs', [1.0] * len(ALL_JOINTS)).value)
    offsets = list(p('joint_offsets_deg', [0.0] * len(ALL_JOINTS)).value)
    if len(signs) != len(ALL_JOINTS) or len(offsets) != len(ALL_JOINTS):
        raise ValueError(f'joint_signs / joint_offsets_deg must have {len(ALL_JOINTS)} '
                         f'entries in order {ALL_JOINTS}')
    return ArmParams(
        port=p('port', '/dev/ttyACM0').value,
        robot_id=p('robot_id', 'vellayan').value,
        calibration_dir=p('calibration_dir', '').value,
        connect_attempts=int(p('connect_attempts', 10).value),
        sign=dict(zip(ALL_JOINTS, map(float, signs))),
        offset=dict(zip(ALL_JOINTS, map(float, offsets))),
        grip_closed=float(p('gripper_closed_rad', -0.174533).value),
        grip_open=float(p('gripper_open_rad', 1.74533).value),
    )


def retry(node, fn, what, attempts, delay_s=1.0):
    for i in range(1, attempts + 1):
        try:
            return fn()
        except Exception as e:
            if i == attempts:
                raise
            node.get_logger().warn(f'{what} failed ({i}/{attempts}): {e}; retrying...')
            time.sleep(delay_s)


def connect_so101(node, ap: ArmParams, disable_torque_on_disconnect=True):
    """Create + connect an SO101Follower in degree mode, without interactive prompts."""
    cfg = SO101FollowerConfig(
        port=ap.port,
        id=ap.robot_id,
        use_degrees=True,
        disable_torque_on_disconnect=disable_torque_on_disconnect,
        **({'calibration_dir': ap.calibration_dir} if ap.calibration_dir else {}),
    )
    robot = SO101Follower(cfg)
    if not robot.calibration:
        raise RuntimeError(
            f'No calibration file found at {robot.calibration_fpath}. '
            'Run `lerobot-calibrate --robot.type=so101_follower '
            f'--robot.port={ap.port} --robot.id={ap.robot_id}` first.')

    def connect():
        if robot.bus.is_connected:  # half-open from a failed attempt
            robot.bus.disconnect(disable_torque=False)
        # calibrate=False -> never block on input(); push the file to the motors instead.
        robot.connect(calibrate=False)
        if not robot.is_calibrated:
            node.get_logger().warn('Motor calibration differs from file; writing it to motors.')
            robot.bus.write_calibration(robot.calibration)

    node.get_logger().info(f'Connecting to SO-101 on {ap.port}...')
    retry(node, connect, f'connect on {ap.port}', ap.connect_attempts)
    return robot


def read_present(robot):
    """Present positions in LeRobot units (deg; gripper 0..100)."""
    obs = robot.bus.sync_read('Present_Position')
    return {j: float(obs[j]) for j in ALL_JOINTS}


def to_joint_state(node, ap: ArmParams, present, clamp_to_urdf=False):
    """LeRobot reading -> sensor_msgs/JointState in URDF radians."""
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = list(ALL_JOINTS)
    pos = [ap.to_urdf(j, present[j]) for j in ALL_JOINTS]
    if clamp_to_urdf:
        pos = [clamp(v, *URDF_LIMITS[j]) for j, v in zip(ALL_JOINTS, pos)]
    msg.position = pos
    return msg
