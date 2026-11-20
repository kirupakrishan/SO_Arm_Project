"""ROS 2 node: gamepad teleoperation of the SO-101 arm.

    joy_node (joy pkg) --/joy--> joy_teleop --/joint_states--> robot_state_publisher -> RViz
                                     ^                    \\--> lerobot_real follower_bridge -> real arm
                                     |                                   |
                                     \\------- /lerobot/joint_states <----/   (initial pose seed)

Two modes, toggled with START:
  * joint     - each stick axis / button pair drives one joint's velocity.
  * cartesian - left/right sticks move the gripper tip in x/y/z (world frame) using
                damped-least-squares differential IK on shoulder_pan/lift/elbow;
                wrist_flex, wrist_roll and gripper stay under direct control.

Safety: nothing moves unless the deadman button (LB by default) is held, input that
goes stale (no /joy for joy_timeout_s) is treated as zero, every joint is
velocity-limited and clamped to the URDF limits. The node keeps publishing the
held pose so RViz / the real arm stay put.
"""

import math
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState

from arm_joy_teleop.joy_mapping import (
    ALL_JOINTS, GRIPPER, EdgeDetector, button, channel, clamp, damped_least_squares,
    load_urdf_limits, step_towards,
)

IK_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex']   # solve tip x/y/z with these
CART_AXES = ['x', 'y', 'z']
TIP_FRAME = 'gripper_frame_link'

# Default mapping (Humble joy / SDL2 indices, see joy_mapping.py).
DEFAULT_JOINT_MAP = {
    'shoulder_pan':  {'axis': 0, 'scale': 1.0},           # left stick X
    'shoulder_lift': {'axis': 1, 'scale': 1.0},           # left stick Y
    'elbow_flex':    {'axis': 3, 'scale': 1.0},           # right stick Y
    'wrist_flex':    {'axis': 2, 'scale': 1.0},           # right stick X
    'wrist_roll':    {'button_pos': 14, 'button_neg': 13},  # dpad right / left
    'gripper':       {'button_pos': 1, 'button_neg': 0},    # B open / A close
}
DEFAULT_CART_MAP = {
    'x': {'axis': 1, 'scale': 1.0},           # left stick Y  -> forward/back
    'y': {'axis': 0, 'scale': 1.0},           # left stick X  -> left/right
    'z': {'axis': 3, 'scale': 1.0},           # right stick Y -> up/down
    'wrist_flex': {'axis': 2, 'scale': 1.0},  # right stick X -> wrist pitch
}
SPEC_KEYS = {'axis': -1, 'scale': 1.0, 'button_pos': -1, 'button_neg': -1}


class JoyTeleop(Node):

    def __init__(self):
        super().__init__('arm_joy_teleop')
        p = self.declare_parameter

        self.rate_hz = float(p('rate_hz', 30.0).value)
        self.deadzone = float(p('deadzone', 0.1).value)
        self.joy_timeout_s = float(p('joy_timeout_s', 0.5).value)
        self.require_deadman = bool(p('require_deadman', True).value)
        self.max_joint_vel = float(p('max_joint_vel_rad_s', 0.8).value)
        self.max_gripper_vel = float(p('max_gripper_vel_rad_s', 1.5).value)
        self.max_linear_vel = float(p('max_linear_vel_m_s', 0.08).value)
        self.ik_damping = float(p('ik_damping', 0.02).value)
        self.speed = float(p('speed_scale', 0.5).value)
        self.speed_min, self.speed_max = 0.1, 1.0
        self.mode = p('start_mode', 'joint').value
        self.home_pose = list(p('home_pose', [0.0] * len(ALL_JOINTS)).value)
        self.seed_timeout_s = float(p('seed_timeout_s', 2.0).value)

        self.btn = {name: int(p(f'buttons.{name}', default).value) for name, default in {
            'deadman': 9, 'mode': 6, 'home': 2, 'speed_up': 11, 'speed_down': 12}.items()}
        self.joint_map = self._declare_specs('joint', DEFAULT_JOINT_MAP)
        self.cart_map = self._declare_specs('cartesian', DEFAULT_CART_MAP)

        urdf = p('urdf_path', '').value or os.path.join(
            get_package_share_directory('arm_movement'), 'Viz_asset', 'SO101',
            'so101_new_calib.urdf')
        self.urdf_path = urdf
        self.limits = load_urdf_limits(urdf)
        self.robot = None            # placo model, loaded on first use of Cartesian mode

        if len(self.home_pose) != len(ALL_JOINTS):
            raise ValueError(f'home_pose needs {len(ALL_JOINTS)} values ({ALL_JOINTS})')
        if self.mode not in ('joint', 'cartesian'):
            raise ValueError("start_mode must be 'joint' or 'cartesian'")

        self.q = None                # current commanded pose {joint: rad}; None until seeded
        self.joy = None
        self.joy_time = 0.0
        self.edges = EdgeDetector()
        self.start_time = time.monotonic()

        out_topic = p('output_topic', '/joint_states').value
        seed_topic = p('seed_topic', '/lerobot/joint_states').value
        self.pub = self.create_publisher(JointState, out_topic, 10)
        self.create_subscription(Joy, p('joy_topic', '/joy').value, self._on_joy, 10)
        self.create_subscription(JointState, seed_topic, self._on_seed, 10)
        self.create_timer(1.0 / self.rate_hz, self._tick)

        if self.mode == 'cartesian':
            self._ensure_ik()
        self.get_logger().info(
            f'Joy teleop ready ({self.mode} mode, speed {self.speed:.2f}). Publishing '
            f'{out_topic}. Waiting up to {self.seed_timeout_s:.1f}s for {seed_topic} '
            f'to start from the real arm pose.'
            + (f' Hold button {self.btn["deadman"]} (deadman) to move.'
               if self.require_deadman else ''))

    # ------------------------------------------------------------------ params
    def _declare_specs(self, prefix, defaults):
        specs = {}
        for name, d in defaults.items():
            specs[name] = {}
            for key, fallback in SPEC_KEYS.items():
                val = self.declare_parameter(f'{prefix}.{name}.{key}', d.get(key, fallback)).value
                specs[name][key] = float(val) if key == 'scale' else int(val)
        return specs

    # ------------------------------------------------------------------ inputs
    def _on_joy(self, msg):
        self.joy = msg
        self.joy_time = time.monotonic()

    def _on_seed(self, msg):
        if self.q is not None:
            return
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in ALL_JOINTS):
            self.q = {j: clamp(pos[j], *self.limits[j]) for j in ALL_JOINTS}
            self.get_logger().info('Starting from the real arm pose: ' + ', '.join(
                f'{j}={math.degrees(v):.0f}°' for j, v in self.q.items()))

    # ------------------------------------------------------------------ IK
    def _ensure_ik(self):
        if self.robot is not None:
            return True
        try:
            import placo
        except ImportError:
            self.get_logger().error('Cartesian mode needs placo (pip install placo); '
                                    'staying in joint mode.')
            self.mode = 'joint'
            return False
        self.robot = placo.RobotWrapper(self.urdf_path, placo.Flags.ignore_collisions)
        self.ik_cols = [self.robot.get_joint_v_offset(j) for j in IK_JOINTS]
        return True

    def _cartesian_qdot(self, axes, buttons):
        """Joint velocities for the IK joints from the stick's tip velocity command."""
        for j, v in self.q.items():
            self.robot.set_joint(j, v)
        self.robot.update_kinematics()
        J = self.robot.frame_jacobian(TIP_FRAME, 'local_world_aligned')[:3, self.ik_cols]
        v = [self.max_linear_vel * self.speed *
             channel(axes, buttons, self.cart_map[a], self.deadzone) for a in CART_AXES]
        if not any(v):
            return {j: 0.0 for j in IK_JOINTS}
        qdot = damped_least_squares(J, v, self.ik_damping)
        # Scale the whole vector (not per joint) so the tip keeps its direction.
        peak = max(abs(x) for x in qdot)
        lim = self.max_joint_vel * self.speed
        if peak > lim:
            qdot = qdot * (lim / peak)
        return dict(zip(IK_JOINTS, qdot.tolist()))

    # ------------------------------------------------------------------ loop
    def _tick(self):
        if self.q is None:
            if time.monotonic() - self.start_time < self.seed_timeout_s:
                return
            self.q = {j: clamp(v, *self.limits[j]) for j, v in zip(ALL_JOINTS, self.home_pose)}
            self.get_logger().warn('No real-arm pose received; starting from home_pose '
                                   '(fine for sim only).')

        dt = 1.0 / self.rate_hz
        fresh = self.joy is not None and time.monotonic() - self.joy_time < self.joy_timeout_s
        axes = list(self.joy.axes) if fresh else []
        buttons = list(self.joy.buttons) if fresh else []

        self._handle_buttons(buttons)
        enabled = fresh and (not self.require_deadman or button(buttons, self.btn['deadman']))

        if enabled and button(buttons, self.btn['home']):
            for i, j in enumerate(ALL_JOINTS):
                vmax = self.max_gripper_vel if j == GRIPPER else self.max_joint_vel
                self.q[j] = step_towards(self.q[j], self.home_pose[i], vmax * self.speed * dt)
        elif enabled:
            for j, v in self._velocities(axes, buttons).items():
                if v:
                    self.q[j] = clamp(self.q[j] + v * dt, *self.limits[j])
        self._publish()

    def _velocities(self, axes, buttons):
        def joint_vel(j, spec_map):
            vmax = self.max_gripper_vel if j == GRIPPER else self.max_joint_vel
            return vmax * self.speed * channel(axes, buttons, spec_map[j], self.deadzone)

        if self.mode == 'cartesian':
            qdot = self._cartesian_qdot(axes, buttons)
            qdot['wrist_flex'] = joint_vel('wrist_flex', self.cart_map)
            for j in ('wrist_roll', GRIPPER):
                qdot[j] = joint_vel(j, self.joint_map)
            return qdot
        return {j: joint_vel(j, self.joint_map) for j in ALL_JOINTS}

    def _handle_buttons(self, buttons):
        # Edge detection runs every tick (with [] when stale) so a held button
        # doesn't re-trigger when /joy comes back.
        if self.edges.pressed(buttons, self.btn['mode']):
            new = 'cartesian' if self.mode == 'joint' else 'joint'
            if new == 'joint' or self._ensure_ik():
                self.mode = new
                self.get_logger().info(f'Mode: {self.mode}')
        up = self.edges.pressed(buttons, self.btn['speed_up'])
        down = self.edges.pressed(buttons, self.btn['speed_down'])
        if up or down:
            factor = 1.25 if up else 0.8
            self.speed = clamp(self.speed * factor, self.speed_min, self.speed_max)
            self.get_logger().info(f'Speed scale: {self.speed:.2f}')

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINTS)
        msg.position = [float(self.q[j]) for j in ALL_JOINTS]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
