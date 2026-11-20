"""ROS 2 node: hand features -> SO-101 joint targets (/joint_states).

    hand_tracker --/hand/position, /hand/openness, /hand/engaged--> hand_follower
        --/joint_states--> robot_state_publisher / RViz, lerobot_real follower_bridge

Clutch ("mouse-lift") control: while your hand is open and visible, the gripper tip
follows your hand's displacement since it engaged (left/right, up/down, toward/away
from the camera). Make a fist or drop your hand out of view to pause, then reposition.
The pinch (thumb-index distance) sets the gripper opening.

mode:=arm (elbow as the base, absolute): your forearm direction points the gripper tip
from the robot's shoulder, your elbow bend sets how far it reaches (straight = far),
your hand bend drives wrist_flex and palm rotation drives wrist_roll. Fist = pause.

The tip is driven with resolved-rate damped-least-squares IK on shoulder_pan,
shoulder_lift and elbow_flex (same method as arm_movement/inv_diff_kine.py).
In hand mode wrist_flex/wrist_roll hold their angles. Runs with system Python (placo).
"""

import math
import os
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32

from arm_joy_teleop.joy_mapping import (
    ALL_JOINTS, GRIPPER, clamp, damped_least_squares, load_urdf_limits, step_towards,
)
from arm_vision_teleop.arm_geometry import map_axes, reach_from_elbow
from arm_vision_teleop.hand_geometry import ClutchMapping, clamp_box

IK_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex']
TIP_FRAME = 'gripper_frame_link'


class HandFollower(Node):

    def __init__(self):
        super().__init__('hand_follower')
        p = self.declare_parameter
        self.rate_hz = float(p('rate_hz', 30.0).value)
        self.hand_timeout_s = float(p('hand_timeout_s', 0.3).value)
        self.kp = float(p('kp', 4.0).value)
        self.max_linear_vel = float(p('max_linear_vel_m_s', 0.15).value)
        self.max_joint_vel = float(p('max_joint_vel_rad_s', 1.0).value)
        self.max_gripper_vel = float(p('max_gripper_vel_rad_s', 2.0).value)
        self.ik_damping = float(p('ik_damping', 0.02).value)
        self.pos_tol = float(p('position_tolerance_m', 0.003).value)
        self.max_lead = float(p('max_lead_m', 0.10).value)
        self.box_lo = list(p('workspace_min', [0.05, -0.35, 0.03]).value)
        self.box_hi = list(p('workspace_max', [0.45, 0.35, 0.45]).value)
        self.control_gripper = bool(p('control_gripper', True).value)
        self.home_pose = list(p('home_pose', [0.0] * len(ALL_JOINTS)).value)
        self.seed_timeout_s = float(p('seed_timeout_s', 2.0).value)
        self.mode = p('mode', 'hand').value
        if self.mode not in ('hand', 'arm'):
            raise ValueError("mode must be 'hand' or 'arm'")
        # --- arm mode (elbow = base)
        self.anchor = np.array(p('arm.anchor_m', [0.0, 0.0, 0.12]).value, float)
        self.axis_map = list(p('arm.axis_map', ['-z', '-x', '-y']).value)
        self.reach_source = p('arm.reach_source', 'elbow_angle').value
        self.reach_fixed = float(p('arm.reach_fixed_m', 0.30).value)
        self.reach_min = float(p('arm.reach_min_m', 0.22).value)
        self.reach_max = float(p('arm.reach_max_m', 0.40).value)
        self.elbow_bent = math.radians(float(p('arm.elbow_bent_deg', 60.0).value))
        self.elbow_straight = math.radians(float(p('arm.elbow_straight_deg', 170.0).value))
        self.wrist_map = {
            j: (float(p(f'arm.{j}_sign', 1.0).value), float(p(f'arm.{j}_gain', 1.0).value),
                float(p(f'arm.{j}_offset', 0.0).value))
            for j in ('wrist_flex', 'wrist_roll')}
        self.control_wrist = bool(p('arm.control_wrist', True).value)
        self.arm = None
        self.arm_time = 0.0
        self.arm_active = False
        self.clutch = ClutchMapping(
            gain_lateral=float(p('gain_lateral', 0.30).value),
            gain_vertical=float(p('gain_vertical', 0.30).value),
            gain_depth=float(p('gain_depth', 0.15).value),
            lateral_axis=int(p('lateral_axis', 1).value),
            lateral_sign=float(p('lateral_sign', -1.0).value),
            vertical_axis=int(p('vertical_axis', 2).value),
            vertical_sign=float(p('vertical_sign', -1.0).value),
            depth_axis=int(p('depth_axis', 0).value),
            depth_sign=float(p('depth_sign', 1.0).value),
        )

        urdf = p('urdf_path', '').value or os.path.join(
            get_package_share_directory('arm_movement'), 'Viz_asset', 'SO101',
            'so101_new_calib.urdf')
        self.limits = load_urdf_limits(urdf)
        import placo
        self.robot = placo.RobotWrapper(urdf, placo.Flags.ignore_collisions)
        self.ik_cols = [self.robot.get_joint_v_offset(j) for j in IK_JOINTS]

        self.q = None
        self.hand = None
        self.hand_time = 0.0
        self.openness = None
        self.engaged_flag = False
        self.engaged_time = 0.0
        self.target = None
        self.start_time = time.monotonic()

        self.pub = self.create_publisher(JointState, p('output_topic', '/joint_states').value, 10)
        self.target_pub = self.create_publisher(PointStamped, '/hand_follower/target', 10)
        self.create_subscription(PointStamped, '/hand/position', self._on_hand, 10)
        self.create_subscription(Float32, '/hand/openness', self._on_open, 10)
        self.create_subscription(Bool, '/hand/engaged', self._on_engaged, 10)
        self.create_subscription(JointState, '/arm/pose', self._on_arm, 10)
        self.create_subscription(JointState, p('seed_topic', '/lerobot/joint_states').value,
                                 self._on_seed, 10)
        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f'Follower ready ({self.mode} mode): open hand to move, pinch for the gripper, '
            'fist to pause.')

    # ------------------------------------------------------------------ inputs
    def _on_hand(self, msg):
        self.hand = (msg.point.x, msg.point.y, msg.point.z)
        self.hand_time = time.monotonic()

    def _on_open(self, msg):
        self.openness = clamp(msg.data, 0.0, 1.0)

    def _on_engaged(self, msg):
        self.engaged_flag = msg.data
        self.engaged_time = time.monotonic()

    def _on_arm(self, msg):
        self.arm = dict(zip(msg.name, msg.position))
        self.arm_time = time.monotonic()

    def _on_seed(self, msg):
        if self.q is not None:
            return
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in ALL_JOINTS):
            self.q = {j: clamp(pos[j], *self.limits[j]) for j in ALL_JOINTS}
            self.get_logger().info('Starting from the real arm pose.')

    # ------------------------------------------------------------------ kinematics
    def _fk(self):
        for j, v in self.q.items():
            self.robot.set_joint(j, v)
        self.robot.update_kinematics()
        return self.robot.get_T_a_b('world', TIP_FRAME)[:3, 3].copy()

    def _ik_step(self, target, dt):
        tip = self._fk()
        err = np.asarray(target) - tip
        if np.linalg.norm(err) < self.pos_tol:
            return
        v = self.kp * err
        n = np.linalg.norm(v)
        if n > self.max_linear_vel:
            v *= self.max_linear_vel / n
        J = self.robot.frame_jacobian(TIP_FRAME, 'local_world_aligned')[:3, self.ik_cols]
        qdot = damped_least_squares(J, v, self.ik_damping)
        peak = np.max(np.abs(qdot))
        if peak > self.max_joint_vel:          # keep direction, scale magnitude
            qdot *= self.max_joint_vel / peak
        for j, w in zip(IK_JOINTS, qdot):
            self.q[j] = clamp(self.q[j] + float(w) * dt, *self.limits[j])

    # ------------------------------------------------------------------ loop
    def _tick(self):
        now = time.monotonic()
        if self.q is None:
            if now - self.start_time < self.seed_timeout_s:
                return
            self.q = {j: clamp(v, *self.limits[j]) for j, v in zip(ALL_JOINTS, self.home_pose)}
            self.get_logger().warn('No real-arm pose received; starting from home_pose '
                                   '(fine for sim only).')

        dt = 1.0 / self.rate_hz
        if self.mode == 'arm':
            self._tick_arm(now, dt)
        else:
            self._tick_hand(now, dt)
        self._publish()

    def _step_gripper(self, dt):
        if self.control_gripper and self.openness is not None:
            lo_g, hi_g = self.limits[GRIPPER]
            goal = lo_g + self.openness * (hi_g - lo_g)
            self.q[GRIPPER] = step_towards(self.q[GRIPPER], goal, self.max_gripper_vel * dt)

    def arm_target(self, arm):
        """Robot tip target (world frame) from the arm features (elbow = base)."""
        u = map_axes([arm['forearm_x'], arm['forearm_y'], arm['forearm_z']], self.axis_map)
        elbow = arm.get('elbow_angle', float('nan'))
        if self.reach_source == 'elbow_angle' and not math.isnan(elbow):
            r = reach_from_elbow(elbow, self.reach_min, self.reach_max,
                                 self.elbow_bent, self.elbow_straight)
        else:
            r = self.reach_fixed
        return np.array(clamp_box(self.anchor + r * u, self.box_lo, self.box_hi))

    def _tick_arm(self, now, dt):
        active = (self.engaged_flag and self.arm is not None
                  and now - self.arm_time < self.hand_timeout_s
                  and now - self.engaged_time < self.hand_timeout_s)
        if active != self.arm_active:
            self.arm_active = active
            self.get_logger().info('Engaged: arm follows your forearm.' if active else
                                   'Paused (fist / arm or hand lost).')
            if not active:
                self.target = None
        if not active:
            return
        # Absolute mapping: the speed limits in _ik_step / step_towards make a big jump
        # (e.g. after a pause) a glide rather than a snap.
        self.target = self.arm_target(self.arm)
        self._ik_step(self.target, dt)
        if self.control_wrist:
            for j, (sign, gain, offset) in self.wrist_map.items():
                goal = clamp(sign * gain * self.arm[j] + offset, *self.limits[j])
                self.q[j] = step_towards(self.q[j], goal, self.max_joint_vel * dt)
        self._step_gripper(dt)

    def _tick_hand(self, now, dt):
        active = (self.engaged_flag and self.hand is not None
                  and now - self.hand_time < self.hand_timeout_s
                  and now - self.engaged_time < self.hand_timeout_s)

        if active and not self.clutch.engaged:
            self.clutch.engage(self.hand, self._fk())
            self.get_logger().info('Engaged: tip follows your hand.')
        elif not active and self.clutch.engaged:
            self.clutch.release()
            self.target = None
            self.get_logger().info('Paused (fist / hand lost).')

        if self.clutch.engaged:
            tip = self._fk()
            raw = np.array(self.clutch.target(self.hand))
            # Never pull the arm toward the box if it started outside it; only stop
            # it from going further out.
            lo = np.minimum(self.box_lo, self.clutch.tip_ref)
            hi = np.maximum(self.box_hi, self.clutch.tip_ref)
            target = np.array(clamp_box(raw, lo, hi))
            # Leash: if the target runs too far ahead (unreachable / arm at its limits),
            # drag the reference along so the arm responds as soon as the hand comes back.
            lead = target - tip
            dist = np.linalg.norm(lead)
            if dist > self.max_lead:
                shift = lead * (1.0 - self.max_lead / dist)
                self.clutch.tip_ref = list(np.array(self.clutch.tip_ref) - shift)
                target = target - shift
            self.target = target
            self._ik_step(target, dt)
            self._step_gripper(dt)

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(ALL_JOINTS)
        msg.position = [float(self.q[j]) for j in ALL_JOINTS]
        self.pub.publish(msg)
        if self.target is not None:
            t = PointStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'world'
            t.point.x, t.point.y, t.point.z = (float(x) for x in self.target)
            self.target_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = HandFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
        # else: Ctrl+C invalidated the context mid-spin; a normal shutdown.
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
