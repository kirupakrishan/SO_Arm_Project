import os
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import placo
from scipy.spatial.transform import Rotation as R

# The gripper (finger) joint is attached to gripper_link via a fixed joint,
# not gripper_frame_link, so it has no effect on end-effector pose and must
# not be part of the differential IK unknowns.
GRIPPER_JOINT = 'gripper'


class DiffInvKinematicsNode(Node):
    """Resolved-rate (Jacobian pseudo-inverse) inverse differential kinematics.

    Continuously drives the end-effector toward the latest target pose by
    integrating joint velocities computed from the position/orientation
    error, rather than solving a one-shot nonlinear IK problem.
    """

    def __init__(self):
        super().__init__('inverse_diff_kinematics_node')

        self.declare_parameter('target_position', [0.2, 0.3, 0.3])
        self.declare_parameter('target_orientation', [0.0, 0.0, np.pi])
        self.declare_parameter('kp_position', 2.0)
        self.declare_parameter('kp_orientation', 2.0)
        self.declare_parameter('joint_velocity_limit', 1.0)  # rad/s
        self.declare_parameter('damping', 0.05)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('position_tolerance', 0.002)  # m
        self.declare_parameter('orientation_tolerance', 0.01)  # rad

        self.target_position = np.array(
            self.get_parameter('target_position').value, dtype=np.float64)
        self.target_rotation = R.from_euler(
            'xyz', self.get_parameter('target_orientation').value).as_matrix()

        self.kp_pos = self.get_parameter('kp_position').value
        self.kp_ori = self.get_parameter('kp_orientation').value
        self.qdot_limit = self.get_parameter('joint_velocity_limit').value
        self.damping = self.get_parameter('damping').value
        self.pos_tol = self.get_parameter('position_tolerance').value
        self.ori_tol = self.get_parameter('orientation_tolerance').value
        self.dt = 1.0 / self.get_parameter('rate_hz').value

        self.publisher = self.create_publisher(JointState, '/diff_ik_joint_states', 10)
        self.target_sub = self.create_subscription(
            PoseStamped, '/diff_ik_target_pose', self.target_pose_callback, 10)

        urdf_path = os.path.join(
            get_package_share_directory('arm_movement'),
            'Viz_asset', 'SO101', 'so101_new_calib.urdf'
        )
        self.robot = placo.RobotWrapper(urdf_path, placo.Flags.ignore_collisions)
        self.joint_names = list(self.robot.joint_names())
        self.ik_joint_names = [j for j in self.joint_names if j != GRIPPER_JOINT]
        # Column indices into the tangent-space Jacobian for our actuated joints
        self.ik_v_offsets = [self.robot.get_joint_v_offset(j) for j in self.ik_joint_names]
        self.lower_bounds = np.array(
            [self.robot.get_joint_limits(j)[0] for j in self.ik_joint_names])
        self.upper_bounds = np.array(
            [self.robot.get_joint_limits(j)[1] for j in self.ik_joint_names])

        self.q = np.zeros(len(self.ik_joint_names))
        self.set_joint_angles(self.ik_joint_names, self.q)
        self.converged = False

        self.timer = self.create_timer(self.dt, self.control_step)

    def target_pose_callback(self, msg):
        self.target_position = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        quat = [msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w]
        self.target_rotation = R.from_quat(quat).as_matrix()
        self.converged = False
        # self.get_logger().info(f'New target pose received: position={self.target_position}, orientation={quat}')

    def control_step(self):
        # self.get_logger().info('Performing control step')
        T = self.robot.get_T_a_b('world', 'gripper_frame_link')
        p = T[:3, 3]
        Rot = T[:3, :3]

        pos_error = self.target_position - p
        # World-frame rotation vector that maps current orientation to target
        ori_error = R.from_matrix(self.target_rotation @ Rot.T).as_rotvec()

        if np.linalg.norm(pos_error) < self.pos_tol and np.linalg.norm(ori_error) < self.ori_tol:
            qdot = np.zeros(len(self.ik_joint_names))
            if not self.converged:
                self.get_logger().info('Target pose reached')
                self.converged = True
        else:
            self.converged = False
            twist = np.hstack([self.kp_pos * pos_error, self.kp_ori * ori_error])
            # World-aligned Jacobian: rows are [linear; angular] velocity of
            # the frame origin directly in world axes, matching the errors above.
            J = self.robot.frame_jacobian('gripper_frame_link', 'local_world_aligned')
            J_ik = J[:, self.ik_v_offsets]
            # Damped least squares instead of a plain pseudo-inverse: this arm
            # only has 5 actuated joints for a 6D pose target, so it passes
            # near Jacobian singularities (and some targets are only
            # approximately reachable) where an undamped pinv blows up and
            # makes the joints chatter between clipped velocity limits
            # instead of settling.
            JJt = J_ik @ J_ik.T
            qdot = J_ik.T @ np.linalg.solve(
                JJt + self.damping ** 2 * np.eye(6), twist)
            qdot = np.clip(qdot, -self.qdot_limit, self.qdot_limit)

        self.q = np.clip(self.q + qdot * self.dt, self.lower_bounds, self.upper_bounds)
        self.set_joint_angles(self.ik_joint_names, self.q)

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        positions = dict(zip(self.ik_joint_names, self.q.tolist()))
        positions[GRIPPER_JOINT] = self.robot.get_joint(GRIPPER_JOINT)
        joint_state_msg.position = [positions[name] for name in self.joint_names]
        velocities = dict(zip(self.ik_joint_names, qdot.tolist()))
        velocities[GRIPPER_JOINT] = 0.0
        joint_state_msg.velocity = [velocities[name] for name in self.joint_names]
        self.publisher.publish(joint_state_msg)

    def set_joint_angles(self, joint_names, joint_angles):
        for name, q in zip(joint_names, joint_angles):
            try:
                self.robot.set_joint(name, float(q))
            except Exception:
                self.get_logger().error(f'Failed to set joint {name} to angle {q}')
        self.robot.update_kinematics()


def main(args=None):
    rclpy.init(args=args)
    node = DiffInvKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
