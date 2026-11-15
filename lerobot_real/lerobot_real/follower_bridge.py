"""ROS 2 node: make a real LeRobot SO-101 follower track /joint_states.

Subscribes to ``/joint_states`` (published by joint_state_publisher(_gui) or the
IK nodes in arm_movement, and rendered in RViz by robot_state_publisher) and
streams those joint targets to the real arm through LeRobot's SO101Follower.
The measured arm state is published back on ``/lerobot/joint_states``.

Must run with a Python interpreter that can import ``lerobot`` (the `lerobot`
conda env) -- the launch files in this package take care of that.
"""

import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from lerobot_real.arm import (
    connect_so101, declare_arm_params, read_present, retry, to_joint_state,
)
from lerobot_real.mapping import ALL_JOINTS, GRIPPER, step_towards
from lerobot_real.rest import DEFAULT_REST_FILE, RestAborted, load_rest_pose, move_to_pose


class FollowerBridge(Node):

    def __init__(self):
        super().__init__('lerobot_follower_bridge')

        self.arm = declare_arm_params(self)
        p = self.declare_parameter
        self.rate_hz = float(p('rate_hz', 30.0).value)
        self.max_vel_deg_s = float(p('max_velocity_deg_s', 90.0).value)
        self.max_gripper_pct_s = float(p('max_gripper_pct_s', 150.0).value)
        self.cmd_timeout_s = float(p('command_timeout_s', 1.0).value)
        self.dry_run = bool(p('dry_run', False).value)
        self.disable_torque_on_exit = bool(p('disable_torque_on_exit', True).value)
        input_topic = p('input_topic', '/joint_states').value
        feedback_topic = p('feedback_topic', '/lerobot/joint_states').value
        self.rest_on_exit = bool(p('go_to_rest_on_exit', True).value)
        self.rest_speed = float(p('rest_speed_deg_s', 30.0).value)
        self.rest_file = p('rest_pose_file', DEFAULT_REST_FILE).value

        self.robot = connect_so101(self, self.arm, self.disable_torque_on_exit)

        if self.dry_run:
            self.robot.bus.disable_torque()
            self.get_logger().warn('dry_run=true: torque OFF, only publishing feedback.')

        # Start the command from wherever the arm physically is, so the first
        # motion is a rate-limited ramp, not a jump.
        self.current_cmd = retry(self, lambda: read_present(self.robot), 'initial position read', 5)
        self.target = None          # latest target in lerobot units
        self.last_msg_time = None

        self.fb_pub = self.create_publisher(JointState, feedback_topic, 10)
        self.create_subscription(JointState, input_topic, self._on_joint_states,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            f'Connected to SO-101 "{self.arm.robot_id}" on {self.arm.port}. '
            f'Following {input_topic} at {self.rate_hz:.0f} Hz, '
            f'max {self.max_vel_deg_s:.0f} deg/s. Feedback on {feedback_topic}.')

    # ------------------------------------------------------------------
    def _on_joint_states(self, msg: JointState):
        if not msg.position:
            return
        target = dict(self.target) if self.target else dict(self.current_cmd)
        got = False
        for name, pos in zip(msg.name, msg.position):
            if name in self.arm.sign:
                target[name] = self.arm.to_lerobot(name, pos)
                got = True
        if got:
            if self.target is None:
                self.get_logger().info('First joint target received; arm will ramp to it.')
            self.target = target
            self.last_msg_time = time.monotonic()

    def _tick(self):
        try:
            present = read_present(self.robot)
        except Exception as e:  # serial glitch: skip this cycle
            self.get_logger().warn(f'read failed: {e}', throttle_duration_sec=2.0)
            present = None

        if present is not None:
            self.fb_pub.publish(to_joint_state(self, self.arm, present))

        if self.dry_run or self.target is None:
            return
        if time.monotonic() - self.last_msg_time > self.cmd_timeout_s:
            # Source stopped publishing; just hold the last commanded pose.
            return

        dt = 1.0 / self.rate_hz
        for j in ALL_JOINTS:
            max_step = (self.max_gripper_pct_s if j == GRIPPER else self.max_vel_deg_s) * dt
            self.current_cmd[j] = step_towards(self.current_cmd[j], self.target[j], max_step)

        try:
            self.robot.send_action({f'{j}.pos': v for j, v in self.current_cmd.items()})
        except Exception as e:
            self.get_logger().warn(f'write failed: {e}', throttle_duration_sec=2.0)

    def shutdown(self):
        if not self.robot.is_connected:
            return
        if self.rest_on_exit and not self.dry_run:
            # print(), not the ROS logger: rclpy may already be shut down here.
            say = lambda m: print(f'[lerobot_follower_bridge] {m}', flush=True)  # noqa: E731
            try:
                move_to_pose(self.robot.bus, load_rest_pose(self.rest_file),
                             speed_deg_s=self.rest_speed, log=say)
            except RestAborted as e:
                say(f'Rest move aborted ({e}); turning torque off now.')
                self.robot.bus.disable_torque()
            except Exception as e:
                say(f'Rest move failed ({e}); turning torque off now.')
                try:
                    self.robot.bus.disable_torque()
                except Exception:
                    pass
        if self.robot.is_connected:
            print('[lerobot_follower_bridge] Disconnecting arm'
                  + (' (torque off)' if self.disable_torque_on_exit else ''), flush=True)
            self.robot.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FollowerBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl+C -> finally: go to rest, then torque off
    except Exception as e:
        if not rclpy.ok():
            # Ctrl+C invalidated the ROS context mid-callback (e.g. during a
            # publish). That's a normal shutdown, not a crash.
            return
        import traceback
        traceback.print_exc()
        rclpy.logging.get_logger('lerobot_follower_bridge').fatal(
            f'Bridge stopped: {e}  (/lerobot/joint_states will NOT be published)')
        raise SystemExit(1)
    finally:
        # A duplicate Ctrl+C must not interrupt the cleanup below (rest move / torque off).
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
