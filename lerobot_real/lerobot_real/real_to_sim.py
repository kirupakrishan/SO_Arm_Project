"""ROS 2 node: make the RViz model follow the real SO-101 (real -> sim).

The opposite of follower_bridge. Turns the arm's torque OFF so you can move it
by hand, reads the motor positions, and publishes them as URDF joint angles on
``/joint_states``. robot_state_publisher turns that into TF and RViz shows the
real arm's pose.

Do NOT run joint_state_publisher(_gui) or follower_bridge at the same time:
they would also publish / command /joint_states.
"""

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from lerobot_real.arm import connect_so101, declare_arm_params, read_present, to_joint_state


class RealToSim(Node):

    def __init__(self):
        super().__init__('lerobot_real_to_sim')
        self.arm = declare_arm_params(self)
        p = self.declare_parameter
        rate_hz = float(p('rate_hz', 30.0).value)
        output_topic = p('output_topic', '/joint_states').value
        # Real arm can sit slightly past the URDF limits (e.g. folded at rest).
        self.clamp = bool(p('clamp_to_urdf_limits', False).value)

        self.robot = connect_so101(self, self.arm, disable_torque_on_disconnect=True)
        self.get_logger().warn('Torque OFF - the arm is limp; support it if it is not resting.')
        self.robot.bus.disable_torque()

        self.pub = self.create_publisher(JointState, output_topic, 10)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'Mirroring real SO-101 "{self.arm.robot_id}" on {self.arm.port} -> '
            f'{output_topic} at {rate_hz:.0f} Hz. Move the arm by hand.')

    def _tick(self):
        try:
            present = read_present(self.robot)
        except Exception as e:  # serial glitch: skip this cycle
            self.get_logger().warn(f'read failed: {e}', throttle_duration_sec=2.0)
            return
        self.pub.publish(to_joint_state(self, self.arm, present, self.clamp))

    def shutdown(self):
        if self.robot.is_connected:
            print('[lerobot_real_to_sim] Disconnecting arm (torque stays off)', flush=True)
            self.robot.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RealToSim()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        if not rclpy.ok():
            # Ctrl+C invalidated the ROS context mid-callback (e.g. during a
            # publish). That's a normal shutdown, not a crash.
            return
        import traceback
        traceback.print_exc()
        rclpy.logging.get_logger('lerobot_real_to_sim').fatal(f'real_to_sim stopped: {e}')
        raise SystemExit(1)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
