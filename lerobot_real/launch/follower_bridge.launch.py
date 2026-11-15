"""Run only the real-arm bridge (bring your own /joint_states source + RViz)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('lerobot_real'), 'config', 'so101_follower.yaml')
    env = LaunchConfiguration('lerobot_env')

    args = [
        DeclareLaunchArgument('lerobot_env',
                              default_value=os.path.expanduser('~/miniforge3/envs/lerobot'),
                              description='Conda env that has lerobot installed (Python 3.10)'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('robot_id', default_value='vellayan'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true = torque off, only publish feedback'),
        DeclareLaunchArgument('input_topic', default_value='/joint_states'),
    ]

    bridge = Node(
        package='lerobot_real',
        executable='follower_bridge',
        name='lerobot_follower_bridge',
        output='screen',
        emulate_tty=True,
        # Run the entry-point script with the lerobot env's interpreter. ROS Humble's
        # rclpy works with it because both are CPython 3.10.
        prefix=[env, '/bin/python'],
        # Conda's _ssl needs its own libcrypto, which ROS's LD_LIBRARY_PATH would shadow.
        additional_env={'PYTHONUNBUFFERED': '1',
                        'LD_LIBRARY_PATH': [env, '/lib:',
                                            EnvironmentVariable('LD_LIBRARY_PATH', default_value='')]},
        parameters=[params_file, {
            'port': LaunchConfiguration('port'),
            'robot_id': LaunchConfiguration('robot_id'),
            'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
            'input_topic': LaunchConfiguration('input_topic'),
        }],
    )
    # Give the bridge time to drive the arm to rest after Ctrl+C before launch
    # escalates to SIGTERM (default is only 5 s).
    return LaunchDescription([SetLaunchConfiguration('sigterm_timeout', '30')] + args + [bridge])
