"""RViz sim (joint_state_publisher_gui + robot_state_publisher) + real SO-101 following it."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    viz = os.path.join(get_package_share_directory('arm_movement'), 'launch', 'arm_viz.launch.py')
    bridge = os.path.join(get_package_share_directory('lerobot_real'), 'launch',
                          'follower_bridge.launch.py')

    forwarded = ['lerobot_env', 'port', 'robot_id', 'dry_run']
    return LaunchDescription([
        SetLaunchConfiguration('sigterm_timeout', '30'),  # time for the rest move on Ctrl+C
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('robot_id', default_value='vellayan'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('lerobot_env',
                              default_value=os.path.expanduser('~/miniforge3/envs/lerobot')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(viz)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge),
            launch_arguments={k: LaunchConfiguration(k) for k in forwarded}.items()),
    ])
