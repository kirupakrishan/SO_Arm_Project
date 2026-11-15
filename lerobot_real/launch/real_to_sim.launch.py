"""Real SO-101 -> RViz: the sim mirrors the physical arm (move it by hand).

Starts robot_state_publisher + RViz from arm_movement (WITHOUT
joint_state_publisher_gui, which would fight over /joint_states) and the
real_to_sim node, which publishes the arm's measured joints on /joint_states.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_share = get_package_share_directory('arm_movement')
    urdf_path = os.path.join(arm_share, 'Viz_asset', 'SO101', 'so101_new_calib.urdf')
    rviz_config = os.path.join(arm_share, 'resource', 'arm.rviz')
    params_file = os.path.join(
        get_package_share_directory('lerobot_real'), 'config', 'so101_follower.yaml')
    env = LaunchConfiguration('lerobot_env')

    args = [
        DeclareLaunchArgument('lerobot_env',
                              default_value=os.path.expanduser('~/miniforge3/envs/lerobot'),
                              description='Conda env that has lerobot installed (Python 3.10)'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('robot_id', default_value='vellayan'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(Command(['cat ', urdf_path]),
                                                         value_type=str)}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    mirror = Node(
        package='lerobot_real',
        executable='real_to_sim',
        name='lerobot_real_to_sim',
        output='screen',
        emulate_tty=True,
        # Same interpreter trick as follower_bridge.launch.py (lerobot lives in conda).
        prefix=[env, '/bin/python'],
        additional_env={'PYTHONUNBUFFERED': '1',
                        'LD_LIBRARY_PATH': [env, '/lib:',
                                            EnvironmentVariable('LD_LIBRARY_PATH', default_value='')]},
        parameters=[params_file, {
            'port': LaunchConfiguration('port'),
            'robot_id': LaunchConfiguration('robot_id'),
        }],
    )
    return LaunchDescription(args + [rsp, rviz, mirror])
