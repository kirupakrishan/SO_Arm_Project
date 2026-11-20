"""Gamepad teleop of the SO-101: joy_node + arm_joy_teleop + robot_state_publisher + RViz.

    ros2 launch arm_joy_teleop joy_teleop.launch.py              # sim only (RViz)
    ros2 launch arm_joy_teleop joy_teleop.launch.py real:=true   # also drive the real arm

With real:=true, lerobot_real's follower_bridge follows /joint_states, and the teleop
starts from the arm's measured pose (/lerobot/joint_states) so there's no jump.
joint_state_publisher_gui is NOT started: it would fight the teleop over /joint_states.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_share = get_package_share_directory('arm_movement')
    urdf_path = os.path.join(arm_share, 'Viz_asset', 'SO101', 'so101_new_calib.urdf')
    rviz_config = os.path.join(arm_share, 'resource', 'arm.rviz')
    default_params = os.path.join(
        get_package_share_directory('arm_joy_teleop'), 'config', 'gamepad.yaml')
    params = LaunchConfiguration('params_file')
    real = LaunchConfiguration('real')

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Button/axis mapping + teleop tuning'),
        DeclareLaunchArgument('device_id', default_value='0', description='Joystick index'),
        DeclareLaunchArgument('real', default_value='false',
                              description='true = also drive the real arm via lerobot_real'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('start_mode', default_value='joint',
                              description='joint | cartesian'),
        # Forwarded to lerobot_real/follower_bridge.launch.py when real:=true
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('robot_id', default_value='vellayan'),
        DeclareLaunchArgument('lerobot_env',
                              default_value=os.path.expanduser('~/miniforge3/envs/lerobot')),
        DeclareLaunchArgument('dry_run', default_value='false'),
    ]

    joy = Node(
        package='joy', executable='joy_node', name='joy_node', output='screen',
        parameters=[params, {'device_id': ParameterValue(LaunchConfiguration('device_id'),
                                                         value_type=int)}],
    )
    teleop = Node(
        package='arm_joy_teleop', executable='joy_teleop', name='arm_joy_teleop',
        output='screen', emulate_tty=True,
        parameters=[params, {'urdf_path': urdf_path,
                             'start_mode': LaunchConfiguration('start_mode')}],
    )
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': ParameterValue(Command(['cat ', urdf_path]),
                                                         value_type=str)}],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_config], output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    actions = args + [joy, teleop, rsp, rviz]
    # Only resolve lerobot_real when it's actually requested, so sim-only use
    # works even if that package isn't built.
    try:
        bridge = os.path.join(get_package_share_directory('lerobot_real'), 'launch',
                              'follower_bridge.launch.py')
        actions += [
            # Gives the bridge time to drive the arm to rest after Ctrl+C.
            SetLaunchConfiguration('sigterm_timeout', '30', condition=IfCondition(real)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge), condition=IfCondition(real),
                launch_arguments={k: LaunchConfiguration(k) for k in
                                  ['port', 'robot_id', 'lerobot_env', 'dry_run']}.items()),
        ]
    except Exception:
        pass
    return LaunchDescription(actions)
