"""Webcam hand teleop of the SO-101.

    ros2 launch arm_vision_teleop hand_teleop.launch.py              # sim (RViz)
    ros2 launch arm_vision_teleop hand_teleop.launch.py real:=true   # + real arm
    ros2 launch arm_vision_teleop hand_teleop.launch.py mode:=arm    # forearm, elbow = base

Then on Windows:  py cam_streamer.py   (see README). Starts camera_bridge and
hand_tracker (in the .venv_vision interpreter: mediapipe/OpenCV), hand_follower
(system Python: placo), robot_state_publisher, RViz and, with real:=true,
lerobot_real's follower_bridge. The tracker opens its own camera preview window.
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
        get_package_share_directory('arm_vision_teleop'), 'config', 'hand_teleop.yaml')
    params = LaunchConfiguration('params_file')
    venv = LaunchConfiguration('vision_env')
    real = LaunchConfiguration('real')

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('vision_env',
                              default_value=os.path.expanduser('~/RobotArm/.venv_vision'),
                              description='venv with mediapipe + opencv'),
        DeclareLaunchArgument('mode', default_value='hand',
                              description='hand (clutch, hand only) | arm (elbow = base)'),
        DeclareLaunchArgument('camera_source', default_value='tcp',
                              description='tcp (Windows cam_streamer) | local (/dev/video)'),
        DeclareLaunchArgument('port', default_value='5005', description='cam_streamer port'),
        DeclareLaunchArgument('real', default_value='false',
                              description='true = also drive the real arm via lerobot_real'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('image_view', default_value='true',
                              description='camera preview window with hand overlay'),
        # Forwarded to lerobot_real when real:=true
        DeclareLaunchArgument('arm_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('robot_id', default_value='vellayan'),
        DeclareLaunchArgument('lerobot_env',
                              default_value=os.path.expanduser('~/miniforge3/envs/lerobot')),
        DeclareLaunchArgument('dry_run', default_value='false'),
    ]

    venv_python = [venv, '/bin/python']
    camera = Node(
        package='arm_vision_teleop', executable='camera_bridge', name='camera_bridge',
        output='screen', emulate_tty=True, prefix=venv_python,
        additional_env={'PYTHONUNBUFFERED': '1'},
        parameters=[params, {
            'source': LaunchConfiguration('camera_source'),
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int)}],
    )
    tracker = Node(
        package='arm_vision_teleop', executable='hand_tracker', name='hand_tracker',
        output='screen', emulate_tty=True, prefix=venv_python,
        # Quiet MediaPipe/TFLite's C++ logging.
        additional_env={'PYTHONUNBUFFERED': '1', 'GLOG_minloglevel': '2',
                        'TF_CPP_MIN_LOG_LEVEL': '2'},
        parameters=[params, {'show_window': ParameterValue(LaunchConfiguration('image_view'),
                                                           value_type=bool),
                             'mode': LaunchConfiguration('mode')}],
    )
    follower = Node(
        package='arm_vision_teleop', executable='hand_follower', name='hand_follower',
        output='screen', emulate_tty=True,
        parameters=[params, {'urdf_path': urdf_path, 'mode': LaunchConfiguration('mode')}],
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
    actions = args + [camera, tracker, follower, rsp, rviz]
    try:
        bridge = os.path.join(get_package_share_directory('lerobot_real'), 'launch',
                              'follower_bridge.launch.py')
        actions += [
            SetLaunchConfiguration('sigterm_timeout', '30', condition=IfCondition(real)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge), condition=IfCondition(real),
                launch_arguments={'port': LaunchConfiguration('arm_port'),
                                  'robot_id': LaunchConfiguration('robot_id'),
                                  'lerobot_env': LaunchConfiguration('lerobot_env'),
                                  'dry_run': LaunchConfiguration('dry_run')}.items()),
        ]
    except Exception:
        pass
    return LaunchDescription(actions)
