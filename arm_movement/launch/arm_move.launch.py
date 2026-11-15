import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('arm_movement')
    urdf_path = os.path.join(pkg_share, 'Viz_asset', 'SO101', 'so101_new_calib.urdf')
    rviz_config = os.path.join(pkg_share, 'resource', 'arm.rviz')

    robot_description = ParameterValue(Command(['cat ', urdf_path]), value_type=str)

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Drives the arm via resolved-rate (Jacobian pseudo-inverse) control.
    # Remap its output topic to /joint_states so robot_state_publisher
    # picks it up directly for the TF tree / RViz visualization, since it
    # publishes to /diff_ik_joint_states by default.
    diff_ik_node = Node(
        package='arm_movement',
        executable='inverse_diff_kinematics',  # match the console_scripts entry point in setup.py
        output='screen',
        remappings=[('/diff_ik_joint_states', '/joint_states')]
    )

    return LaunchDescription([rsp_node, rviz_node, diff_ik_node])