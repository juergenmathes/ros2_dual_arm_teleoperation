from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    # Get the path to the robot description
    teleop_pkg_share = FindPackageShare('teleop_arm_control').find('teleop_arm_control')
    urdf_path = os.path.join(teleop_pkg_share, 'urdf', 'robot_arm.urdf.xacro')

    # Get URDF via xacro
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Joint Publisher Node (simplified control without MoveIt)
    joint_publisher_node = Node(
        package='teleop_arm_control',
        executable='joint_publisher_node',
        name='joint_publisher_node',
        output='screen',
    )

    # Teleoperation node
    teleoperation_node = Node(
        package='teleop_arm_control',
        executable='teleoperation_node',
        name='teleoperation_node',
        output='screen',
    )

    # Camera node
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        parameters=[{
            'video_device': '/dev/video0',
            'pixel_format': 'YUYV',
            'image_width': 640,
            'image_height': 480,
            'camera_frame_id': 'camera_link'
        }],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(teleop_pkg_share, 'config', 'simple_config.rviz')],
        output='screen'
    )

    # Launch Description
    return LaunchDescription([
        robot_state_publisher_node,
        joint_publisher_node,
        teleoperation_node,
        camera_node,
        rviz_node
    ])