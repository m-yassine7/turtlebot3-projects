# ECE7425, Assignment 3 - Controller Launch File
# Launches the AprilTag detector and robot controller

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Find the package and its configuration files
    package_share = get_package_share_directory('matador_controller')
    detector_config = os.path.join(package_share, 'config', 'apriltag.yaml')
    controller_config = os.path.join(package_share, 'config', 'controller.yaml')

    # Launch settings for camera topics and simulation time
    image_topic = LaunchConfiguration('image_topic')
    info_topic = LaunchConfiguration('camera_info_topic')
    sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        # Default topic names and time setting
        DeclareLaunchArgument('image_topic', default_value='/camera/image_rect'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Start the AprilTag detection node
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            output='screen',
            parameters=[detector_config, {'use_sim_time': sim_time}],
           
            # Connect detector inputs to camera topics
            remappings=[
                ('image_rect', image_topic),
                ('camera_info', info_topic),
                ('detections', '/detections'),
            ],
        ),

        # Start the Python node that controls the robot
        Node(
            package='matador_controller',
            executable='tag_follower',
            name='tag_follower',
            output='screen',
            parameters=[controller_config, {'use_sim_time': sim_time}],
        ),
    ])
