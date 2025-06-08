#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, TextSubstitution # Added TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    LDS_MODEL = os.environ['LDS_MODEL']
    LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    # NEW: Declare a launch argument for the robot namespace
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value=TextSubstitution(text=''), # Default to empty for no namespace
        description='Namespace for the robot topics (e.g., real_robot, virtual_robot)'
    )
    robot_namespace = LaunchConfiguration('robot_namespace')

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            TURTLEBOT3_MODEL + '.yaml'))

    if LDS_MODEL == 'LDS-01':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))
    elif LDS_MODEL == 'LDS-02':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('ld08_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/ld08.launch.py'
    else:
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        # NEW: Add the robot_namespace argument declaration
        robot_namespace_arg,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time}.items(),
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
        #     launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        # ),

        # Node(
        #     package='turtlebot3_node',
        #     executable='turtlebot3_ros',
        #     parameters=[tb3_param_dir],
        #     arguments=['-i', usb_port],
        #     output='screen'),

        # MODIFIED: Include my_turtlebot3_state_publisher with namespace
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/my_turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'robot_namespace': robot_namespace # Pass namespace
                             }.items(),
        ),

        # MODIFIED: Include LiDAR driver with namespace and remapping
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyUSB0', 
                              'frame_id': 'base_scan',
                              'namespace': robot_namespace # Pass namespace to lidar driver launch
                             }.items(),
            # Additionally, remap the '/scan' topic to '/<robot_namespace>/scan'
            remappings=[
                ('/scan', [robot_namespace, '/scan'])
            ]
        ),

        # MODIFIED: turtlebot3_node with namespace
        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen',
            namespace=robot_namespace, # Apply namespace to this node
            remappings=[ # Remap common topics handled by turtlebot3_ros
                ('/cmd_vel', [robot_namespace, '/cmd_vel']),
                ('/odom', [robot_namespace, '/odom'])
            ]
        ),
    ])
