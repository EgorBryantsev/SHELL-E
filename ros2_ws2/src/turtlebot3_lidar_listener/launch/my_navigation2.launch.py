#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    # New: Declare Launch Argument for Robot Namespace
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value=TextSubstitution(text=''), # Default to empty for no namespace, or 'robot'
        description='Namespace for the robot topics (e.g., real_robot, virtual_robot)'
    )
    robot_namespace = LaunchConfiguration('robot_namespace')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'turtlebot3_world.yaml'))

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        robot_namespace_arg, # Add the new argument declaration

        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # NEW: Include Nav2 Bringup (now with namespace)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'robot_namespace': robot_namespace # Pass the namespace to Nav2 bringup
                }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # NEW: Your LidarListener Node (now namespaced)
        # Ensure 'your_package_name' is replaced with your actual package name
        Node(
            package='your_package_name', 
            executable='lidar_listener_node', # Entry point name from setup.py
            name='lidar_listener',
            namespace=robot_namespace, # Apply the namespace
            output='screen',
            remappings=[
                # Remap '/scan' to be within the robot's namespace
                ('/scan', [robot_namespace, '/scan']), 
                # Remap 'closest_obstacle_distance' to be within the robot's namespace
                ('/closest_obstacle_distance', [robot_namespace, '/closest_obstacle_distance']),
                # Remap 'cluster_points' for RViz visualization within the namespace
                ('/cluster_points', [robot_namespace, '/cluster_points'])
            ]
        ),

        # NEW: Turtlebot3ObstacleDetection Node (now namespaced)
        Node(
            package='your_package_name', 
            executable='turtlebot3_obstacle_detection', # Entry point name from setup.py
            name='obstacle_avoidance',
            namespace=robot_namespace, # Apply the namespace
            output='screen',
            parameters=[common_avoidance_params], # Pass parameters
            remappings=[
                # The command velocity topic will now be namespaced
                ('/cmd_vel', [robot_namespace, '/cmd_vel']), 
                # If you use cmd_vel_raw for manual control override, it should also be namespaced
                ('/cmd_vel_raw', [robot_namespace, '/cmd_vel_raw']), 
                # Subscribe to the namespaced obstacle distance from your LidarListener
                ('/closest_obstacle_distance', [robot_namespace, '/closest_obstacle_distance'])
            ]
        )
    ])
