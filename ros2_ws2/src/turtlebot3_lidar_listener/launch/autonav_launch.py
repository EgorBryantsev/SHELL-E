#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('turtlebot3_lidar_listener')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # --- File Paths ---
    # Path to your map file
    map_yaml_file = os.path.join(pkg_share, 'maps', 'map.yaml')
    # Path to the Nav2 parameters file
    nav2_params_file = os.path.join(pkg_share, 'params', 'nav2_params.yaml')

    # --- Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # --- Your Custom Nodes ---
    
    # 1. LidarListener Node
    lidar_listener_node = Node(
        package='turtlebot3_lidar_listener',
        executable='lidar_listener',
        name='lidar_listener',
        output='screen'
    )
    
    # 2. ObstacleDetection Node
    # We remap 'cmd_vel_raw' to '/cmd_vel_nav' so it listens to Nav2's output.
    # It will then publish the final, safety-checked command to '/cmd_vel'.
    obstacle_detection_node = Node(
        package='turtlebot3_lidar_listener',
        executable='turtlebot3_obstacle_detection',
        name='turtlebot3_obstacle_detection',
        output='screen',
        remappings=[
            ('cmd_vel_raw', '/cmd_vel_nav')
        ]
    )

    # --- Nav2 Bringup ---
    # This includes the entire Nav2 stack (map_server, amcl, planner, controller, etc.)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            # Remap the controller's output to '/cmd_vel_nav' to be intercepted by our safety node
            'autostart': 'true',
            'cmd_vel_topic': '/cmd_vel_nav' 
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # Start your custom nodes
        lidar_listener_node,
        obstacle_detection_node,
        
        # Start the Nav2 stack
        nav2_bringup,
    ])

