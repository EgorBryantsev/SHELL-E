import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package name, which is the name of the directory
    pkg_name = 'my_robot_nav'

    return LaunchDescription([
        # --- 1. Lidar Listener Node ---
        # Takes raw /scan and publishes clustered points and closest distance
        Node(
            package=pkg_name,
            executable='lidar_listener_node.py',
            name='lidar_listener',
            output='screen',
            # Parameters for the lidar listener are now correctly defined in your python script
            # You can override them here if you wish, for example:
            # parameters=[
            #     {'tol': 0.15},
            # ]
        ),

        # --- 2. Basic Navigator Node (The Brain) ---
        # Decides where to go. Subscribes to /scan, publishes to /cmd_vel_raw
        Node(
            package=pkg_name,
            executable='basic_navigator_node.py',
            name='basic_navigator',
            output='screen',
            parameters=[
                {'forward_speed': 0.15},
                {'turn_speed': 0.4},
                {'obstacle_distance': 0.35},  # Increased slightly for safety margin
                {'scan_fov_deg': 30.0}
            ]
        ),

        # --- 3. Obstacle Detection Node (The Safety) ---
        # Subscribes to /closest_obstacle_distance and /cmd_vel_raw
        # Publishes final, safe commands to /cmd_vel
        Node(
            package=pkg_name,
            executable='obstacle_detection_node.py',
            name='turtlebot3_obstacle_detection',
            output='screen',
            # This node takes the output of the two nodes above as input.
            # No special parameters needed as its logic is self-contained.
        ),
    ])
