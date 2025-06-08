#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.descriptions import ParameterGroup

def generate_launch_description():
    # Ensure the name matches the one in setup.py
    package_name = 'turtlebot3_lidar_listener'

    # Define common parameters for the nodes (e.g., safety distance)
    # You can also set these inside each Node if they differ per robot
    common_params = ParameterGroup([
        {'safety_distance': 0.3}, # The safety distance used in Turtlebot3ObstacleDetection
    ])

    # Initializes the main LaunchDescription object, to which 
    # all actions (like starting nodes) will be added.
    ld = LaunchDescription()

    # Nodes for the REAL Robot
    real_robot_group = GroupAction(
        actions=[
            # LidarListener node for the real robot
            Node(
                package=package_name,
                executable='lidar_listener', # The entry point name from setup.py
                namespace='real_lidar_listener',
                output='screen',
                remappings=[
                    ('/scan', '/real_robot/scan'), # Real robot's LiDAR topic
                    ('/closest_obstacle_distance', '/real_robot/closest_obstacle_distance'),
                    ('/cluster_points', '/real_robot/cluster_points') # For RViz visualization
                ]
            ),
            # Turtlebot3ObstacleDetection node for the real robot
            Node(
                package=package_name,
                executable='turtlebot3_obstacle_detection', # The entry point name from setup.py
                namespace='real_obstacle_avoidance',
                output='screen',
                remappings=[
                    ('/cmd_vel', '/real_robot/cmd_vel'), # Real robot's command velocity topic
                    ('/cmd_vel_raw', '/real_robot/cmd_vel_raw'), # If used for manual control override
                    ('/closest_obstacle_distance', '/real_robot/closest_obstacle_distance') # From real_lidar_listener
                ],
                parameters=[common_params] # Apply common parameters
            ),
        ]
    )

    ld.add_action(real_robot_group)

    # Nodes for the VIRTUAL Robot
    virtual_robot_group = GroupAction(
        actions=[
            # LidarListener node for the virtual robot
            Node(
                package=package_name,
                executable='lidar_listener',
                namespace='virtual_lidar_listener',
                output='screen',
                remappings=[
                    ('/scan', '/virtual_robot/scan'), # Virtual robot's LiDAR topic
                    ('/closest_obstacle_distance', '/virtual_robot/closest_obstacle_distance'),
                    ('/cluster_points', '/virtual_robot/cluster_points')
                ]
            ),
            # Turtlebot3ObstacleDetection node for the virtual robot
            Node(
                package=package_name,
                executable='turtlebot3_obstacle_detection',
                namespace='virtual_obstacle_avoidance',
                output='screen',
                remappings=[
                    ('/cmd_vel', '/virtual_robot/cmd_vel'),
                    ('/cmd_vel_raw', '/virtual_robot/cmd_vel_raw'),
                    ('/closest_obstacle_distance', '/virtual_robot/closest_obstacle_distance')
                ],
                parameters=[common_params]
            ),
        ]
    )

    ld.add_action(virtual_robot_group)

    return ld