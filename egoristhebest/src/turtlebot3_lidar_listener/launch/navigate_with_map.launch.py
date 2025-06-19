# ros2_ws/src/turtlebot3_lidar_listener/launch/navigate_with_map.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import xacro  # <--- IMPORT XACRO

def generate_launch_description():
    # ────────────────────────────── Paths & files ─────────────────────────────
    pkg_share                  = get_package_share_directory('turtlebot3_lidar_listener')
    nav2_bringup_dir           = get_package_share_directory('nav2_bringup')
    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')

    map_yaml_file = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_share, 'maps', 'map.yaml')
    )
    nav2_params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    )
    rviz_config_file = LaunchConfiguration(
        'rviz_config',
        default=os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ───────────────────── Process the XACRO file ──────────────────────
    # CHANGE: Point to the .urdf.xacro file instead of the .urdf file
    xacro_file = os.path.join(
        turtlebot3_description_dir, 'urdf', 'turtlebot3_burger.urdf.xacro'
    )
    # CHANGE: Use xacro to process the file and generate the robot_description
    robot_description_content = Command(['xacro ', xacro_file, ' frame_prefix:=', ''])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Pass the processed xacro content
            'robot_description': robot_description_content,
        }],
    )

    # ─────────────────────────── Custom safety nodes ──────────────────────────
    lidar_listener_node = Node(
        package='turtlebot3_lidar_listener',
        executable='lidar_listener',
        name='lidar_listener',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # FIX: Set a larger radius to avoid detecting the robot's own chassis
            'min_scan_radius': 0.20,
        }]
    )

    obstacle_detection_node = Node(
        package='turtlebot3_lidar_listener',
        executable='turtlebot3_obstacle_detection',
        name='turtlebot3_obstacle_detection',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_raw', '/cmd_vel_nav')],
    )

    # ──────────────────────────── Nav2 bring-up ───────────────────────────────
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
            # This topic name is correct based on your remapping
            'cmd_vel_topic': '/cmd_vel_nav',
        }.items(),
    )

    # ─────────────────────────────── RViz2 ────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # ────────────────────────── Build launch description ──────────────────────
    return LaunchDescription([
        DeclareLaunchArgument('map',          default_value=map_yaml_file),
        DeclareLaunchArgument('params_file',  default_value=nav2_params_file),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz_config',  default_value=rviz_config_file),

        robot_state_publisher,
        lidar_listener_node,
        obstacle_detection_node,
        nav2_bringup_launch,
        rviz_node,
    ])