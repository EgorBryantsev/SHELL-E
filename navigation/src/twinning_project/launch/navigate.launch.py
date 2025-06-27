import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('twinning_project')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # --- Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file_path = LaunchConfiguration('map', default=os.path.join(pkg_share, 'maps', 'map.yaml'))
    params_file_path = LaunchConfiguration('params_file', default=os.path.join(pkg_share, 'params', 'nav2_params.yaml'))

    # --- Your Custom Nodes ---
    lidar_listener_node = Node(
        package='twinning_project',
        executable='lidar_listener',
        name='lidar_listener',
        output='screen'
    )

    obstacle_detection_node = Node(
        package='twinning_project',
        executable='obstacle_detection',
        name='obstacle_detection_node',
        output='screen',
        remappings=[
            ('cmd_vel_raw', '/cmd_vel_nav'),  # Subscribe to Nav2's output
            ('cmd_vel', '/cmd_vel')  # Publish to the robot's input
        ],
        parameters=[{'safety_distance': 0.25}]  # Set your safety distance
    )

    # --- Nav2 Stack ---
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file_path,
            'use_sim_time': use_sim_time,
            'params_file': params_file_path,
            'autostart': 'true',
            'cmd_vel_topic': '/cmd_vel_nav'  # KEY: Remap Nav2's output to our intermediate topic
        }.items(),
    )

    # --- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true'),
        DeclareLaunchArgument('map', default_value=map_file_path, description='Full path to map file'),

        lidar_listener_node,
        obstacle_detection_node,
        nav2_bringup,
        rviz_node,
    ])