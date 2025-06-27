import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_nav')

    # --- Paths ---
    map_file_path = os.path.join(pkg_share, 'maps', 'map.yaml')
    params_file_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # --- Argument Declarations ---
    # FIX: Default to 'true' since we are using a simulation (Unity)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Unity) clock if true'
    )

    # --- Nav2 Bringup ---
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': params_file_path
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        nav2_bringup_launch
    ])
