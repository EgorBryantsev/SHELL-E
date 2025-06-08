#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

     # NEW: Declare a launch argument for the robot namespace
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value=TextSubstitution(text=''), # Default to empty for no namespace
        description='Namespace for the robot topics'
    )
    robot_namespace = LaunchConfiguration('robot_namespace')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)

    # Major refactor of the robot_state_publisher
    # Reference page: https://github.com/ros2/demos/pull/426
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    # print (robot_desc) # Printing urdf information.

    return LaunchDescription([
        # NEW: Add the robot_namespace argument declaration
        robot_namespace_arg,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}],
            namespace=robot_namespace # NEW: Apply namespace to robot_state_publisher
        )
    ])