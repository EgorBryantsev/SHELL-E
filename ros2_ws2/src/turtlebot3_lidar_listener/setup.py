from setuptools import setup
# New:
import os
from glob import glob

package_name = 'turtlebot3_lidar_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # NEW: This line installs ALL .py files from our 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntuhost',
    maintainer_email='ubuntuhost@todo.todo',
    description='Custom LiDAR listener and obstacle avoidance for TurtleBot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'lidar_listener = turtlebot3_lidar_listener.lidar_listener:main', # Entry point for LidarListener node
            'turtlebot3_obstacle_detection = turtlebot3_lidar_listener.turtlebot3_obstacle_detection:main', # Entry point for Turtlebot3ObstacleDetection node
        ],

        # NEW: Tell colcon/ament_package to generate environment hooks (e.g. ROS_PACKAGE_PATH)
        'colcon_ament_package_hooks': [
            'ament = ament_package.resources:generate_environment_hooks', # standard entry point
        ]

    },
)