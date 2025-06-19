from setuptools import setup
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
        # Add these lines to install launch, params, and maps
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
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
            'lidar_listener = turtlebot3_lidar_listener.lidar_listener:main',
            'turtlebot3_obstacle_detection = turtlebot3_lidar_listener.turtlebot3_obstacle_detection:main',
        ],
    },
)