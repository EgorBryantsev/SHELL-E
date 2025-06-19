from setuptools import setup
import os
from glob import glob

package_name = 'twinning_project'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files from the 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all map files from the 'maps' directory
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        # Include all params files from the 'params' directory
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team41',
    maintainer_email='team41@todo.todo',
    description='Digital Twinning project for Autonomous Systems course.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_listener = twinning_project.lidar_listener:main',
            'obstacle_detection = twinning_project.obstacle_detection:main',
        ],
    },
)