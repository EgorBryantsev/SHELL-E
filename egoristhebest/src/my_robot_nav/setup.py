import os
from glob import glob
from setuptools import setup

package_name = 'my_robot_nav'

setup(
    name=package_name,
    version='0.0.2', # Incremented version
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- IMPORTANT: Install directories ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='An autonomous navigation package for TurtleBot using Nav2.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_listener_node.py = my_robot_nav.lidar_listener_node:main',
            'obstacle_detection_node.py = my_robot_nav.obstacle_detection_node:main',
            'basic_navigator_node.py = my_robot_nav.basic_navigator_node:main',
        ],
    },
)