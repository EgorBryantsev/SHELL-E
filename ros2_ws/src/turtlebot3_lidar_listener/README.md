--> Build the package:
cd ~/ros2_ws
colcon build
source install/setup.bash

--> Run the node:
ros2 run turtlebot3_lidar_listener lidar_listener

Note: Make sure your TurtleBot3 is turned on and publishing /scan topic over the ROS 2 network.

You should see an output like:
[INFO] [1747646910.205770301] [lidar_listener]: Min distance: 0.27 meters

/*************************************/

Troubleshooting:
chmod +x lidar_listener.py --> make sure that the node you created in executable
Rubuild and source:
colcon build --package-select turtlebot3_lidar_listener
source install/setup.bash

ros2 topic list --> you should see /scan topic
ros2 topic echo /scan --> you should see the laser scanner data
ros2 node list --> you should see the node you created: /lidar_listener
ros2 topic info /scan --> you should see that the type is a "sensor_msgs/msg/LaserScan" and there should be 1 publisher and one subscriber