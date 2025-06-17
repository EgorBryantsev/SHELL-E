# TurtleBot3 LiDAR Shape Detection Node

The lidar_listener ROS2 node subscribes to the 'scan' topic and analyzes the scan data published by the TB3 LiDAR. It aims to compute the distance to teh cluster of points - detected objects. The turtlebot3_obstacle_detection node aims to inform the TB3 about the safety distance, sending Twist commands to the robot together with a "stop" message in terminal.

Before you strat with the file setup, you'll need to install pip
1) Download the get-pip.py script:
https://bootstrap.pypa.io/pip/3.8/get-pip.py
2) python3 get-pip.py
3) Verify: pip --version

-------------------------------

Next install scikit-learn:
1) pip install --user --upgrade scikit-learn

This single command will correctly install scikit-learn and upgrade numpy and scipy to compatible versions only for your user account.

Note the "--user" part - it tells pip to install the packages into your personal user directory (~/.local/lib/python3.8/site-packages) instead of trying to write to the global system directory. This avoids permission errors and doesn't interfere with the VM's base setup.  

2) Verify: python3 -c "import numpy; import sklearn; print('NumPy version:', numpy.__version__); print('Scikit-learn version:', sklearn.__version__)"

This should return the numpy version and the scikit-learn version.  This confirms that Python is finding the correct, newly-installed packages.

--------------------------------

Begin the file setup:

Note - you should end-up with something like this:
ros2_ws\src\turtlebot3_lidar_listener
├── package.xml
├── resource
│   └── turtlebot3_lidar_listener
├── setup.cfg
├── setup.py
├── test
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── turtlebot3_lidar_listener
    ├── __init__.py
    ├── lidar_listener.py
    └── turtlebot3_obstacle_detection.py

After you finish with the setup you can check your structure by running:


1) If you already followed the previous setup steps, delete the entire turtlebot3_lidar_listener folder and start all over agian.

2) Create the turtlebot3_lidar_listener ROS2 Python Package:
- source /opt/ros/foxy/setup.bash
- cd ~/ros2_ws/src
- ros2 pkg create --build-type ament_python turtlebot3_lidar_listener

3) Create a new file:
- cd ~/turtlebot3_ws/src/turtlebot3_lidar_listener/turtlebot3_lidar_listener
- Create an empty lidar_listener.py file and an empty turtlebot3_obstacle_detection.py file:
touch lidar_listener.py
touch turtlebot3_obstacle_detection.py
- Make sure the files are executable:
chmod +x lidar_listener.py
chmod +x turtlebot3_obstacle_detection.py
- Verify:
ls -l

If the files are colored in green, it means that they're executable.

4) From the github repo SHELL-E go into the ros2_ws2\src\turtlebot3_lidar_listener and copy-paste the contents of the following files into the empty files in your ros2_ws\src\turtlebot3_lidar_listener:
- setup.py
- package.xml
From the ros2_ws2\src\turtlebot3_lidar_listener\turteblt3_lidar_listener you should copy-paste the code into the corresponding files inside the vm's folder ros2_ws\src\turtlebot3_lidar_listener\turtlebot3_lidar_listener:
- lidar_listener.py
- turtlebot3_obstacle_detection.py

5) Build the package in a new terminal window:
- cd ~/ros2_ws2
- rm -rf build/ install/ log/
- colcon build
- source install/setup.bash

6) Check if turtlebot3_lidar_listener is in the ament prefix path (otherwise later you'll get "Package 'turtlebot3_lidar_listener' not found"):
- echo $AMENT_PREFIX_PATH

You should see the turtlebot3_lidar_listener listed in the path: "/home/ubuntuhost/ros2_ws/install/unity_slam_example:/home/ubuntuhost/ros2_ws/install/turtlebot3_lidar_listener:/home/ubuntuhost/ros2_ws/install/ros_tcp_endpoint:/opt/ros/foxy"

------------------------------
Time to run the nodes!

7) Optional: Connect to the real TB3 (in a new terminal):
- ssh ubuntu@<the IP on the robot>
- export TURTLEBOT3_MODEL=burger
- ros2 launch turtlebot3_bringup robot.launch.py

8) In a new terminal:
- cd ~/ros2_ws2
- source /opt/ros/foxy/setup.bash
- source install/setup.bash
- export TURTLEBOT3_MODEL=burger
- ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1

Now hit the play button in Unity and the arrows in the Game field should turn blue - connection is now active!

9) In a new terminal activate the teleop keyboard - make sure the AGV controller in Unity for the TB3 is set to "ROS" and not "Keyboard" (click on turtlebot3_burger in the Hierarchy window):
- cd ~/ros2_ws2
- source /opt/ros/foxy/setup.bash
- source install/setup.bash
- export TURTLEBOT3_MODEL=burger
- ros2 run turtlebot3_teleop teleop_keyboard

10) In a new terminal:
- cd ~/ros2_ws2
- source /opt/ros/foxy/setup.bash
- source install/setup.bash
- export TURTLEBOT3_MODEL=burger
- ros2 run turtlebot3_lidar_listener lidar_listener

Note: Make sure your TurtleBot3 is turned on and publishing /scan topic over the ROS 2 network. (Just run "ros2 topic list")

You should see an output like:
[INFO] [1747646910.205770301] [lidar_listener]: Min distance: 0.27 meters

11) In a new terminal:
- cd ~/ros2_ws2
- source /opt/ros/foxy/setup.bash
- source install/setup.bash
- export TURTLEBOT3_MODEL=burger
- ros2 run turtlebot3_lidar_listener turtlebot3_obstacle_detection

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