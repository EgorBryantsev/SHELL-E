# Camera Setup (foxy)
Steps to stream video from a usb camera into ros2, making it available via a topic. (in progress)
### Prerequisites
Install needed packages:
```bash
sudo apt update
sudo apt install v4l-utils ros-foxy-v4l2-camera ros-foxy-rqt-image-view
# optionally sudo apt install qv4l2
```
### camera test
After plugging in the camera, check detection with:
```bash
ls /dev/video*
```
You can also test video output with (using qv4l2):
```bash
qv4l2
```
### ROS 2 setup
Sources:
```bash
source /opt/ros/foxy/setup.bash
export TURTLEBOT3_MODEL=burger
```
And run the camera node:
```bash
ros2 run v4l2_camera v4l2_camera_node # --ros-args -p video_device:="/dev/videoX"
```
There should be a camera topic inn `ros2 list topics`. The camera output through ros can be viewed with `rqt`.
### python subscriber
In `/etc/apt/sources.list.d/ros2.list` add:
```
deb-src [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main
```
Then update repos:
```bash
sudo apt update
```
Set up the ros2 folder:
```
mkdir ~/ros2_ws && cd ros2_ws 
apt source ros-foxy-turtlebot3-description 
cd src
ros2 pkg create --build-type ament_python camera_subscriber --dependencies rclpy sensor_msgs cv_bridge
```
Add `~/ros2_ws/src/camera_subscriber/setup.py` and `~/ros2_ws/src/camera_subscriber/camera_subscriber/camera_viewer_node.py` (from repo), then:
```bash
colcon build --packages-select turtlebot3_description camera_subscriber 
```
Then source or add to bashrc:
```bash
source ~/ros2_ws/install/setup.bash
```

---
not finished, urdf still needed
---