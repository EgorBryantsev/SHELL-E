# Examples/Resources

## ROS2
`ROS2/Examples/`
  * `ListenerExample.cs`
  * `TalkerExample.cs`

`ROS2/turtlebot3/Scripts/`
  * `TurtlebotWheelSpinner.cs`
### ROS Messages
`ROS2/turtlebot3/msg/`
  * `SensorState.msg`
  * `Sound.msg`
  * `VersionInfo.msg`

---
## Scripts
`Scripts/Tests/PlayMode/`
  * `WaypointIntegrationTest.cs`

`Scripts/`
  * `AGVController.cs`
  * `Clock.cs`
  * `FreeCam.cs`
  * `LaserScanSensor.cs`
  * `ROSClockPublisher.cs`
  * `ROSTransformTreePublisher.cs`
  * `TimeStamp.cs`
  * `TransformExtensions.cs`
  * `TransformTreeNode.cs`

---
## T3 ROS Packages (foxy-devel)

### `turtlebot3_bringup`
`launch/`
  * `robot_launch.py`
  * `rviz2_launch.py`
  * `turtlebot3_state_publisher_launch.py`

### `turtlebot3_cartographer`
`config/`
  * `turtlebot3_lds_2d.lua`
`launch/`
  * `cartographer_launch.py`
  * `occupancy_grid_launch.py`
`rviz/`
  * `tb3_cartographer.rviz`

### `turtlebot3_example`
  * `setup.py`

`turtlebot3_obstacle_detection/`
* `__init__.py`
* `main.py`
* `turtlebot3_obstacle_detection.py`

`turtlebot3_patrol_client/`
* `__init__.py`
* `main.py`
* `turtlebot3_patrol_client.py`

`turtlebot3_patrol_server/`
* `__init__.py`
* `main.py`
* `turtlebot3_patrol_server.py`

  `turtlebot3_position_control/`
* `__init__.py`

### `turtlebot3_navigation2`
`launch/`
  * `navigation2_launch.py`
`param/`
  * `burger.yaml`

### `turtlebot3_node`
- `include/turtlebot3_node/`
- `odometry.hpp`
- `turtlebot3.hpp`
`devices/`
* `sound.hpp`
* `reset.hpp`
* `devices.hpp`
* `motor_power.hpp`
`sensors/`
* `battery_state.hpp`
* `imu.hpp`
* `joint_state.hpp`
* `sensor_state.hpp`
* `sensors.hpp`


### `turtlebot3_teleop`
  * `__init__.py`
  
`script/`
  * `teleop_keyboard.py`
  
`param/`
  * `burger.yaml`

---
## Unity Robotics Demo

### Messages
`UnityRoboticsDemo/msg/`
  * `PosRotMsg.cs`
  * `UnityColorMsg.cs`

### Services
`UnityRoboticsDemo/srv/`
  * `ObjectPoseServiceRequest.cs`
  * `ObjectPoseServiceResponse.cs`
  * `PositionServiceRequest.cs`
  * `PositionServiceResponse.cs`

---
## Misc
`Misc/`
  * `MazeAreaDensityCalculator.cs`
  * `ObstacleCounter.cs`
  * `UpdateYAML.cs`