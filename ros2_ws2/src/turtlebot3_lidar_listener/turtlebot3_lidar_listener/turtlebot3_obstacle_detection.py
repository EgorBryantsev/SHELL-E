#!/usr/bin/env python3

import rclpy

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.linear_velocity = 0.0  # unit: m/s
        self.angular_velocity = 0.0  # unit: m/s
         # self.scan_ranges = [] # No longer directly using raw scan ranges for avoidance
        self.detected_obstacle_distance = float('inf') # NEW: Store the distance from the new topic
        self.init_scan_state = False  # To get the initial scan data at the beginning

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10) # specifies how the subscription should behave (e.g., reliability, history depth)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # # Initialise subscribers
        # self.scan_sub = self.create_subscription(
        #     LaserScan,
        #     'scan',
        #     self.scan_callback,
        #     qos_profile=qos_profile_sensor_data)

        # NEW: Subscribe to the closest_obstacle_distance topic from LidarListener
        self.obstacle_distance_sub = self.create_subscription(
                    Float32, # The message type is Float32
                    'closest_obstacle_distance', # The topic name is 'closest_obstacle_distance'
                    self.closest_obstacle_distance_callback, # The callback function to handle incoming messages
                    qos) # Use a reliable Quality of Service for this critical info

        self.cmd_vel_raw_sub = self.create_subscription(
                    Twist,
                    'cmd_vel_raw',
                    self.cmd_vel_raw_callback,
                    qos)

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.update_timer = self.create_timer(
            0.010,  # unit: s
            self.update_callback)

        self.get_logger().info("Turtlebot3 obstacle detection node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    # def scan_callback(self, msg): # REMOVED: This callback is no longer needed
    #     self.scan_ranges = msg.ranges
    #     self.init_scan_state = True # This state will now be set by the new callback

    # NEW: Callback for the closest_obstacle_distance topic
    def closest_obstacle_distance_callback(self, msg):
        self.detected_obstacle_distance = msg.data
        self.init_scan_state = True # Indicate that we have received initial obstacle data

    def cmd_vel_raw_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_callback(self):
        if self.init_scan_state is True:
            self.detect_obstacle()

    def detect_obstacle(self):
        twist = Twist()
        # obstacle_distance = min(self.scan_ranges) # OLD: Replaced by new variable
        safety_distance = 0.3  # unit: m

        # CHANGE: Use the distance from our sophisticated obstacle detector
        if self.detected_obstacle_distance > safety_distance:
            twist.linear.x = self.linear_velocity
            twist.angular.z = self.angular_velocity
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info(f"Clustered obstacles detected at {self.detected_obstacle_distance:.2f} m. Robot stopped.")

        # Implement Twist publishing to control robot's movement 
        self.cmd_vel_pub.publish(twist)

# NOTE: The main function for Turtlebot3ObstacleDetection remains the same
# The main fucntion moved from the separate main.py file
# The main function should be separate from the class
def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)
    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
