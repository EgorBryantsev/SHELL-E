#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class BasicNavigator(Node):
    """
    A simple navigation node that tries to move forward and turns when
    an obstacle is detected in front of it.
    """

    def __init__(self):
        super().__init__('basic_navigator')
        self.get_logger().info("Basic Navigator Node Started!")

        # --- Parameters ---
        self.declare_parameter('forward_speed', 0.2)  # Speed in m/s to move forward
        self.declare_parameter('turn_speed', 0.5)  # Speed in rad/s to turn
        self.declare_parameter('obstacle_distance', 0.35)  # Distance in meters to consider an obstacle
        self.declare_parameter('scan_fov_deg', 40.0)  # Field of view in degrees to check for obstacles

        # --- Publisher for velocity commands ---
        # This node publishes to 'cmd_vel_raw', which is then processed by the
        # safety node (Turtlebot3ObstacleDetection) before being sent to the robot.
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_raw', 10)

        # --- Subscriber to the laser scan ---
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data)

        # --- State Variables ---
        self.is_obstacle_ahead = False

        # --- Timer for publishing commands ---
        # We use a timer to continuously publish commands, ensuring the robot
        # keeps moving or turning.
        self.timer = self.create_timer(0.1, self.timer_callback)

    def scan_callback(self, msg: LaserScan):
        """
        Processes the laser scan data to detect obstacles.
        """
        # Get parameters
        obstacle_distance_threshold = self.get_parameter('obstacle_distance').get_parameter_value().double_value
        fov_deg = self.get_parameter('scan_fov_deg').get_parameter_value().double_value

        # We want to check a field of view in front of the robot.
        # The laser scan is 360 degrees, with 0 degrees being straight ahead.
        # We calculate the range of indices in the `ranges` array that correspond to our FOV.

        # Let's check a cone in front of the robot.
        # The angle for each measurement is: angle_min + index * angle_increment
        # We want to find the indices for -fov/2 to +fov/2 degrees.

        angle_min_rad = msg.angle_min
        angle_max_rad = msg.angle_max
        angle_increment_rad = msg.angle_increment

        # Calculate the start and end index for the front-facing cone
        center_index = int((-angle_min_rad) / angle_increment_rad)
        fov_rad = np.deg2rad(fov_deg)
        half_fov_indices = int((fov_rad / 2.0) / angle_increment_rad)

        start_index = center_index - half_fov_indices
        end_index = center_index + half_fov_indices

        # Ensure indices are within the valid range
        start_index = max(0, start_index)
        end_index = min(len(msg.ranges) - 1, end_index)

        # Extract the front-facing scan ranges
        front_ranges = msg.ranges[start_index:end_index]

        # Filter out 'inf' and 'nan' values which can occur
        valid_ranges = [r for r in front_ranges if np.isfinite(r) and r > 0.0]

        if not valid_ranges:
            # If there are no valid ranges in front (e.g., all inf), assume it's clear
            self.is_obstacle_ahead = False
            return

        # Check if the minimum distance in the front cone is less than our threshold
        min_distance_ahead = min(valid_ranges)

        if min_distance_ahead < obstacle_distance_threshold:
            self.is_obstacle_ahead = True
            self.get_logger().info(f"Obstacle DETECTED at {min_distance_ahead:.2f}m", throttle_duration_sec=1)
        else:
            self.is_obstacle_ahead = False
            self.get_logger().info(f"Path is clear. Min distance ahead: {min_distance_ahead:.2f}m",
                                   throttle_duration_sec=1)

    def timer_callback(self):
        """
        Based on the current state, decides whether to move forward or turn.
        """
        twist_msg = Twist()
        forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value

        if self.is_obstacle_ahead:
            # State: Obstacle detected -> TURN
            # Stop moving forward and turn. We'll turn left by convention.
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = turn_speed
        else:
            # State: Path is clear -> GO STRAIGHT
            twist_msg.linear.x = forward_speed
            twist_msg.angular.z = 0.0

        # Publish the raw command. The safety node will decide if it's safe to execute.
        self.publisher_.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BasicNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
