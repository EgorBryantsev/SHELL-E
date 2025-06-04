#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String
import numpy as np


class LidarListener(Node):
    def __init__(self):
        super().__init__('lidar_listener')
        self.get_logger().info("Lidar Listener Node Started!")

        # Subscribe to the /scan topic from the LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile_sensor_data)

    def listener_callback(self, msg):
        # Convert scan ranges to a NumPy array and filter invalid values
        ranges = np.array(msg.ranges)
        valid = ranges[(ranges > 0.05) & (ranges < 1.0)]  # remove NaNs, noise, etc.

        if len(valid) < 5:
            self.get_logger().warn("Too few valid scan points.")
            return

        # Compute statistical properties
        mean_dist = np.mean(valid)
        variance = np.var(valid)

        # Shape classification based on radial variance
        if variance < 0.0005:
            shape = 'circle'
        else:
            shape = 'square'

        # Log the result
        self.get_logger().info(f"Shape: {shape} | Var: {variance:.5f} | Mean: {mean_dist:.2f}m")

        # Publish shape result as a string message
        msg = String()
        msg.data = shape
        self.shape_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()