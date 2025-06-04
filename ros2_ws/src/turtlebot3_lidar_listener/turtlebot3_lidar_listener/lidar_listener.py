#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

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
        print("Callback triggered!")
        self.get_logger().info(f"Min distance: {min(msg.ranges):.2f} meters")
        # Or anything else

def main(args=None):
    rclpy.init(args=args)
    node = LidarListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()