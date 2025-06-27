# File: ~/ros2_ws/src/twinning_project/twinning_project/lidar_listener.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
from sklearn.cluster import DBSCAN


class LidarListener(Node):
    def __init__(self):
        super().__init__('lidar_listener')

        # Parameters for clustering
        self.declare_parameter('dbscan_eps', 0.15)  # Distance between points to be considered a cluster
        self.declare_parameter('dbscan_min_samples', 5)  # Min points to form a cluster
        self.eps = self.get_parameter('dbscan_eps').get_parameter_value().double_value
        self.min_samples = self.get_parameter('dbscan_min_samples').get_parameter_value().integer_value

        # Create subscription to the raw LiDAR scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)

        # Create publisher for the closest obstacle distance
        self.distance_publisher = self.create_publisher(Float32, 'closest_obstacle_distance', 10)

        self.get_logger().info('Lidar Listener node has been started.')

    def scan_callback(self, msg):
        # Convert polar coordinates (ranges and angles) to Cartesian coordinates (x, y)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)

        # Filter out invalid range readings (inf, nan)
        valid_indices = np.isfinite(ranges)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]

        if len(valid_ranges) == 0:
            return

        # Convert to x, y points
        points = np.array([
            valid_ranges * np.cos(valid_angles),
            valid_ranges * np.sin(valid_angles)
        ]).T

        # Perform DBSCAN clustering to find groups of points (obstacles)
        db = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points)
        labels = db.labels_

        # -1 label is noise, 0 and above are valid clusters
        unique_labels = set(labels)
        closest_distance = float('inf')

        if len(unique_labels) > 1:  # More than just noise found
            # Calculate distance to each point in valid clusters
            for label in unique_labels:
                if label != -1:  # Ignore noise points
                    cluster_points = points[labels == label]
                    distances = np.linalg.norm(cluster_points, axis=1)
                    min_dist_in_cluster = np.min(distances)
                    if min_dist_in_cluster < closest_distance:
                        closest_distance = min_dist_in_cluster

        # Publish the calculated closest distance
        distance_msg = Float32()
        distance_msg.data = closest_distance
        self.distance_publisher.publish(distance_msg)


def main(args=None):
    rclpy.init(args=args)
    lidar_listener_node = LidarListener()
    rclpy.spin(lidar_listener_node)
    lidar_listener_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()