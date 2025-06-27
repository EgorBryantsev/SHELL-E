#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32, Header
import numpy as np
from sklearn.cluster import DBSCAN
from sensor_msgs_py import point_cloud2


class LidarListener(Node):
    """
    A ROS2 node that listens to LaserScan data, performs Euclidean clustering to detect
    obstacles, and publishes the distance to the closest obstacle and a point cloud
    of the detected clusters for visualization.
    """

    def __init__(self):
        super().__init__('lidar_listener')

        # Declare parameters for tunability
        self.declare_parameter('min_scan_radius', 0.10)  # Ignore points within 10cm of the sensor
        self.declare_parameter('dbscan_eps', 0.2)  # Clustering distance tolerance (20cm)
        self.declare_parameter('dbscan_min_samples', 5)  # Min points to form a cluster
        self.declare_parameter('cluster_min_size', 5)  # Min points for a valid obstacle cluster
        self.declare_parameter('cluster_max_size', 100)  # Max points to filter out walls

        # Get parameters
        self.min_scan_radius = self.get_parameter('min_scan_radius').get_parameter_value().double_value
        self.eps = self.get_parameter('dbscan_eps').get_parameter_value().double_value
        self.min_samples = self.get_parameter('dbscan_min_samples').get_parameter_value().integer_value
        self.cluster_min_size = self.get_parameter('cluster_min_size').get_parameter_value().integer_value
        self.cluster_max_size = self.get_parameter('cluster_max_size').get_parameter_value().integer_value

        # Subscriber to the raw laser scan data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)

        # Publisher for the distance to the closest detected obstacle cluster
        self.closest_obstacle_distance_pub = self.create_publisher(Float32, 'closest_obstacle_distance', 10)

        # Publisher for visualizing the detected obstacle clusters in RViz
        self.cluster_points_pub = self.create_publisher(PointCloud2, 'cluster_points', 10)

        self.get_logger().info(f"Lidar Listener Node Started with min_scan_radius: {self.min_scan_radius} m")

    def scan_callback(self, msg):
        """
        Callback function for the LaserScan subscriber. Processes scan data to find obstacle clusters.
        """
        # 1. Convert LaserScan data from polar to Cartesian coordinates (x, y)
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filter out invalid range values (inf, nan)
        valid_indices = np.isfinite(ranges)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        points = np.array([
            (r * np.cos(theta), r * np.sin(theta))
            for r, theta in zip(ranges, angles)
        ])

        # Handle case with no valid points at all
        if points.shape[0] == 0:
            return

        # 2. **Self-Filtering**: Remove points within a minimum radius to ignore the robot's own chassis.
        norms = np.linalg.norm(points, axis=1)
        points = points[norms > self.min_scan_radius]

        # Handle case where no points remain after self-filtering
        if points.shape[0] < self.min_samples:
            self.closest_obstacle_distance_pub.publish(Float32(data=float('inf')))
            # Publish an empty point cloud
            self.publish_empty_point_cloud(msg.header)
            return

        # 3. Perform DBSCAN clustering
        db = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points)
        labels = db.labels_

        unique_labels = set(labels)
        all_cluster_points = []

        # 4. Process the clusters
        for label in unique_labels:
            if label == -1:
                # -1 is the label for noise points, ignore them
                continue

            class_member_mask = (labels == label)
            cluster_points = points[class_member_mask]

            # 5. Filter clusters by size to distinguish obstacles from walls or small noise
            if self.cluster_min_size <= len(cluster_points) <= self.cluster_max_size:
                all_cluster_points.extend(cluster_points)

        # 6. Calculate distance to the closest point among all valid clusters
        if all_cluster_points:
            all_cluster_points = np.array(all_cluster_points)
            distances = np.linalg.norm(all_cluster_points, axis=1)
            closest_distance = np.min(distances)

            # Publish the distance
            self.closest_obstacle_distance_pub.publish(Float32(data=float(closest_distance)))
            self.get_logger().info(f'Closest detected obstacle (clustered): {closest_distance:.2f} m',
                                   throttle_duration_sec=1.0)

            # Publish the point cloud for visualization
            self.publish_point_cloud(all_cluster_points, msg.header)
        else:
            # No valid obstacle clusters were found
            self.closest_obstacle_distance_pub.publish(Float32(data=float('inf')))
            self.publish_empty_point_cloud(msg.header)

    def publish_point_cloud(self, points, header):
        """
        Creates and publishes a PointCloud2 message from a numpy array of points.
        """
        points_3d = np.hstack([points, np.zeros((points.shape[0], 1), dtype=np.float32)])

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Update the frame_id in the header
        pc_header = Header(stamp=header.stamp, frame_id=header.frame_id)
        pc2_msg = point_cloud2.create_cloud(pc_header, fields, points_3d)
        self.cluster_points_pub.publish(pc2_msg)

    def publish_empty_point_cloud(self, header):
        """Publishes an empty point cloud, useful for clearing visualizations."""
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        header.frame_id = 'base_scan'
        pc2_msg = point_cloud2.create_cloud(header, fields, [])
        self.cluster_points_pub.publish(pc2_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LidarListener()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()