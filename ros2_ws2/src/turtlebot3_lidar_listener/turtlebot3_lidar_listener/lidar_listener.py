#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from sklearn.neighbors import KDTree
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud, LaserScan
from std_msgs.msg import Header, Float32 #Add float 32 for publishing distance
from rclpy.node import Node
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
        
        # Publisher for detected clusters (for RViz visualization)
        self.cluster_pub = self.create_publisher(PointCloud, 'cluster_points', 10)

        # Publisher for the closest detected obstacle distance
        self.closest_obstacle_distance_pub = self.create_publisher(Float32, 'closest_obstacle_distance', 10)

    def listener_callback(self, msg):
        # print("Callback triggered!")
        # self.get_logger().info(f"Min distance: {min(msg.ranges):.2f} meters")
        # # Or anything else
        
        # 1. LaserScan ➜ Cartesian points
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.asarray(msg.ranges)
        valid = np.isfinite(ranges) & (ranges > 0.05) # ignore NaNs + the robot skin
        xs, ys = ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid])
        points = np.vstack((xs, ys)).T # shape = (N,2)

        # NEW: Initialize all_cluster_points
        all_cluster_points = [] 

        if points.shape[0] == 0: # empty scan – nothing to cluster
            # If no points, assume no obstacle detected (or very far away)
            closest_dist_msg = Float32()
            closest_dist_msg.data = float('inf') # Indicate no close obstacle
            self.closest_obstacle_distance_pub.publish(closest_dist_msg)
            return

        # 2. Fast Euclidean Clustering
        tol = 0.12 # [m] neighbour radius (ros2 param if you like)
        min_size = 3 # ignore tiny blobs
        max_size = 250 # ignore very large blobs (walls, etc.)

        tree = KDTree(points, leaf_size=16)
        processed = np.zeros(points.shape[0], dtype=bool)
        labels = -np.ones(points.shape[0], dtype=int)
        current_lbl = 0

        for idx in range(points.shape[0]):
            if processed[idx]:
                continue
            
            # Breadth-first flood fill over neighbours within `tol`
            queue = [idx]
            cluster = []
            while queue:
                i = queue.pop(0) # FIFO for BFS
                if processed[i]:
                    continue
                processed[i] = True
                cluster.append(i)
                # find all neighbours of point i inside radius = tol
                nbrs = tree.query_radius(points[i:i+1], r=tol)[0]
                queue.extend([n for n in nbrs if not processed[n]])

            if min_size <= len(cluster) <= max_size:
                labels[cluster] = current_lbl
                all_cluster_points.extend(points[cluster]) # Add clustered points
                current_lbl += 1


        # # 3. Report the clusters
        # for lbl in range(current_lbl):
        #     pts = points[labels == lbl]
        #     centroid = pts.mean(axis=0)
        #     self.get_logger().info(
        #         f'Cluster {lbl:02d} -> N={len(pts)}  centroid=({centroid[0]:.2f},{centroid[1]:.2f})')


        # 3. Determine the closest distance from the clustered points (obstacle)
        # initialize at infinity to find the min valid distance
        closest_obstacle_from_clusters = float('inf')
        
        # Measure only the distance to those clusters (populated during the scan) containing > 0 points
        if len(all_cluster_points) > 0:
            # Calculate distances for all clustered points from the origin (robot's center)
            distances = np.linalg.norm(np.array(all_cluster_points), axis=1)
            closest_obstacle_from_clusters = np.min(distances)
            self.get_logger().info(f"Closest detected obstacle (clustered): {closest_obstacle_from_clusters:.2f} m")
        
        # Publish the closest obstacle distance
        closest_dist_msg = Float32()
        closest_dist_msg.data = closest_obstacle_from_clusters
        self.closest_obstacle_distance_pub.publish(closest_dist_msg)

        # 4. Publish as PointCloud for RViz / downstream nodes
        cloud = PointCloud()
        cloud.header = Header(stamp=self.get_clock().now().to_msg(),
                                frame_id=msg.header.frame_id)
        
        # Only publish points that were part of a valid cluster
        for p, lbl in zip(points, labels):
            if lbl == -1: # skip noise
                continue
            pt = Point32()
            pt.x, pt.y, pt.z = float(p[0]), float(p[1]), 0.0
            cloud.points.append(pt)

        if len(cloud.points) > 0: # Only publish if there're clustered points
            self.cluster_pub.publish(cloud)

def main(args=None):
    rclpy.init(args=args)
    node = LidarListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()