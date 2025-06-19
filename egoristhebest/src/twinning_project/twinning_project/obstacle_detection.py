# File: ~/ros2_ws/src/twinning_project/twinning_project/obstacle_detection.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class ObstacleDetection(Node):
    def __init__(self):
        super().__init__('obstacle_detection')

        # Parameter for safety distance
        self.declare_parameter('safety_distance', 0.25)  # meters
        self.safety_distance = self.get_parameter('safety_distance').get_parameter_value().double_value

        # Subscription to the closest obstacle distance calculated by lidar_listener
        self.distance_subscription = self.create_subscription(
            Float32,
            'closest_obstacle_distance',
            self.distance_callback,
            10)

        # Subscription to the raw velocity commands (from Nav2 or teleop)
        self.velocity_subscription = self.create_subscription(
            Twist,
            'cmd_vel_raw',  # This will be remapped in the launch file
            self.velocity_callback,
            10)

        # Publisher for the final, safety-checked velocity commands to the robot
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.last_received_velocity = Twist()
        self.obstacle_too_close = False
        self.get_logger().info(f"Obstacle Detection node started with safety distance: {self.safety_distance}m")

    def distance_callback(self, msg):
        # Check if the detected obstacle is within the safety distance
        if msg.data < self.safety_distance:
            self.obstacle_too_close = True
        else:
            self.obstacle_too_close = False

    def velocity_callback(self, msg):
        self.last_received_velocity = msg

        final_velocity = Twist()

        # If an obstacle is too close, command a stop.
        # We only stop if the robot is trying to move forward.
        if self.obstacle_too_close and self.last_received_velocity.linear.x > 0:
            final_velocity.linear.x = 0.0
            final_velocity.angular.z = 0.0  # Also stop turning
            self.get_logger().warn('Obstacle too close! Stopping robot.')
        else:
            # Otherwise, pass the received velocity command through.
            final_velocity = self.last_received_velocity

        self.velocity_publisher.publish(final_velocity)


def main(args=None):
    rclpy.init(args=args)
    obstacle_detection_node = ObstacleDetection()
    rclpy.spin(obstacle_detection_node)
    obstacle_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()