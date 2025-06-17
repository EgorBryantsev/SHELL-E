import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool  # To publish obstacle detection status
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraViewerNode(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')

        self.image_topic = '/camera/image_raw'
        self.obstacle_topic = '/camera/obstacle_detected'
        self.min_contour_area = 3000

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)  # QoS profile depth
        self.subscription

        self.obstacle_publisher = self.create_publisher(Bool, self.obstacle_topic, 10)

        self.bridge = CvBridge()
        self.get_logger().info(f'Camera Viewer Node started.')
        self.get_logger().info(f'--> Subscribing to image topic: {self.image_topic}')
        self.get_logger().info(f'--> Publishing obstacle status to: {self.obstacle_topic}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        obstacle_detected = self.detect_red_obstacle(cv_image)

        detection_msg = Bool()
        detection_msg.data = obstacle_detected
        self.obstacle_publisher.publish(detection_msg)

        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)

    def detect_red_obstacle(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red_1 = np.array([0, 120, 70])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 120, 70])
        upper_red_2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)
        red_mask = mask1 + mask2

        cv2.imshow("Red Mask", red_mask)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > self.min_contour_area:
                self.get_logger().info('Red obstacle DETECTED!', throttle_duration_sec=1)

                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                return True

        return False


def main(args=None):
    rclpy.init(args=args)
    camera_viewer_node = CameraViewerNode()
    try:
        rclpy.spin(camera_viewer_node)
    except KeyboardInterrupt:
        camera_viewer_node.get_logger().info('Camera viewer node shutting down...')
    finally:
        # Cleanup
        camera_viewer_node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
