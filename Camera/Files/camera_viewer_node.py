import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewerNode(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')
        self.topic_name = '/image_raw'
        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            10) # qos profile depth
        self.subscription

        self.bridge = CvBridge()
        self.get_logger().info(f'Camera Viewer Node started. Subscribing to {self.topic_name}...')

    def image_callback(self, msg):
        # called every time image received on subscribed topic
        self.get_logger().info('received image')

        try:
            # convert to opencv image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # display
        cv2.imshow("Camera Feed from Subscriber", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    camera_viewer_node = CameraViewerNode() # create instance of node

    try:
        rclpy.spin(camera_viewer_node)
                                      
    except KeyboardInterrupt:
        camera_viewer_node.get_logger().info('Camera viewer node shutting down due to KeyboardInterrupt (Ctrl+C)...')
    finally:
        camera_viewer_node.destroy_node() 
        cv2.destroyAllWindows()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()