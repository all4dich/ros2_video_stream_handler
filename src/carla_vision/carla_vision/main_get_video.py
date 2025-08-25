import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import qos_profile_sensor_data

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.declare_parameter('image_topic', '/sensing/camera/traffic_light/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile=qos_profile_sensor_data) # Use sensor data QoS profile for camera streams [13]
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.get_logger().info(f"Subscribing to topic: {image_topic}")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            # 'bgr8' is a common encoding for color images. Use 'passthrough' if uncertain or 'mono8' for grayscale. [1, 5]
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Display the image
        cv2.imshow("Carla Traffic Light Camera", cv_image)
        cv2.waitKey(1) # Refresh rate of 1ms [1, 2]

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        image_subscriber.get_logger().info('Shutting down image viewer.')
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows() # Close all OpenCV windows

if __name__ == '__main__':
    main()
