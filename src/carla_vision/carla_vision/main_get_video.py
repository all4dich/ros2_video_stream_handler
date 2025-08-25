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

        # Parameters for video saving
        self.declare_parameter('output_video_filename', 'output_video.avi')
        self.video_filename = self.get_parameter('output_video_filename').get_parameter_value().string_value

        self.declare_parameter('output_video_fps', 30.0) # Default FPS for the output video
        self.output_fps = self.get_parameter('output_video_fps').get_parameter_value().double_value

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile=qos_profile_sensor_data) # Use sensor data QoS profile for camera streams [13]
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.get_logger().info(f"Subscribing to topic: {image_topic}")

        self.video_writer = None
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID') # Codec for AVI file [6, 11]

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Initialize VideoWriter on the first frame
        if self.video_writer is None:
            # Get frame dimensions from the first image [3, 4]
            height, width = cv_image.shape[:2]
            try:
                self.video_writer = cv2.VideoWriter(self.video_filename, self.fourcc, self.output_fps, (width, height)) # [6, 11]
                if not self.video_writer.isOpened():
                    self.get_logger().error(f"Error: Could not open video writer for file {self.video_filename}.")
                    self.video_writer = None # Mark as not opened
                else:
                    self.get_logger().info(f"Started recording video to {self.video_filename} at {self.output_fps} FPS with dimensions {width}x{height}.")
            except Exception as e:
                self.get_logger().error(f"Error initializing video writer: {e}")
                self.video_writer = None # Ensure it's None if initialization fails

        # Write the frame to the video file
        if self.video_writer is not None and self.video_writer.isOpened():
            try:
                self.video_writer.write(cv_image) # [12]
            except Exception as e:
                self.get_logger().error(f"Error writing frame to video file: {e}")

        # Display the image
        cv2.imshow("Carla Traffic Light Camera", cv_image)
        cv2.waitKey(1) # Refresh rate of 1ms [1, 2]

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        image_subscriber.get_logger().info('Shutting down image viewer and video recorder.')
    finally:
        # Release the video writer resources
        if image_subscriber.video_writer is not None and image_subscriber.video_writer.isOpened():
            image_subscriber.video_writer.release() # [9]
            image_subscriber.get_logger().info(f"Video recording finished. File saved to {image_subscriber.video_filename}.")
        image_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows() # Close all OpenCV windows

if __name__ == '__main__':
    main()
