import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import cv2
from rclpy.qos import qos_profile_sensor_data
from argparse import ArgumentParser
import os
from sensor_msgs_py import point_cloud2

# Constants for BEV image generation
BEV_WIDTH = 512  # pixels
BEV_HEIGHT = 512  # pixels
METERS_PER_PIXEL = 0.1  # Determines the scale of the BEV image
WORLD_WIDTH = BEV_WIDTH * METERS_PER_PIXEL  # meters
WORLD_HEIGHT = BEV_HEIGHT * METERS_PER_PIXEL  # meters
Z_RANGE = (-2.0, 2.0)  # Min and max Z values to consider for coloring

arg_parser = ArgumentParser()
arg_parser.add_argument('--output-filename', type=str, default='/tmp/lidar_bev.avi', help='Output video filename')
arg_parser.add_argument('--output-fps', type=float, default=15.0, help='Output video frames per second (FPS)')
args_script = arg_parser.parse_args()

class LidarSubscriber(Node):
    def __init__(self, output_filename, output_fps):
        super().__init__('lidar_viewer')
        self.declare_parameter('lidar_topic', '/sensor/lidar/front')
        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value

        self.declare_parameter('output_video_filename', output_filename)
        self.video_filename = self.get_parameter('output_video_filename').get_parameter_value().string_value

        self.declare_parameter('output_video_fps', output_fps)
        self.output_fps = self.get_parameter('output_video_fps').get_parameter_value().double_value

        self.subscription = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            qos_profile=qos_profile_sensor_data)
        self.get_logger().info(f"Subscribing to topic: {self.lidar_topic}")

        self.video_writer = None
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')

    def lidar_callback(self, msg):
        # Read point cloud data
        points = point_cloud2.read_points(msg, field_names=("x", "y", "z"))

        # Create a blank BEV image
        bev_image = np.zeros((BEV_HEIGHT, BEV_WIDTH, 3), dtype=np.uint8)

        # Project points to BEV
        for x, y, z in points:
            # Filter points outside the Z range
            if Z_RANGE[0] <= z <= Z_RANGE[1]:
                # Scale world coordinates to pixel coordinates
                pixel_x = int(-y / METERS_PER_PIXEL + BEV_WIDTH / 2)
                pixel_y = int(-x / METERS_PER_PIXEL + BEV_HEIGHT / 2)

                # Check if the pixel is within the image bounds
                if 0 <= pixel_x < BEV_WIDTH and 0 <= pixel_y < BEV_HEIGHT:
                    # Normalize z value to a 0-255 grayscale color
                    color_val = int(((z - Z_RANGE[0]) / (Z_RANGE[1] - Z_RANGE[0])) * 255)
                    color = (color_val, color_val, color_val)
                    bev_image[pixel_y, pixel_x] = color

        # Initialize VideoWriter on the first frame
        if self.video_writer is None:
            try:
                self.video_writer = cv2.VideoWriter(self.video_filename, self.fourcc, self.output_fps, (BEV_WIDTH, BEV_HEIGHT))
                if not self.video_writer.isOpened():
                    self.get_logger().error(f"Error: Could not open video writer for file {self.video_filename}.")
                    self.video_writer = None
                else:
                    self.get_logger().info(f"Started recording video to {self.video_filename} at {self.output_fps} FPS.")
            except Exception as e:
                self.get_logger().error(f"Error initializing video writer: {e}")
                self.video_writer = None

        # Write the frame to the video file
        if self.video_writer and self.video_writer.isOpened():
            self.video_writer.write(bev_image)

        # Display the image
        cv2.imshow("Carla LiDAR BEV" + f": {self.lidar_topic}", bev_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber(output_filename=args_script.output_filename, output_fps=args_script.output_fps)
    try:
        rclpy.spin(lidar_subscriber)
    except KeyboardInterrupt:
        lidar_subscriber.get_logger().info('Shutting down lidar viewer and video recorder.')
    finally:
        if lidar_subscriber.video_writer and lidar_subscriber.video_writer.isOpened():
            lidar_subscriber.video_writer.release()
            lidar_subscriber.get_logger().info(f"Video recording finished. File saved to {lidar_subscriber.video_filename}.")
        lidar_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
