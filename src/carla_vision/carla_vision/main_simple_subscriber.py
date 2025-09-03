import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from argparse import ArgumentParser

arg_parser = ArgumentParser()
arg_parser.add_argument('--topic-name', type=str, default='/my_topic', help='Topic name to subscribe to')
args_script = arg_parser.parse_args()

class SimpleSubscriber(Node):
    def __init__(self, topic_name):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            topic_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info(f'Subscribing to topic: "{topic_name}"')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber(topic_name=args_script.topic_name)
    try:
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        simple_subscriber.get_logger().info('Shutting down simple subscriber.')
    finally:
        simple_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
