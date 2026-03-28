import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.subscription = self.create_subscription(
            String, 'test_topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = TestSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
