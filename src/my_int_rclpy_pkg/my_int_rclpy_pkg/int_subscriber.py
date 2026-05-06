import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32


class IntSubscriber(Node):
    def __init__(self):
        super().__init__('int_subscriber')
        qos_profile = QoSProfile(depth=10)
        self.int_subscriber = self.create_subscription(
            Int32,
            'counter',
            self.subscribe_topic_message,
            qos_profile)

    def subscribe_topic_message(self, msg):
        self.get_logger().info('Received message: {0}'.format(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = IntSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
