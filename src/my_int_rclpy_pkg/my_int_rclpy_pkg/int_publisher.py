import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32


class IntPublisher(Node):
    def __init__(self):
        super().__init__('int_publisher')
        qos_profile = QoSProfile(depth=10)
        self.int_publisher = self.create_publisher(Int32, 'counter', qos_profile)
        self.timer = self.create_timer(1, self.publish_int_msg)
        self.count = 0

    def publish_int_msg(self):
        msg = Int32()
        msg.data = self.count
        self.int_publisher.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.data))
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = IntPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
