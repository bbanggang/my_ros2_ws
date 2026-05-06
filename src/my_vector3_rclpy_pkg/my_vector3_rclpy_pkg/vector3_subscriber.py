import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Vector3


class Vector3Subscriber(Node):
    def __init__(self):
        super().__init__('vector3_subscriber')
        qos_profile = QoSProfile(depth=10)
        self.vector3_subscriber = self.create_subscription(
            Vector3,
            'vector3',
            self.subscribe_topic_message,
            qos_profile)

    def subscribe_topic_message(self, msg):
        self.get_logger().info(
            'Received message: x={0}, y={1}, z={2}'.format(msg.x, msg.y, msg.z))


def main(args=None):
    rclpy.init(args=args)
    node = Vector3Subscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
