import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Vector3


class Vector3Publisher(Node):
    def __init__(self):
        super().__init__('vector3_publisher')
        qos_profile = QoSProfile(depth=10)
        self.vector3_publisher = self.create_publisher(Vector3, 'vector3', qos_profile)

    def publish_vector3_msg(self, x, y, z):
        msg = Vector3()
        msg.x = x
        msg.y = y
        msg.z = z
        self.vector3_publisher.publish(msg)
        self.get_logger().info(
            'Published message: x={0}, y={1}, z={2}'.format(msg.x, msg.y, msg.z))


def main(args=None):
    rclpy.init(args=args)
    node = Vector3Publisher()
    try:
        while rclpy.ok():
            try:
                raw = input('실수값 3개를 입력하세요 (x y z): ')
                x, y, z = map(float, raw.split())
                node.publish_vector3_msg(x, y, z)
            except ValueError:
                node.get_logger().warn('올바른 형식으로 입력하세요. 예) 1.0 2.5 -3.0')
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
