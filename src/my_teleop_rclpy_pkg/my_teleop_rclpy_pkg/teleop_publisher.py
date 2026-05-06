import sys
import tty
import termios

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

MSG = """
Turtlesim Teleop
---------------------------
이동:
        w
   a    s    d

w : 앞으로 이동
s : 뒤로 이동
a : 좌회전
d : 우회전
q : 종료

속도 설정:
  선속도(linear.x) : 0.5
  각속도(angular.z): 1.0
---------------------------
"""

LINEAR_SPEED = 0.5
ANGULAR_SPEED = 1.0


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopPublisher(Node):
    def __init__(self):
        super().__init__('teleop_publisher')
        qos_profile = QoSProfile(depth=10)
        self.twist_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', qos_profile)

    def publish_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.twist_publisher.publish(msg)
        self.get_logger().info(
            'linear.x: {0:.1f}, angular.z: {1:.1f}'.format(linear_x, angular_z))


def main(args=None):
    rclpy.init(args=args)
    node = TeleopPublisher()
    settings = termios.tcgetattr(sys.stdin)
    print(MSG)
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == 'w':
                node.publish_twist(LINEAR_SPEED, 0.0)
            elif key == 's':
                node.publish_twist(-LINEAR_SPEED, 0.0)
            elif key == 'a':
                node.publish_twist(0.0, ANGULAR_SPEED)
            elif key == 'd':
                node.publish_twist(0.0, -ANGULAR_SPEED)
            elif key == 'q':
                node.get_logger().info('종료합니다.')
                break
            else:
                node.publish_twist(0.0, 0.0)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.publish_twist(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
