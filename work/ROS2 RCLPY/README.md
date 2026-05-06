# ROS2 RCLPY

## 과제 1

`colcon build --symlink-install` 옵션 유무에 따른 소스코드 변경 시 재빌드 필요 여부를 설명한다.

---

### 1. `--symlink-install` 옵션으로 빌드한 경우

`--symlink-install` 옵션을 사용하면 install 디렉터리에 실제 파일이 복사되지 않고 **소스 파일을 가리키는 심볼릭 링크**가 생성된다.  
따라서 Python 소스코드(`.py`)를 수정하면 **재빌드 없이 즉시 반영**된다.

> **이유**: ROS2가 실행 시점에 심볼릭 링크를 통해 원본 소스 파일을 직접 참조하기 때문에, 소스를 수정하면 다음 실행 때 변경 사항이 자동으로 적용된다.

---

### 2. `--symlink-install` 옵션 없이 빌드한 경우

`--symlink-install` 없이 빌드하면 소스 파일이 install 디렉터리로 **직접 복사**된다.  
따라서 소스코드를 수정해도 install 디렉터리의 복사본은 변경되지 않으므로 **반드시 재빌드가 필요**하다.

> **이유**: ROS2는 install 디렉터리의 복사된 파일을 실행하기 때문에, 원본 소스를 수정해도 재빌드하여 install에 다시 복사하지 않으면 변경 사항이 반영되지 않는다.

---

## 과제 2

### 패키지: `my_int_rclpy_pkg`

`int_publisher` 노드가 `counter` 토픽에 `Int32` 메시지를 1초 주기로 퍼블리시한다.  
카운트 값은 0으로 시작하여 퍼블리시할 때마다 1씩 증가한다.  
`int_subscriber` 노드는 `counter` 토픽을 구독하여 수신한 정수값을 터미널 로그로 출력한다.

---

### Publisher 코드 (`int_publisher.py`)

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32  # 정수형 메시지 타입 임포트


class IntPublisher(Node):
    def __init__(self):
        super().__init__('int_publisher')           # 노드 이름 'int_publisher'로 초기화
        qos_profile = QoSProfile(depth=10)          # 큐 크기 10인 QoS 프로파일 생성
        self.int_publisher = self.create_publisher(Int32, 'counter', qos_profile)
        # Int32 타입 메시지를 'counter' 토픽으로 발행하는 퍼블리셔 생성
        self.timer = self.create_timer(1, self.publish_int_msg)
        # 1초마다 publish_int_msg 콜백을 호출하는 타이머 생성
        self.count = 0                              # 퍼블리시할 정수값을 0으로 초기화

    def publish_int_msg(self):
        msg = Int32()                               # Int32 메시지 객체 생성
        msg.data = self.count                       # 현재 카운트 값을 메시지에 저장
        self.int_publisher.publish(msg)             # 'counter' 토픽으로 메시지 발행
        self.get_logger().info('Published message: {0}'.format(msg.data))
        # 발행한 정수값을 터미널에 로그 출력
        self.count += 1                             # 다음 발행을 위해 카운트 1 증가


def main(args=None):
    rclpy.init(args=args)                           # ROS2 통신 초기화
    node = IntPublisher()                           # IntPublisher 노드 생성
    try:
        rclpy.spin(node)                            # 노드가 종료될 때까지 이벤트 루프 실행
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()                         # 노드 종료
        rclpy.shutdown()                            # ROS2 종료


if __name__ == '__main__':
    main()
```

---

### Subscriber 코드 (`int_subscriber.py`)

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32  # 정수형 메시지 타입 임포트


class IntSubscriber(Node):
    def __init__(self):
        super().__init__('int_subscriber')          # 노드 이름 'int_subscriber'로 초기화
        qos_profile = QoSProfile(depth=10)          # 큐 크기 10인 QoS 프로파일 생성
        self.int_subscriber = self.create_subscription(
            Int32,                                  # 구독할 메시지 타입
            'counter',                              # 구독할 토픽 이름
            self.subscribe_topic_message,           # 메시지 수신 시 호출할 콜백 함수
            qos_profile)

    def subscribe_topic_message(self, msg):
        self.get_logger().info('Received message: {0}'.format(msg.data))
        # 수신한 정수값을 터미널에 로그 출력


def main(args=None):
    rclpy.init(args=args)                           # ROS2 통신 초기화
    node = IntSubscriber()                          # IntSubscriber 노드 생성
    try:
        rclpy.spin(node)                            # 노드가 종료될 때까지 이벤트 루프 실행
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()                         # 노드 종료
        rclpy.shutdown()                            # ROS2 종료


if __name__ == '__main__':
    main()
```

---

### 실행 결과

[video1](https://github.com/user-attachments/assets/3ccc3456-2ab9-4a2a-9d3e-aa5e5939aa1f)

---

## 과제 3

### 패키지: `my_vector3_rclpy_pkg`

사용자가 터미널에 공백으로 구분된 실수값 3개(x, y, z)를 입력하면  
`vector3_publisher` 노드가 `vector3` 토픽에 `Vector3` 메시지로 퍼블리시한다.  
`vector3_subscriber` 노드는 `vector3` 토픽을 구독하여 수신한 x, y, z 값을 터미널 로그로 출력한다.  
잘못된 형식을 입력하면 경고 메시지를 출력하고 재입력을 기다린다.

---

### Publisher 코드 (`vector3_publisher.py`)

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Vector3  # 3차원 벡터 메시지 타입 임포트


class Vector3Publisher(Node):
    def __init__(self):
        super().__init__('vector3_publisher')       # 노드 이름 'vector3_publisher'로 초기화
        qos_profile = QoSProfile(depth=10)          # 큐 크기 10인 QoS 프로파일 생성
        self.vector3_publisher = self.create_publisher(Vector3, 'vector3', qos_profile)
        # Vector3 타입 메시지를 'vector3' 토픽으로 발행하는 퍼블리셔 생성

    def publish_vector3_msg(self, x, y, z):
        msg = Vector3()                             # Vector3 메시지 객체 생성
        msg.x = x                                   # x 성분 설정
        msg.y = y                                   # y 성분 설정
        msg.z = z                                   # z 성분 설정
        self.vector3_publisher.publish(msg)         # 'vector3' 토픽으로 메시지 발행
        self.get_logger().info(
            'Published message: x={0}, y={1}, z={2}'.format(msg.x, msg.y, msg.z))
        # 발행한 x, y, z 값을 터미널에 로그 출력


def main(args=None):
    rclpy.init(args=args)                           # ROS2 통신 초기화
    node = Vector3Publisher()                       # Vector3Publisher 노드 생성
    try:
        while rclpy.ok():                           # ROS2가 정상 동작하는 동안 반복
            try:
                raw = input('실수값 3개를 입력하세요 (x y z): ')
                # 사용자로부터 공백 구분 실수 3개 입력받기
                x, y, z = map(float, raw.split())   # 입력값을 float으로 변환하여 x, y, z에 저장
                node.publish_vector3_msg(x, y, z)   # 변환된 값으로 Vector3 메시지 퍼블리시
            except ValueError:
                node.get_logger().warn('올바른 형식으로 입력하세요. 예) 1.0 2.5 -3.0')
                # 잘못된 형식 입력 시 경고 출력 후 재입력 대기
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()                         # 노드 종료
        rclpy.shutdown()                            # ROS2 종료


if __name__ == '__main__':
    main()
```

---

### Subscriber 코드 (`vector3_subscriber.py`)

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Vector3  # 3차원 벡터 메시지 타입 임포트


class Vector3Subscriber(Node):
    def __init__(self):
        super().__init__('vector3_subscriber')      # 노드 이름 'vector3_subscriber'로 초기화
        qos_profile = QoSProfile(depth=10)          # 큐 크기 10인 QoS 프로파일 생성
        self.vector3_subscriber = self.create_subscription(
            Vector3,                                # 구독할 메시지 타입
            'vector3',                              # 구독할 토픽 이름
            self.subscribe_topic_message,           # 메시지 수신 시 호출할 콜백 함수
            qos_profile)

    def subscribe_topic_message(self, msg):
        self.get_logger().info(
            'Received message: x={0}, y={1}, z={2}'.format(msg.x, msg.y, msg.z))
        # 수신한 x, y, z 값을 터미널에 로그 출력


def main(args=None):
    rclpy.init(args=args)                           # ROS2 통신 초기화
    node = Vector3Subscriber()                      # Vector3Subscriber 노드 생성
    try:
        rclpy.spin(node)                            # 노드가 종료될 때까지 이벤트 루프 실행
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()                         # 노드 종료
        rclpy.shutdown()                            # ROS2 종료


if __name__ == '__main__':
    main()
```

---

### 실행 결과

[video2](https://github.com/user-attachments/assets/0baea285-de37-4a62-ae03-f8ace7355577)

---

## 과제 4

### 패키지: `my_teleop_rclpy_pkg`

`tty`와 `termios`를 사용해 Enter 없이 키 입력을 즉시 감지한다.  
`w`, `s`, `a`, `d` 키 입력에 따라 선속도(`linear.x`)와 각속도(`angular.z`) 값을 설정한 `Twist` 메시지를  
`/turtle1/cmd_vel` 토픽으로 퍼블리시하여 Turtlesim 거북이를 조종한다.  
정의되지 않은 키를 누르면 속도 0의 정지 명령을 발행하고, `q`를 누르면 노드를 종료한다.


---

### Publisher 코드 (`teleop_publisher.py`)

```python
import sys
import tty
import termios  # 터미널 설정 제어를 위한 모듈

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist  # 선속도/각속도를 담는 메시지 타입 임포트

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

LINEAR_SPEED = 0.5   # 전후 이동 선속도 상수
ANGULAR_SPEED = 1.0  # 좌우 회전 각속도 상수


def get_key(settings):
    tty.setraw(sys.stdin.fileno())  # 터미널을 raw 모드로 전환하여 Enter 없이 즉시 입력 감지
    key = sys.stdin.read(1)         # 키 한 글자 읽기
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    # 터미널 설정을 원래 상태로 복원
    return key                      # 읽은 키 반환


class TeleopPublisher(Node):
    def __init__(self):
        super().__init__('teleop_publisher')        # 노드 이름 'teleop_publisher'로 초기화
        qos_profile = QoSProfile(depth=10)          # 큐 크기 10인 QoS 프로파일 생성
        self.twist_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', qos_profile)
        # Twist 타입 메시지를 '/turtle1/cmd_vel' 토픽으로 발행하는 퍼블리셔 생성

    def publish_twist(self, linear_x, angular_z):
        msg = Twist()                               # Twist 메시지 객체 생성
        msg.linear.x = linear_x                    # 전후 선속도 설정 (양수: 전진, 음수: 후진)
        msg.angular.z = angular_z                   # 회전 각속도 설정 (양수: 좌회전, 음수: 우회전)
        self.twist_publisher.publish(msg)           # '/turtle1/cmd_vel' 토픽으로 메시지 발행
        self.get_logger().info(
            'linear.x: {0:.1f}, angular.z: {1:.1f}'.format(linear_x, angular_z))
        # 발행한 선속도, 각속도 값을 터미널에 로그 출력


def main(args=None):
    rclpy.init(args=args)                           # ROS2 통신 초기화
    node = TeleopPublisher()                        # TeleopPublisher 노드 생성
    settings = termios.tcgetattr(sys.stdin)         # 현재 터미널 설정 저장 (종료 시 복원용)
    print(MSG)                                      # 키 조작 안내 메시지 출력
    try:
        while rclpy.ok():                           # ROS2가 정상 동작하는 동안 반복
            key = get_key(settings)                 # 키보드 입력 한 글자 읽기
            if key == 'w':
                node.publish_twist(LINEAR_SPEED, 0.0)    # 전진
            elif key == 's':
                node.publish_twist(-LINEAR_SPEED, 0.0)   # 후진
            elif key == 'a':
                node.publish_twist(0.0, ANGULAR_SPEED)   # 좌회전
            elif key == 'd':
                node.publish_twist(0.0, -ANGULAR_SPEED)  # 우회전
            elif key == 'q':
                node.get_logger().info('종료합니다.')
                break                               # 'q' 입력 시 루프 탈출 후 종료
            else:
                node.publish_twist(0.0, 0.0)        # 정의되지 않은 키 → 정지 명령 발행
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.publish_twist(0.0, 0.0)               # 종료 전 거북이 정지 명령 발행
        node.destroy_node()                         # 노드 종료
        rclpy.shutdown()                            # ROS2 종료


if __name__ == '__main__':
    main()
```

---

### 실행 결과

[video3](https://github.com/user-attachments/assets/7480ad3a-826c-4980-a6ef-dd89e5893c45)
