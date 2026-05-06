# ROS2 RCLPY

## 과제 1

`colcon build --symlink-install` 옵션 유무에 따른 소스코드 변경 시 재빌드 필요 여부를 설명한다.

---

### 1. `--symlink-install` 옵션으로 빌드한 경우

`--symlink-install` 옵션을 사용하면 install 디렉터리에 실제 파일이 복사되지 않고 **소스 파일을 가리키는 심볼릭 링크**가 생성된다.  
따라서 Python 소스코드(`.py`)를 수정하면 **재빌드 없이 즉시 반영**된다.

> **이유**: ROS2가 실행 시점에 심볼릭 링크를 통해 원본 소스 파일을 직접 참조하기 때문에, 소스를 수정하면 다음 실행 때 변경 사항이 자동으로 적용된다.  
> 단, `package.xml`, `setup.py`, `setup.cfg` 등 빌드 설정 파일을 변경한 경우에는 재빌드가 필요하다.

```bash
colcon build --symlink-install --packages-select <패키지명>
# 이후 소스코드 수정 → 재빌드 없이 바로 실행 가능
ros2 run <패키지명> <노드명>
```

---

### 2. `--symlink-install` 옵션 없이 빌드한 경우

`--symlink-install` 없이 빌드하면 소스 파일이 install 디렉터리로 **직접 복사**된다.  
따라서 소스코드를 수정해도 install 디렉터리의 복사본은 변경되지 않으므로 **반드시 재빌드가 필요**하다.

> **이유**: ROS2는 install 디렉터리의 복사된 파일을 실행하기 때문에, 원본 소스를 수정해도 재빌드하여 install에 다시 복사하지 않으면 변경 사항이 반영되지 않는다.

```bash
colcon build --packages-select <패키지명>
# 소스코드 수정 후 반드시 재빌드 필요
colcon build --packages-select <패키지명>
ros2 run <패키지명> <노드명>
```

---

## 과제 2

`my_int_rclpy_pkg` — 정수값을 0으로 초기화하고 1씩 증가시키며 퍼블리시하고 서브스크라이브하는 패키지

---

### `IntPublisher.__init__()`

`int_publisher` 노드를 초기화하고, `counter` 토픽에 `Int32` 메시지를 발행할 퍼블리셔와 1초 주기 타이머를 생성한다.

```python
def __init__(self):
    super().__init__('int_publisher')          # 노드 이름을 'int_publisher'로 설정
    qos_profile = QoSProfile(depth=10)         # 큐 크기 10인 QoS 프로파일 생성
    self.int_publisher = self.create_publisher(Int32, 'counter', qos_profile)
    # Int32 타입으로 'counter' 토픽에 퍼블리셔 등록
    self.timer = self.create_timer(1, self.publish_int_msg)
    # 1초마다 publish_int_msg 콜백 호출하는 타이머 생성
    self.count = 0                             # 퍼블리시할 정수값을 0으로 초기화
```

---

### `IntPublisher.publish_int_msg()`

타이머 콜백으로, `Int32` 메시지에 현재 카운트 값을 담아 퍼블리시하고 1씩 증가시킨다.

```python
def publish_int_msg(self):
    msg = Int32()                              # Int32 메시지 객체 생성
    msg.data = self.count                      # 현재 카운트 값을 메시지에 저장
    self.int_publisher.publish(msg)            # 'counter' 토픽으로 메시지 발행
    self.get_logger().info('Published message: {0}'.format(msg.data))
    # 발행한 값을 로그로 출력
    self.count += 1                            # 다음 발행을 위해 카운트 1 증가
```

---

### `IntSubscriber.__init__()`

`int_subscriber` 노드를 초기화하고, `counter` 토픽을 구독할 서브스크라이버를 생성한다.

```python
def __init__(self):
    super().__init__('int_subscriber')         # 노드 이름을 'int_subscriber'로 설정
    qos_profile = QoSProfile(depth=10)         # 큐 크기 10인 QoS 프로파일 생성
    self.int_subscriber = self.create_subscription(
        Int32,                                 # 구독할 메시지 타입
        'counter',                             # 구독할 토픽 이름
        self.subscribe_topic_message,          # 메시지 수신 시 호출할 콜백 함수
        qos_profile)
```

---

### `IntSubscriber.subscribe_topic_message()`

`counter` 토픽에서 수신한 `Int32` 메시지의 값을 로그로 출력하는 콜백 함수다.

```python
def subscribe_topic_message(self, msg):
    self.get_logger().info('Received message: {0}'.format(msg.data))
    # 수신한 정수값을 로그로 출력
```

---

### 실행 결과

추후 추가

---

## 과제 3

`my_vector3_rclpy_pkg` — 키보드로 실수값 3개를 입력받아 `geometry_msgs/msg/Vector3` 메시지로 퍼블리시하고 서브스크라이브하는 패키지

---

### `Vector3Publisher.__init__()`

`vector3_publisher` 노드를 초기화하고, `vector3` 토픽에 `Vector3` 메시지를 발행할 퍼블리셔를 생성한다.

```python
def __init__(self):
    super().__init__('vector3_publisher')      # 노드 이름을 'vector3_publisher'로 설정
    qos_profile = QoSProfile(depth=10)         # 큐 크기 10인 QoS 프로파일 생성
    self.vector3_publisher = self.create_publisher(Vector3, 'vector3', qos_profile)
    # Vector3 타입으로 'vector3' 토픽에 퍼블리셔 등록
```

---

### `Vector3Publisher.publish_vector3_msg()`

입력받은 x, y, z 실수값을 `Vector3` 메시지에 담아 퍼블리시한다.

```python
def publish_vector3_msg(self, x, y, z):
    msg = Vector3()                            # Vector3 메시지 객체 생성
    msg.x = x                                  # x 성분 설정
    msg.y = y                                  # y 성분 설정
    msg.z = z                                  # z 성분 설정
    self.vector3_publisher.publish(msg)        # 'vector3' 토픽으로 메시지 발행
    self.get_logger().info(
        'Published message: x={0}, y={1}, z={2}'.format(msg.x, msg.y, msg.z))
    # 발행한 x, y, z 값을 로그로 출력
```

---

### `main()` — Vector3Publisher

ROS2를 초기화하고 노드를 생성한 뒤, 키보드 입력을 반복적으로 받아 Vector3 메시지를 퍼블리시한다.

```python
def main(args=None):
    rclpy.init(args=args)                      # ROS2 초기화
    node = Vector3Publisher()                  # 퍼블리셔 노드 생성
    try:
        while rclpy.ok():                      # ROS2가 정상 동작하는 동안 반복
            try:
                raw = input('실수값 3개를 입력하세요 (x y z): ')
                # 사용자로부터 공백 구분 실수 3개 입력받기
                x, y, z = map(float, raw.split())
                # 입력 문자열을 분리하여 각각 float로 변환
                node.publish_vector3_msg(x, y, z)
                # 변환된 값으로 메시지 퍼블리시
            except ValueError:
                node.get_logger().warn('올바른 형식으로 입력하세요. 예) 1.0 2.5 -3.0')
                # 변환 실패 시 경고 출력 후 재입력 대기
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()                    # 노드 종료
        rclpy.shutdown()                       # ROS2 종료
```

---

### `Vector3Subscriber.__init__()`

`vector3_subscriber` 노드를 초기화하고, `vector3` 토픽을 구독할 서브스크라이버를 생성한다.

```python
def __init__(self):
    super().__init__('vector3_subscriber')     # 노드 이름을 'vector3_subscriber'로 설정
    qos_profile = QoSProfile(depth=10)         # 큐 크기 10인 QoS 프로파일 생성
    self.vector3_subscriber = self.create_subscription(
        Vector3,                               # 구독할 메시지 타입
        'vector3',                             # 구독할 토픽 이름
        self.subscribe_topic_message,          # 메시지 수신 시 호출할 콜백 함수
        qos_profile)
```

---

### `Vector3Subscriber.subscribe_topic_message()`

`vector3` 토픽에서 수신한 `Vector3` 메시지의 x, y, z 값을 로그로 출력하는 콜백 함수다.

```python
def subscribe_topic_message(self, msg):
    self.get_logger().info(
        'Received message: x={0}, y={1}, z={2}'.format(msg.x, msg.y, msg.z))
    # 수신한 x, y, z 값을 로그로 출력
```

---

### 실행 결과

추후 추가

---

## 과제 4

`my_teleop_rclpy_pkg` — 키보드 입력으로 `/turtle1/cmd_vel` 토픽에 `geometry_msgs/msg/Twist` 메시지를 발행하여 Turtlesim을 조종하는 패키지

---

### `get_key()`

터미널을 raw 모드로 전환하여 Enter 없이 키 입력 한 글자를 즉시 읽어 반환한다.

```python
def get_key(settings):
    tty.setraw(sys.stdin.fileno())             # 터미널을 raw 모드로 전환 (즉시 입력 감지)
    key = sys.stdin.read(1)                    # 키 한 글자 읽기
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    # 터미널 설정을 원래 상태로 복원
    return key                                 # 읽은 키 반환
```

---

### `TeleopPublisher.__init__()`

`teleop_publisher` 노드를 초기화하고, `/turtle1/cmd_vel` 토픽에 `Twist` 메시지를 발행할 퍼블리셔를 생성한다.

```python
def __init__(self):
    super().__init__('teleop_publisher')       # 노드 이름을 'teleop_publisher'로 설정
    qos_profile = QoSProfile(depth=10)         # 큐 크기 10인 QoS 프로파일 생성
    self.twist_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', qos_profile)
    # Twist 타입으로 '/turtle1/cmd_vel' 토픽에 퍼블리셔 등록
```

---

### `TeleopPublisher.publish_twist()`

선속도와 각속도 값을 `Twist` 메시지에 담아 `/turtle1/cmd_vel` 토픽으로 퍼블리시한다.

```python
def publish_twist(self, linear_x, angular_z):
    msg = Twist()                              # Twist 메시지 객체 생성
    msg.linear.x = linear_x                   # 전후 선속도 설정 (양수: 전진, 음수: 후진)
    msg.angular.z = angular_z                  # 회전 각속도 설정 (양수: 좌회전, 음수: 우회전)
    self.twist_publisher.publish(msg)          # '/turtle1/cmd_vel' 토픽으로 메시지 발행
    self.get_logger().info(
        'linear.x: {0:.1f}, angular.z: {1:.1f}'.format(linear_x, angular_z))
    # 발행한 선속도, 각속도 값을 로그로 출력
```

---

### `main()` — TeleopPublisher

ROS2를 초기화하고 키보드 입력을 반복적으로 감지하여 키에 따라 Twist 메시지를 퍼블리시한다.

```python
def main(args=None):
    rclpy.init(args=args)                      # ROS2 초기화
    node = TeleopPublisher()                   # teleop 퍼블리셔 노드 생성
    settings = termios.tcgetattr(sys.stdin)    # 현재 터미널 설정 저장 (복원용)
    print(MSG)                                 # 키 조작 안내 메시지 출력
    try:
        while rclpy.ok():                      # ROS2가 정상 동작하는 동안 반복
            key = get_key(settings)            # 키보드 입력 한 글자 읽기
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
                break                          # 'q' 입력 시 루프 탈출
            else:
                node.publish_twist(0.0, 0.0)  # 그 외 키는 정지
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.publish_twist(0.0, 0.0)          # 종료 전 거북이 정지 명령 발행
        node.destroy_node()                    # 노드 종료
        rclpy.shutdown()                       # ROS2 종료
```

---

### 실행 결과

추후 추가
