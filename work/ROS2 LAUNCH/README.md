# ROS2 LAUNCH

## 과제 1

### 시스템 개요

카메라 영상을 보면서 키보드로 로봇을 조종하는 시스템을 launch 파일로 구성한다.  
rapi5에서 카메라 영상을 퍼블리시하고, WSL2에서 키보드 입력을 받아 속도 명령을 퍼블리시한다.  
두 머신은 `Image_Topic`과 `vel_cmd_topic` 두 토픽으로 데이터를 주고받는다.

```
[rapi5]                                   [WSL2]
  campub  ──── Image_Topic ──────────────>  camsub_wsl  (화면 표시)
  dxl     <─── vel_cmd_topic ────────────  node_dxlpub (키보드 조종)
```

---

### 노드별 동작 설명

#### rapi5 — `campub` 노드 (`camera_ros2` 패키지)

libcamera GStreamer 파이프라인으로 카메라(640×480, 30fps)를 캡처하여 JPEG 압축 후  
`Image_Topic` 토픽에 `CompressedImage` 메시지로 퍼블리시한다. QoS는 BEST_EFFORT를 사용한다.

---

#### rapi5 — `dxl` 노드 (`dxl_nano` 패키지)

`vel_cmd_topic` 토픽에서 `Vector3` 메시지를 구독하여 Dynamixel 모터 2개의 속도를 제어한다.  
`msg->x`는 좌측 모터 RPM, `msg->y`는 우측 모터 RPM으로 사용하며, Dynamixel SDK로 SyncWrite 명령을 전송한다.

---

#### WSL2 — `camsub_wsl` 노드 (`camera_ros2` 패키지)

`Image_Topic` 토픽에서 `CompressedImage` 메시지를 구독하여 OpenCV로 디코딩한 후 화면에 출력한다.  
BEST_EFFORT QoS를 사용하여 rapi5와 QoS가 일치한다.



---

#### WSL2 — `node_dxlpub` 노드 (`dxl_wsl` 패키지)

Enter 없이 키 입력을 즉시 감지하여 좌/우 모터 속도를 계산하고 `vel_cmd_topic`에 `Vector3`로 퍼블리시한다.  
가속/감속 처리가 포함되어 있어 키를 누를 때마다 목표 속도에 5RPM씩 점진적으로 수렴한다.



| 키 | 동작 | vel.x (좌) | vel.y (우) |
|---|---|---|---|
| `f` | 전진 | +50 | -50 |
| `b` | 후진 | -50 | +50 |
| `l` | 좌회전 | -50 | -50 |
| `r` | 우회전 | +50 | +50 |
| `s` / `Space` | 정지 | 0 | 0 |

---

### rapi5 Launch 파일 (`nano_launch.py`)

rapi5에서 카메라 퍼블리셔와 Dynamixel 구독 노드를 동시에 실행한다.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros2',
            executable='pub',
            name='campub',              # 카메라 캡처 후 Image_Topic으로 퍼블리시
            output='screen',
        ),
        Node(
            package='dxl_nano',
            executable='dxl',
            name='node_dxlsub',         # vel_cmd_topic 수신 후 Dynamixel 모터 제어
            output='screen',
        ),
    ])
```

---

### WSL2 Launch 파일 (`wsl2_launch.py`)

WSL2에서 카메라 구독 노드와 키보드 조종 노드를 동시에 실행한다.  
`xterm`으로 `node_dxlpub`를 별도 터미널에 띄워 키보드 입력을 분리한다.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros2',
            executable='sub',
            name='camsub_wsl',          # Image_Topic 수신 후 OpenCV 화면 출력
            output='screen',
        ),
        Node(
            package='dxl_wsl',
            executable='pub',
            name='node_dxlpub',         # 키보드 입력 → vel_cmd_topic 퍼블리시
            output='screen',
            prefix='xterm -fa "Monospace" -fs 11 -e',  # 키보드 입력을 위해 별도 터미널 실행
        ),
    ])
```

> **`prefix` 사용 이유**: `node_dxlpub`는 키보드 입력을 직접 받아야 하므로 별도 xterm 창에 실행해야 한다.  
> launch 터미널에서 실행하면 키 입력이 정상적으로 전달되지 않는다.

---

### 실행 방법

```bash
# rapi5에서
ros2 launch nano_launch.py

# WSL2에서
ros2 launch wsl2_launch.py
```


---

## 과제 2

### 패키지: `my_int_rclpy_pkg`

`turtlesim_mimic_launch.py`를 참고하여 `my_int_rclpy_pkg`의 `int_publisher`와 `int_subscriber` 두 노드를  
launch 파일로 동시에 실행하고 topic list를 확인한다.  
`int_publisher` 노드가 `counter` 토픽에 `Int32` 메시지를 1초 주기로 퍼블리시하고,  
`int_subscriber` 노드가 해당 토픽을 구독하여 수신값을 터미널에 출력한다.

---

### 참고: `turtlesim_mimic_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim',
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim',
            ros_arguments=['--log-level', 'warn']
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```

---

### Launch 파일 (`int_pubsub_launch.py`)

`turtlesim_mimic_launch.py`와 동일한 구조로, `Node()` 항목을 추가하는 것만으로 노드를 늘릴 수 있다.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_int_rclpy_pkg',     # 실행할 노드가 속한 패키지 이름
            executable='int_pub',            # setup.py의 entry_points에 등록된 실행 이름
            name='int_publisher',            # 실행 시 노드에 부여할 이름
            output='screen',                 # 로그를 터미널 화면에 출력
        ),
        Node(
            package='my_int_rclpy_pkg',
            executable='int_sub',
            name='int_subscriber',
            output='screen',
        ),
    ])
```

---

### 실행 명령어

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch /home/linux/ros2_ws/launch/int_pubsub_launch.py
```

---

### 실행 결과

```
[INFO] [int_pub-1]: process started with pid [1959]
[INFO] [int_sub-2]: process started with pid [1960]
[int_pub-1] [INFO] [int_publisher]: Published message: 0
[int_sub-2] [INFO] [int_subscriber]: Received message: 0
[int_pub-1] [INFO] [int_publisher]: Published message: 1
[int_sub-2] [INFO] [int_subscriber]: Received message: 1
[int_pub-1] [INFO] [int_publisher]: Published message: 2
[int_sub-2] [INFO] [int_subscriber]: Received message: 2
```

---

### `ros2 topic list` 확인

다른 터미널에서 아래 명령어로 토픽 목록과 상세 정보를 확인한다.

```bash
$ ros2 topic list
/counter
/parameter_events
/rosout

$ ros2 topic info /counter
Type: std_msgs/msg/Int32
Publisher count: 1
Subscription count: 1
```

> `Publisher count: 1`, `Subscription count: 1` → `int_publisher`와 `int_subscriber` 두 노드가  
> 동시에 실행되어 `/counter` 토픽으로 데이터를 주고받는 것을 확인할 수 있다.

---

### 실행 결과

![image1](https://github.com/user-attachments/assets/8394b3f2-03dd-4641-b659-89898fb0a3fd)
