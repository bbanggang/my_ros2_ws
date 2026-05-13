# ROS2 LAUNCH2

## 과제 1

### 패키지: `my_first_ros_rclpy_pkg`

`helloworld_publisher` 노드가 `helloworld` 토픽에 `String` 메시지를 1초 주기로 퍼블리시한다.  
메시지는 `"Hello World: {count}"` 형식으로, 퍼블리시할 때마다 카운트가 1씩 증가한다.  
`helloworld_subscriber` 노드는 `helloworld` 토픽을 구독하여 수신한 문자열을 터미널 로그로 출력한다.  
`helloworld_launch.py`를 통해 두 노드를 동시에 실행할 수 있다.

---

### Launch 파일 (`helloworld_launch.py`)

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_ros_rclpy_pkg',          # 실행할 노드가 속한 패키지 이름
            executable='hello_pub',                     # setup.py의 entry_points에 등록된 실행 이름
            name='helloworld_publisher',                # 실행 시 노드에 부여할 이름
            output='screen',                            # 로그를 터미널 화면에 출력
        ),
        Node(
            package='my_first_ros_rclpy_pkg',
            executable='hello_sub',
            name='helloworld_subscriber',
            output='screen',
        ),
    ])
```

---

### 실행 결과

![image1](https://github.com/user-attachments/assets/28fcb800-7736-4430-8a69-79e8de8fea53)

---

## 과제 2

### 시스템 개요

rapi5(Raspberry Pi 5)와 WSL 두 머신에서 각각 launch 파일로 여러 노드를 동시에 실행하고, 토픽을 통해 서로 데이터를 주고받는 자율주행 시스템을 구성한다.

- **rapi5**: 카메라 영상을 퍼블리시하고, WSL에서 받은 속도 명령으로 Dynamixel 모터를 구동한다.
- **WSL**: 라이다 데이터를 받아 주행 경로를 분석하고, 속도 명령을 rapi5로 전송한다.

---

### 노드별 동작 설명

#### rapi5 — `campub` 노드 (`camera_ros2` 패키지)

libcamera GStreamer 파이프라인으로 카메라(640×480, 30fps)를 캡처하여 JPEG 압축 후  
`Image_Topic` 토픽에 `CompressedImage` 메시지로 퍼블리시한다.


---

#### rapi5 — `dxl` 노드 (`dxl_nano` 패키지)

`vel_cmd_topic` 토픽에서 `Vector3` 메시지를 구독하여 Dynamixel 모터 2개의 속도를 제어한다.  
`msg->x`는 좌측 모터 RPM, `msg->y`는 우측 모터 RPM으로 사용하며, Dynamixel SDK로 SyncWrite 명령을 전송한다.


---

#### WSL — `sllidar_node` 노드 (`sllidar_ros2` 패키지)

라이다 하드웨어와 시리얼 통신하여 360° 거리 데이터를 `scan` 토픽에  
`LaserScan` 메시지로 퍼블리시한다. `lidardrive` 노드의 입력 소스가 된다.

---

#### WSL — `sllidar_client` 노드 (`lidardrive` 패키지)

`scan` 토픽에서 `LaserScan` 데이터를 수신하여 OpenCV로 2D 맵을 생성하고,  
좌우 경계선을 검출한 뒤 중심 오차(error)에 비례한 좌/우 모터 속도를 계산해  
`vel_cmd_topic`에 `Vector3`로 퍼블리시한다.


---

### rapi5 Launch 파일 (`rapi5_launch.py`)

rapi5에서 카메라 퍼블리셔와 Dynamixel 구독 노드를 동시에 실행한다.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros2',
            executable='pub',
            name='campub',              # 카메라 캡처 → Image_Topic 퍼블리시
            output='screen',
        ),
        Node(
            package='dxl_nano',
            executable='dxl',
            name='node_dxlsub',         # vel_cmd_topic 수신 → Dynamixel 모터 제어
            output='screen',
        ),
    ])
```

---

### WSL Launch 파일 (`wsl_lidardrive_launch.py`)

WSL에서 라이다 드라이버와 라인 검출 노드를 동시에 실행한다.  
`sllidar_node`가 퍼블리시하는 `scan` 토픽을 `sllidar_client`가 구독하여 연속으로 처리한다.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',        # 라이다 드라이버: scan 토픽 퍼블리시
            parameters=[{
                'serial_port': '/dev/ttyUSBLidar',
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'angle_compensate': True,
            }],
            output='screen',
        ),
        Node(
            package='lidardrive',
            executable='lidardrive',
            name='sllidar_client',      # scan 구독 → 라인 검출 → vel_cmd_topic 퍼블리시
            output='screen',
        ),
    ])
```

### 실행 결과

추후 추가

---

## 과제 3

### 런치파일 분석: `view_sllidar_c1_launch.py`

SLAMTEC LIDAR C1의 런치파일을 분석하여 파라미터 설정, 명령행 인자 설정 등의 기능을 설명한다.  
이 런치파일은 `sllidar_node`(라이다 드라이버)와 `rviz2`(시각화 도구) 두 노드를 동시에 실행하며,  
실행 시 명령행 인자로 포트, 보레이트 등 다양한 설정값을 동적으로 변경할 수 있다.

---

### 주요 기능 설명

#### 1. `LaunchConfiguration` — 런치 인자 값 참조

```python
serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
```

`LaunchConfiguration`은 런치 인자의 현재 값을 참조하는 객체다.  
`default` 값은 사용자가 인자를 전달하지 않았을 때 사용된다.  
이 객체를 `parameters`나 `arguments`에 전달하면 런치 시점에 실제 값으로 치환된다.

---

#### 2. `DeclareLaunchArgument` — 명령행 인자 선언

```python
DeclareLaunchArgument(
    'serial_port',
    default_value=serial_port,
    description='Specifying usb port to connected lidar')
```

런치 파일 실행 시 외부에서 값을 주입할 수 있는 인자를 선언한다.  
아래와 같이 실행 시 인자를 덮어쓸 수 있다.

```bash
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py serial_port:=/dev/ttyUSB1 serial_baudrate:=115200
```

| 인자명 | 기본값 | 설명 |
|---|---|---|
| `channel_type` | `serial` | 통신 채널 타입 (serial / udp) |
| `serial_port` | `/dev/ttyUSB0` | 라이다 연결 USB 포트 경로 |
| `serial_baudrate` | `460800` | 시리얼 통신 보레이트 |
| `frame_id` | `laser` | TF 좌표계 프레임 ID |
| `inverted` | `false` | 스캔 데이터 상하 반전 여부 |
| `angle_compensate` | `true` | 각도 보정 활성화 여부 |
| `scan_mode` | `Standard` | 라이다 스캔 모드 |

---

#### 3. `parameters` — 노드 파라미터 설정

```python
Node(
    ...
    parameters=[{
        'serial_port': serial_port,
        'serial_baudrate': serial_baudrate,
        ...
    }],
)
```

`parameters`에 딕셔너리를 전달하면 노드 실행 시 ROS2 파라미터 서버에 값이 등록된다.  
노드 내부에서 `declare_parameter` / `get_parameter`로 읽을 수 있으며,  
런치 인자(`LaunchConfiguration`)를 그대로 연결하면 명령행 입력값이 노드 파라미터로 전달된다.

---

#### 4. `get_package_share_directory` — 패키지 경로 동적 참조

```python
rviz_config_dir = os.path.join(
    get_package_share_directory('sllidar_ros2'),
    'rviz',
    'sllidar_ros2.rviz')
```

설치 환경마다 패키지 경로가 달라질 수 있으므로, 절대 경로를 하드코딩하지 않고  
`get_package_share_directory`로 패키지의 `share` 디렉터리 경로를 동적으로 획득한다.

---
