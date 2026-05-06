# dxl_nano 패키지 설명

## 전체 프로그램 동작 설명

`dxl_sub.cpp`는 ROS2 기반 Dynamixel 모터 제어 노드로 `vel_cmd_topic`의 `geometry_msgs::msg::Vector3` 메시지를 수신하여, 실제 Dynamixel 모터에 좌/우 바퀴 속도를 전달

노드 구조는 클래스 기반이 아닌 함수 기반으로 구성되어 있으며, `main()`에서 노드 생성·구독 설정·모터 초기화를 모두 처리하고, 콜백 함수 `mysub_callback()`에서 수신된 속도 값을 Dynamixel에 직접 전달한다.

---

## 함수별 설명

### 1. `mysub_callback()` — 속도 명령 수신 콜백

```cpp
void mysub_callback(rclcpp::Node::SharedPtr node, Dxl& dxl,
                    const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    // 모터 제어 지연 시간 측정을 위한 시작 시각 기록
    auto start_time = std::chrono::high_resolution_clock::now();

    // double → int 캐스팅하여 정수 속도 단위로 변환
    dxl.setVelocity((int)msg->x, (int)msg->y);

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end_time - start_time;

    // 수신된 속도 값과 setVelocity() 실행 시간(ms)을 로깅
    RCLCPP_INFO(node->get_logger(),
                "Received message: %.2lf,%.2lf Execution time: %.3f ms",
                msg->x, msg->y, elapsed.count());
}
```

---

### 2. `main()` — 진입점 (노드 생성 및 모터 초기화)

```cpp
int main(int argc, char* argv[])    
{
    rclcpp::init(argc, argv);

    // Dynamixel 모터 제어 객체 생성
    // dxl_nano 라이브러리의 Dxl 클래스가 시리얼 통신을 담당
    Dxl dxl;
    auto node = std::make_shared<rclcpp::Node>("node_dxlsub");

    // Dynamixel 포트 열기 실패 시 에러 로깅 후 즉시 종료
    if(!dxl.open())
    {
        RCLCPP_ERROR(node->get_logger(), "dynamixel open error");
        rclcpp::shutdown();
        return -1;
    }

    // BestEffort: 실시간 모터 제어에서 지연보다 최신 데이터 수신을 우선
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // std::bind로 콜백에 node, dxl 객체를 바인딩
    std::function<void(const geometry_msgs::msg::Vector3::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, dxl, _1);

    auto mysub = node->create_subscription<geometry_msgs::msg::Vector3>(
        "vel_cmd_topic", qos_profile, fn);

    rclcpp::spin(node);

    // 종료 시 Dynamixel 포트를 닫고 ROS2 정리
    dxl.close();
    rclcpp::shutdown();
    return 0;
}
```