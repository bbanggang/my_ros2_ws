#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "dxl_nano/dxl.hpp"
#include <memory>
#include <functional>
using namespace std::placeholders;
void mysub_callback(rclcpp::Node::SharedPtr node, Dxl& dxl, const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    dxl.setVelocity((int)msg->x, (int)msg->y);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
    RCLCPP_INFO(node->get_logger(), "Received message: %.2lf,%.2lf Execution time: %.3f ms", msg->x, msg->y, elapsed.count());
}

int main(int argc, char* argv[])    
{

    rclcpp::init(argc, argv);
    Dxl dxl;
    auto node = std::make_shared<rclcpp::Node>("node_dxlsub");
    if(!dxl.open())
    {
    RCLCPP_ERROR(node->get_logger(), "dynamixel open error");
    rclcpp::shutdown();
    return -1;
    }
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    std::function<void(const geometry_msgs::msg::Vector3::SharedPtr msg)>
    fn;
    fn = std::bind(mysub_callback, node, dxl, _1);
    auto mysub = node->create_subscription<geometry_msgs::msg::Vector3>("vel_cmd_topic",qos_profile,fn);
    rclcpp::spin(node);
    dxl.close();
    rclcpp::shutdown();
    return 0;
}