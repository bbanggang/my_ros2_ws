#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <chrono>
#include <rclcpp/executors.hpp>

std::string video_path = "/home/rapi5/ros2_ws/video/simulation/5_lt_cw_100rpm_out.mp4";


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("video_pub");
    // auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); //TCP
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); //UDP
    auto mypub = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile );
    
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::CompressedImage::SharedPtr msg;
    rclcpp::WallRate loop_rate(30.0);

    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Could not open video!");
        rclcpp::shutdown();
        return -1;
    }  
    cv::Mat frame;

    while(rclcpp::ok())
    {
        cap >> frame;
        if (frame.empty()) { RCLCPP_ERROR(node->get_logger(), "frame empty"); break;}

        hdr.stamp = node->get_clock()->now();
        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
        mypub->publish(*msg);
        loop_rate.sleep();
        rclcpp::spin_some(node);
    }
    cap.release();
    rclcpp::shutdown();
    return 0;
}
