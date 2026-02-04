#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/header.hpp"
#include <memory>
#include <chrono>
#include <opencv2/videoio.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rate.hpp>

std::string src = "libcamerasrc ! \
	video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! \
    queue ! videoconvert ! videoflip method=rotate-180 ! \
    videoconvert ! video/x-raw, format=(string)BGR ! appsink";

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("campub");
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  auto pub_raw = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/raw/compressed", qos_profile);
  auto pub_gray = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/gray/compressed", qos_profile);
  auto pub_bin = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/binary/compressed", qos_profile);
  
  

  cv::VideoCapture cap(src, cv::CAP_GSTREAMER);
  if (!cap.isOpened()) 
  {
    RCLCPP_ERROR(node->get_logger(), "Could not open video!");
    rclcpp::shutdown();
    return -1;
  }

  cv::Mat frame, gray_frame, bin_frame;
  rclcpp::WallRate loop_rate(30.0);

  while(rclcpp::ok())
  {
    cap >> frame;
    if (frame.empty()) { RCLCPP_ERROR(node->get_logger(), "frame empty"); break;}
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
    cv::threshold(gray_frame, bin_frame, 127, 255, cv::THRESH_BINARY);

    std_msgs::msg::Header hdr;
    hdr.stamp = node->get_clock()->now();
    hdr.frame_id = "camera_frame";
    auto msg_raw = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
    auto msg_gray = cv_bridge::CvImage(hdr, "mono8", gray_frame).toCompressedImageMsg();
    auto msg_bin = cv_bridge::CvImage(hdr, "mono8", bin_frame).toCompressedImageMsg();

    // 3. 메시지 발행
    pub_raw->publish(*msg_raw);
    pub_gray->publish(*msg_gray);
    pub_bin->publish(*msg_bin);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
