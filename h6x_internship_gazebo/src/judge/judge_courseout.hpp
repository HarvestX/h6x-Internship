#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/msg/bool.hpp>

class JudgeCourseout : public rclcpp::Node
{
public:
  JudgeCourseout();

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & ptr);

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr data_pub_;
  image_transport::Subscriber sub_image_;
};
