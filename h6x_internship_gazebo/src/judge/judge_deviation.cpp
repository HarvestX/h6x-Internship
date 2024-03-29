// Copyright 2022 HarvestX Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "judge_deviation.hpp"

JudgeDeviation::JudgeDeviation(const rclcpp::NodeOptions & options)
: Node("judge_deviation", options)
{
  rmw_qos_profile_t qos_sensor = rmw_qos_profile_sensor_data;
  this->sub_image_ = image_transport::create_subscription(
    this, "/camera_linetrace/camera1/image_raw",
    std::bind(
      &JudgeDeviation::onImage, this, std::placeholders::_1),
    "raw", qos_sensor);

  data_pub_ =
    this->create_publisher<std_msgs::msg::Bool>("/judge_deviation/data", 10);
}

void JudgeDeviation::onImage(
  const sensor_msgs::msg::Image::ConstSharedPtr & ptr)
{
  cv_bridge::CvImagePtr cv_ptr;
  std_msgs::msg::Bool msg_data;
  cv::Mat frame;

  try {
    auto img = cv_bridge::toCvCopy(ptr, "bgr8");
    frame = img->image;
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // process image
  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  // 0 ~ 100 : ok
  // width = 100, height = 1
  int black_count = 0;
  for (int i = 0; i < gray.cols; i++) {
    if (gray.at<uchar>(0, i) < 100) {
      black_count++;
    }
  }

  if (black_count < 5) {
    msg_data.data = true;
    data_pub_->publish(msg_data);
  } else {
    data_pub_->publish(msg_data);
  }
}
