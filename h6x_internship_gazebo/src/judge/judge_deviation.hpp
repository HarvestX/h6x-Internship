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

#pragma once

#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include <std_msgs/msg/bool.hpp>

class JudgeDeviation : public rclcpp::Node
{
public:
  explicit JudgeDeviation(const rclcpp::NodeOptions & options);

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & ptr);

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr data_pub_;
  image_transport::Subscriber sub_image_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(JudgeDeviation)
