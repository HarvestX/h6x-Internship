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

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

#include "game_define.hpp"

class GameMaster : public rclcpp::Node
{
public:
  explicit GameMaster(const rclcpp::NodeOptions &);

  void onDeviationCount(const std_msgs::msg::Int32::SharedPtr ptr);
  void onStatus(const std_msgs::msg::Int32::SharedPtr ptr);

  void timerCountUpdate();
  void scoreUpdateAndPublish();

  void gameMasterStatus();
  void gameMasterTime();
  void gameMasterScore();

  void publishVehicleStatus();

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_deviation_;

  // 3 state --------------------------------------------------
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_status_;

  // deviation ------------------------------------------------
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_deviation_count_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_charge_;


  rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr pub_log_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_score_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_time_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed_limit_;

  rclcpp::TimerBase::SharedPtr timer_;

  int64_t game_status_;
  bool game_over_was_sent_;

  bool tmp_charge_flag_;

  // deviation ------------------------------------------------
  int64_t count_;
  int64_t game_over_score_;

  int64_t tmp_deviation_count_;
  int64_t deviation_count_;

  int64_t past_time_;
  int64_t score_;
  int64_t charge_score_;

  int64_t vehicle_status_;
  // for vehicle speed limit (normal: 1.0, slow: 0.5, fast: 2.0, stop: 0.0)
  float speed_limit_;

  std::string prefix_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(GameMaster)
