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
// 説明 --------------------------------------------------
// judge_goalは、スタート前、後、ゴール後の状態遷移とスコア用のコースアウトカウントを行い、2つのデータを送信する。

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include "game_define.hpp"

class JudgeGoal : public rclcpp::Node
{
public:
  explicit JudgeGoal(const rclcpp::NodeOptions &);

  // データの受信を行うのみ ---------------------------------------------------
  void onStartStatus(const std_msgs::msg::Bool::SharedPtr ptr);
  void onGoalStatus(const std_msgs::msg::Bool::SharedPtr ptr);

  // コースアウト時の挙動
  void onLineDeviation(const std_msgs::msg::Bool::SharedPtr ptr);

  // timer --------------------------------------------------
  void timer_callback();

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr deviation_count_pub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_goal_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_deviation_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool started_;
  bool goal_;

  int64_t deviation_count_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(JudgeGoal)
