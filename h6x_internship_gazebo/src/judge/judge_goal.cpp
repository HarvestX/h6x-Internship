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

#include "judge_goal.hpp"

JudgeGoal::JudgeGoal(const rclcpp::NodeOptions & options)
: Node("judge_goal", options)
{
  started_ = false;
  goal_ = false;
  this->deviation_count_ = 0;

  rclcpp::QoS _qos = rclcpp::SensorDataQoS().reliable();

  sub_start_ =
    this->create_subscription<std_msgs::msg::Bool>(
    "/start/touched", _qos,
    std::bind(&JudgeGoal::onStartStatus, this, std::placeholders::_1));
  sub_goal_ =
    this->create_subscription<std_msgs::msg::Bool>(
    "/goal/touched", _qos,
    std::bind(&JudgeGoal::onGoalStatus, this, std::placeholders::_1));

  this->sub_deviation_ = this->create_subscription<std_msgs::msg::Bool>(
    "/judge_deviation/data", 10, std::bind(
      &JudgeGoal::onLineDeviation, this,
      std::placeholders::_1));

  // define timer
  timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&JudgeGoal::timer_callback, this));

  // define publisher
  status_pub_ =
    this->create_publisher<std_msgs::msg::Int32>("/judge_status", 10);
  deviation_count_pub_ =
    this->create_publisher<std_msgs::msg::Int32>("/deviation_count", 10);
}

// データの受信を行うのみ ---------------------------------------------------
void JudgeGoal::onStartStatus(const std_msgs::msg::Bool::SharedPtr ptr)
{
  if (ptr->data) {
    this->started_ = true;
  }
}

void JudgeGoal::onGoalStatus(const std_msgs::msg::Bool::SharedPtr ptr)
{
  // スタート済が前提
  if (ptr->data && this->started_) {
    this->goal_ = true;
  }
}

// コースアウト時の挙動
void JudgeGoal::onLineDeviation(const std_msgs::msg::Bool::SharedPtr ptr)
{
  // ゴール済かスタート前は無視
  if (this->goal_ || !this->started_) {
    return;
  }

  if (ptr->data) {     // deviation
    this->deviation_count_++;
  }
}

// timer --------------------------------------------------

void JudgeGoal::timer_callback()
{
  std_msgs::msg::Int32 msg_data;
  std_msgs::msg::Int32 msg_deviation;

  if (goal_) {
    msg_data.data = GOAL;
  } else if (started_) {
    msg_data.data = START;
  } else {
    msg_data.data = READY;
  }

  status_pub_->publish(msg_data);

  // deviation
  msg_deviation.data = this->deviation_count_;
  this->deviation_count_pub_->publish(msg_deviation);
}
