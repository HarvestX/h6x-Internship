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
  JudgeGoal();

  // データの受信を行うのみ ---------------------------------------------------
  void start_status_callback(const std_msgs::msg::Bool::SharedPtr ptr);
  void goal_status_callback(const std_msgs::msg::Bool::SharedPtr ptr);

  // コースアウト時の挙動
  void line_courseout_callback(const std_msgs::msg::Bool::SharedPtr ptr);

  // timer --------------------------------------------------
  void timer_callback();

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr courseout_count_pub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_goal_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_courseout_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool started_;
  bool goal_;

  int64_t courseout_count_;
};
