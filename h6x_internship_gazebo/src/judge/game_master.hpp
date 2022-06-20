#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

#include "game_define.hpp"

class GameMaster : public rclcpp::Node
{
public:
  GameMaster();

  void courseout_count_callback(const std_msgs::msg::Int32::SharedPtr ptr);
  void status_callback(const std_msgs::msg::Int32::SharedPtr ptr);

  void timer_count_update();
  void score_update_and_publish();

  void game_master_status();
  void game_master_time();
  void game_master_score();

  void publish_viecle_status();

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_courseout_;

  // 3 state --------------------------------------------------
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_status_;

  // courseout --------------------------------------------------
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_courseout_count_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_charge_;


  rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr pub_log_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_score_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_time_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed_limit_;

  rclcpp::TimerBase::SharedPtr timer_;

  int64_t game_status_;
  bool gameover_was_sent_;

  bool tmp_charge_flag_;

  // courseout --------------------------------------------------
  int64_t count_;
  int64_t gameover_socre_;

  int64_t tmp_courseout_count_;
  int64_t courseout_count_;

  int64_t past_time_;
  int64_t score_;
  int64_t charge_score_;

  int64_t viecle_status_;
  float speed_limit_;   // for viecle speed limit (normal: 1.0, slow: 0.5, fast: 2.0, stop: 0.0)

  std::string prefix_;
};
