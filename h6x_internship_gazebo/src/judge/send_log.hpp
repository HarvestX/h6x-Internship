#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

#define DEBUG 10
#define INFO 20
#define WARN 30
#define ERROR 40
#define FATAL 50

#define GAMEOVER_SCORE 100
#define SPEED_LIMIT_NORMAL 1.0

class SendLog : public rclcpp::Node
{
public:
    SendLog();

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_courseout_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_goal_;
    rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr pub_log_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_score_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_time_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed_limit_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool started_;
    bool goal_flag_;
    bool gameover_was_sent_;

    unsigned int count_;

    unsigned int couseout_count_;
    unsigned int past_time_;
    unsigned int score_;

    int viecle_status_;
    float speed_limit_; // for viecle speed limit (normal: 1.0, slow: 0.5, fast: 2.0, stop: 0.0)

    std::string prefix_;

    void line_courseout_callback(const std_msgs::msg::Bool::SharedPtr ptr);
    void line_goal_callback(const std_msgs::msg::Bool::SharedPtr ptr);
    void time_update_callback();
    void score_update_callback();

    void viecle_status_callback(const unsigned int _score);
};