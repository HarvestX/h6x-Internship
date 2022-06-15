// 説明 --------------------------------------------------
// judge_goalは、スタート前、後、ゴール後の状態遷移とスコア用のコースアウトカウントを行い、2つのデータを送信する。

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include "game_define.hpp"

class JudgeGoal : public rclcpp::Node
{
public:
    JudgeGoal()
        : Node("judge_goal")
    {
        started_ = false;
        goal_ = false;
        courseout_count_ = 0;

        rclcpp::QoS _qos = rclcpp::SensorDataQoS().reliable();
        sub_start_ = this->create_subscription<std_msgs::msg::Bool>("/start/touched", _qos, std::bind(&JudgeGoal::start_status_callback, this, std::placeholders::_1));
        sub_goal_ = this->create_subscription<std_msgs::msg::Bool>("/goal/touched", _qos, std::bind(&JudgeGoal::goal_status_callback, this, std::placeholders::_1));

        sub_courseout_ = this->create_subscription<std_msgs::msg::Bool>("/judge_courseout/data", 10, std::bind(&JudgeGoal::line_courseout_callback, this, std::placeholders::_1));

        // define timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&JudgeGoal::timer_callback, this));

        // define publisher
        status_pub_ = this->create_publisher<std_msgs::msg::Int32>("/judge_status", 10);
        courseout_count_pub_ = this->create_publisher<std_msgs::msg::Int32>("/courseout_count", 10);
    }

    // データの受信を行うのみ ---------------------------------------------------
    void start_status_callback(const std_msgs::msg::Bool::SharedPtr ptr)
    {
        if (ptr->data)
        {
            started_ = true;
        }
    }

    void goal_status_callback(const std_msgs::msg::Bool::SharedPtr ptr)
    {
        // スタート済が前提
        if (ptr->data && started_)
        {
            goal_ = true;
        }
    }

    // コースアウト時の挙動
    void line_courseout_callback(const std_msgs::msg::Bool::SharedPtr ptr)
    {
        // ゴール済かスタート前は無視
        if (goal_ || !started_)
        {
            return;
        }

        if (ptr->data) // courseout
        {
            courseout_count_++;
        }
    }

    // timer --------------------------------------------------

    void timer_callback()
    {
        std_msgs::msg::Int32 msg_data;
        std_msgs::msg::Int32 msg_couseout;

        if (goal_)          msg_data.data = GOAL;
        else if (started_)  msg_data.data = START;
        else                msg_data.data = READY;

        status_pub_->publish(msg_data);

        // courseout
        msg_couseout.data = courseout_count_;
        courseout_count_pub_->publish(msg_couseout);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr courseout_count_pub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_goal_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_courseout_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool started_ = false;
    bool goal_ = false;

    int courseout_count_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JudgeGoal>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}