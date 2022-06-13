#include "send_log.hpp"

SendLog::SendLog() : Node("send_log")
{
    count_ = 0;
    score_ = 0;
    past_time_ = 0;
    couseout_count_ = 0;

    started_ = false;
    goal_flag_ = false;
    gameover_was_sent_ = false;

    sub_courseout_ = this->create_subscription<std_msgs::msg::Bool>("/judge_courseout/data", 10, std::bind(&SendLog::line_courseout_callback, this, std::placeholders::_1));
    sub_goal_ = this->create_subscription<std_msgs::msg::Bool>("/judge_goal/data", 10, std::bind(&SendLog::line_goal_callback, this, std::placeholders::_1));

    pub_log_ = this->create_publisher<rcl_interfaces::msg::Log>("/log", 10);
    pub_score_ = this->create_publisher<std_msgs::msg::Int32>("/score", 10);
    pub_time_ = this->create_publisher<std_msgs::msg::Int32>("/time", 10);
    pub_speed_limit_ = this->create_publisher<std_msgs::msg::Float32>("/speed_limit", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&SendLog::time_update_callback, this));
}

void SendLog::line_courseout_callback(const std_msgs::msg::Bool::SharedPtr ptr)
{
    rcl_interfaces::msg::Log _log;

    if (goal_flag_)
    {
        return;
    }

    if (ptr->data) // courseout
    {
        // not started_ : not courseout
        if (!started_)
            return;

        count_++;
        if (count_ > 10)
        {
            _log.level = ERROR;
            _log.msg = prefix_ + "courseout!";
            pub_log_->publish(_log);
            count_ = 0;
            couseout_count_++;
        }
    }
    else // no courseout
    {
        if (!started_)
        {
            started_ = true;
            _log.level = INFO;
            _log.msg = prefix_ + "===========================";
            pub_log_->publish(_log);
            _log.msg = prefix_ + "Start !!";
            pub_log_->publish(_log);
            _log.msg = prefix_ + "===========================";
            pub_log_->publish(_log);
            return;
        };

        if (count_ > 0)
        {
            _log.level = INFO;
            _log.msg = prefix_ + "===========================";
            pub_log_->publish(_log);
            _log.msg = prefix_ + "come back to the line";
            pub_log_->publish(_log);
            _log.msg = prefix_ + "===========================";
            pub_log_->publish(_log);
        }
        count_ = 0;
    }
}

void SendLog::line_goal_callback(const std_msgs::msg::Bool::SharedPtr ptr)
{
    if (goal_flag_)
    {
        return;
    }
    rcl_interfaces::msg::Log _log;

    if (ptr->data) // goal
    {
        if (goal_flag_)
        {
            return;
        }
        goal_flag_ = true;
        // add crawn emoji
        prefix_ = "(GOAL) ";

        _log.level = INFO;
        _log.msg = prefix_ + "goal!";
        pub_log_->publish(_log);
    }
    else // no goal
    {
        ;
    }
}

void SendLog::time_update_callback()
{
    started_ ? past_time_++ : past_time_ = 0;
    std_msgs::msg::Int32 msg;
    msg.data = (int)past_time_;
    pub_time_->publish(msg);

    score_update_callback();

    viecle_status_callback(score_);
}

void SendLog::score_update_callback()
{
    if (goal_flag_)
    {
        return;
    }
    std_msgs::msg::Int32 msg;
    score_ = past_time_ + couseout_count_ * 10;
    msg.data = (int)score_;
    pub_score_->publish(msg);
}

void SendLog::viecle_status_callback(const unsigned int _score)
{
    bool stop = false;
    std_msgs::msg::Float32 msg;

    if (goal_flag_)
    {
        stop = true;
    }

    if (_score > GAMEOVER_SCORE)
    {
        stop = true;
        // send error log
        if (!gameover_was_sent_)
        {
            rcl_interfaces::msg::Log _log;
            _log.level = ERROR;
            _log.msg = prefix_ + "===========================";
            pub_log_->publish(_log);
            _log.msg = prefix_ + "Game Over!";
            pub_log_->publish(_log);
            _log.msg = prefix_ + "===========================";
            pub_log_->publish(_log);
            gameover_was_sent_ = true;
        }
    }

    // publish speed limit
    if (stop)
    {
        msg.data = 0.0;
    }
    else
    {
        msg.data = SPEED_LIMIT_NORMAL;
    }

    pub_speed_limit_->publish(msg);
}

// --------------------------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SendLog>());
    rclcpp::shutdown();
    return 0;
}