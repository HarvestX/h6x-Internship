#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <std_msgs/msg/bool.hpp>

#define DEBUG 10
#define INFO 20
#define WARN 30
#define ERROR 40
#define FATAL 50

class SendLog : public rclcpp::Node
{
public:
    SendLog()
        : Node("send_log")
    {
        count_ = 0;
        courseout_init_flag = true;
        goal_flag_ = false;

        sub_courseout_ = this->create_subscription<std_msgs::msg::Bool>("/judge_courseout/data", 10, std::bind(&SendLog::line_courseout_callback, this, std::placeholders::_1));
        sub_goal_ = this->create_subscription<std_msgs::msg::Bool>("/judge_goal/data", 10, std::bind(&SendLog::line_goal_callback, this, std::placeholders::_1));
        pub_log_ = this->create_publisher<rcl_interfaces::msg::Log>("/log", 10);
    }

    void line_courseout_callback(const std_msgs::msg::Bool::SharedPtr ptr)
    {
        rcl_interfaces::msg::Log msg;

        if (ptr->data) // courseout
        {
            if (courseout_init_flag)
            {
                // courseout_init_flag = false;
                return;
            }
            count_++;
            if (count_ > 10)
            {
                msg.level = ERROR;
                msg.msg = prefix_ + "courseout!";
                pub_log_->publish(msg);
                count_ = 0;
            }
        }
        else // no courseout
        {
            if (count_ > 0)
            {
                msg.level = INFO;
                msg.msg = prefix_ + "===========================";
                pub_log_->publish(msg);
                msg.msg = prefix_ + "come back to the line";
                pub_log_->publish(msg);
                msg.msg = prefix_ + "===========================";
                pub_log_->publish(msg);
            }
            count_ = 0;

            if (courseout_init_flag)
            {
                courseout_init_flag = false;
                return;
            };
            // msg.level = INFO;
            // msg.msg = "no courseout";
        }
    }

    void line_goal_callback(const std_msgs::msg::Bool::SharedPtr ptr)
    {
        rcl_interfaces::msg::Log msg;

        if (ptr->data) // goal
        {
            if (goal_flag_)
            {
                return;
            }
            goal_flag_ = true;
            // add crawn emoji
            prefix_ = "(GOAL) ";

            msg.level = INFO;
            msg.msg = prefix_ + "goal!";
            pub_log_->publish(msg);
        }
        else // no goal
        {
            // msg.level = INFO;
            // msg.msg = prefix_ + "no goal";
            // pub->publish(msg);
            ;
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_courseout_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_goal_;
    rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr pub_log_;
    bool courseout_init_flag;
    bool goal_flag_;

    unsigned int count_;

    std::string prefix_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SendLog>());
    rclcpp::shutdown();
    return 0;
}