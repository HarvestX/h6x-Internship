#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <std_msgs/msg/bool.hpp>

#define TOUCH_THRESHOLD 1.0

class JudgeGoal : public rclcpp::Node
{
public:
    JudgeGoal()
        : Node("judge_goal")
    {
        // rmw_qos_profile_t qos_sensor = rmw_qos_profile_sensor_data;
        this->sub_range_ = this->create_subscription<sensor_msgs::msg::Range>("/raypoint/range", rclcpp::SensorDataQoS(), std::bind(&JudgeGoal::range_callback, this, std::placeholders::_1));

        data_pub_ = this->create_publisher<std_msgs::msg::Bool>("/judge_goal/data", 10);
    }

    void range_callback(const sensor_msgs::msg::Range::SharedPtr ptr)
    {
        float range = ptr->range;
        std_msgs::msg::Bool msg_data;

        if (range < TOUCH_THRESHOLD)
        {
            RCLCPP_INFO(this->get_logger(), "GOAL!!!");
            msg_data.data = true;
        }
        else
        {
            msg_data.data = false;
        }

        data_pub_->publish(msg_data);
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr data_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_range_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JudgeGoal>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}