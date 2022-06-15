#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/msg/bool.hpp>

class JudgeCourseout : public rclcpp::Node
{
public:
    JudgeCourseout()
        : Node("judge_courseout")
    {
        rmw_qos_profile_t qos_sensor = rmw_qos_profile_sensor_data;
        this->sub_image_ = image_transport::create_subscription(
            this, "/camera_linetrace/camera1/image_raw", std::bind(&JudgeCourseout::image_callback, this, std::placeholders::_1), "raw", qos_sensor);

        data_pub_ = this->create_publisher<std_msgs::msg::Bool>("/judge_courseout/data", 10);
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& ptr)
    {
        cv_bridge::CvImagePtr cv_ptr;
        std_msgs::msg::Bool msg_data;
        cv::Mat frame;

        try {
            auto img = cv_bridge::toCvCopy(ptr, "bgr8");
            frame = img->image;
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // process image
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        // 0 ~ 100 : ok
        // width = 100, height = 1
        unsigned short int black_count = 0;
        for (int i = 0; i < gray.cols; i++)
        {
            if (gray.at<uchar>(0, i) < 100)
            {
                black_count++;
            }
        }

        if (black_count < 5)
        {
            msg_data.data = true;
            data_pub_->publish(msg_data);
        }
        else
        {
            data_pub_->publish(msg_data);
        }
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr data_pub_;
    image_transport::Subscriber sub_image_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JudgeCourseout>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}