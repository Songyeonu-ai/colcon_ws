#include <memory>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_tune/msg/tuning_value.hpp"
#include "vision_tune/msg/process_result.hpp"

class ProcessNode : public rclcpp::Node
{
public:
    ProcessNode();

private:
    void tuningCallback(const vision_tune::msg::TuningValue::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void publishResult();

    vision_tune::msg::TuningValue tuning_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<vision_tune::msg::TuningValue>::SharedPtr tuning_sub_;
    rclcpp::Publisher<vision_tune::msg::ProcessResult>::SharedPtr result_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};