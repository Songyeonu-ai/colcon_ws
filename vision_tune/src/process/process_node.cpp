#include "vision_tune/process/process_node.hpp"

ProcessNode::ProcessNode() : Node("process_node")
{
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/camera/image_raw",
        10,
        std::bind(&ProcessNode::imageCallback, this, std::placeholders::_1));

    tuning_sub_ = this->create_subscription<vision_tune::msg::TuningValue>(
        "/vision/tuning", 10,
        std::bind(&ProcessNode::tuningCallback, this, std::placeholders::_1));

    result_pub_ = this->create_publisher<vision_tune::msg::ProcessResult>("/vision/result", 10);
    result_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/result_image", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&ProcessNode::publishResult, this)); // 약 30Hz
}

void ProcessNode::tuningCallback(const vision_tune::msg::TuningValue::SharedPtr msg)
{
    tuning_ = *msg;
}

void ProcessNode::publishResult()
{
}

void ProcessNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    (void)msg;

    // TODO:
    // raw image + tuning_ 값으로 계산

    vision_tune::msg::ProcessResult result;
    result.detected = true;
    result.center_x = 100.0f;
    result.center_y = 120.0f;
    result.area = 500.0f;
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    auto img_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

    img_msg->header.stamp = this->now();
    img_msg->header.frame_id = "camera_frame";

    result_image_pub_->publish(*img_msg);

    result_pub_->publish(result);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProcessNode>());
    rclcpp::shutdown();
    return 0;
}