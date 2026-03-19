#ifndef VISION_TUNE_PROCESS_NODE_HPP_
#define VISION_TUNE_PROCESS_NODE_HPP_

#include <mutex>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "vision_tune/msg/tuning_value.hpp"
#include "vision_tune/msg/process_result.hpp"
#include "vision_tune/utils/config_utils.hpp"


enum class vision_target
{
  red = 0,
  blue = 1,
  line = 2
};
class ProcessNode : public rclcpp::Node
{
public:
  ProcessNode();
  vision_tune::utils::hsv_config hsv_config_;
  vision_target selected_target_ = vision_target::red;
  bool tuning_dirty_ = false;

private:
  void tuning_callback(const vision_tune::msg::TuningValue::SharedPtr msg);
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void process_tick();
  void declare_parameters();
  void get_parameters();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<vision_tune::msg::TuningValue>::SharedPtr tuning_sub_;

  rclcpp::Publisher<vision_tune::msg::ProcessResult>::SharedPtr result_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bird_image_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex data_mutex_;

  cv::Mat latest_raw_mat_;
  vision_tune::msg::TuningValue tuning_;

  bool raw_dirty_ = false;
  std::string result_topic;
  std::string input_topic;
  std::string result_image_topic;
  std::string tuning_topic;
  std::string bird_image_topic;
  double node_hz_ = 30.0;
};

#endif