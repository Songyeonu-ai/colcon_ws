#ifndef VISION_TUNE_UI_NODE_HPP_
#define VISION_TUNE_UI_NODE_HPP_

#include <QObject>
#include <QPixmap>
#include <mutex>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_tune/msg/process_result.hpp>
#include <vision_tune/msg/tuning_value.hpp>

class MainWindow;

class UiNode : public QObject, public rclcpp::Node
{
  Q_OBJECT

public:
  explicit UiNode(MainWindow *window);

Q_SIGNALS:
  void raw_image_received(const QPixmap &pixmap);
  void result_image_received(const QPixmap &pixmap);
  void bird_image_received(const QPixmap &pixmap);

private:
  void publish_tuning();
  void result_callback(const vision_tune::msg::ProcessResult::SharedPtr msg);
  void raw_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void result_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void bird_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void declare_parameters();
  void get_parameters();
  void ui_tick();

  MainWindow *window_;

  rclcpp::Publisher<vision_tune::msg::TuningValue>::SharedPtr tuning_pub_;

  rclcpp::Subscription<vision_tune::msg::ProcessResult>::SharedPtr result_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr result_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr bird_image_sub_;

  rclcpp::TimerBase::SharedPtr ui_timer_;

  std::mutex data_mutex_;

  cv::Mat latest_raw_mat_;
  cv::Mat latest_result_mat_;
  cv::Mat latest_bird_mat_;
  vision_tune::msg::ProcessResult latest_result_msg_;

  bool raw_dirty_ = false;
  bool result_image_dirty_ = false;
  bool result_msg_dirty_ = false;
  bool bird_image_dirty_ = false;

  std::string result_topic_;
  std::string raw_image_topic_;
  std::string result_image_topic_;
  std::string tuning_topic_;
  std::string bird_image_topic_;
  double node_hz_ = 15.0;
  double ui_apply_hz_ = 15.0;
};

#endif