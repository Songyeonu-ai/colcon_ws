#include "vision_tune/ui/ui_node.hpp"
#include "vision_tune/ui/main_window.hpp"
#include "vision_tune/utils/common_utils.hpp"

#include <QApplication>
#include <QTimer>
#include <QImage>
#include <QPixmap>

#include <cv_bridge/cv_bridge.h>

UiNode::UiNode(MainWindow *window)
    : QObject(),
      rclcpp::Node("ui_node"),
      window_(window)
{
  declare_parameters();
  get_parameters();
  window_->set_ui_update_hz(ui_apply_hz_);

  tuning_pub_ = this->create_publisher<vision_tune::msg::TuningValue>(
      tuning_topic_, 1);

  result_sub_ = this->create_subscription<vision_tune::msg::ProcessResult>(
      result_topic_,
      1,
      std::bind(&UiNode::result_callback, this, std::placeholders::_1));

  raw_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      raw_image_topic_,
      1,
      std::bind(&UiNode::raw_image_callback, this, std::placeholders::_1));

  result_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      result_image_topic_,
      1,
      std::bind(&UiNode::result_image_callback, this, std::placeholders::_1));

  bird_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      bird_image_topic_,
      1,
      std::bind(&UiNode::bird_image_callback, this, std::placeholders::_1));

  ui_timer_ = this->create_wall_timer(
      vision_tune::utils::hz_to_period(node_hz_),
      std::bind(&UiNode::ui_tick, this)); // yaml에서 토픽/주기 설정
}

void UiNode::declare_parameters() // ui_config.yaml
{
  declare_parameter<std::string>("topic.result_topic", "/vision/result");
  declare_parameter<std::string>("topic.raw_image_topic", "/camera1/camera/compressed_image");
  declare_parameter<std::string>("topic.result_image_topic", "/vision/result_image");
  declare_parameter<std::string>("topic.tuning_topic", "/vision/tuning");
  declare_parameter<std::string>("topic.bird_image_topic", "/vision/bird_image");

  declare_parameter<double>("node_hz", 15.0);
  declare_parameter<double>("ui_apply_hz", 15.0);
}

void UiNode::get_parameters()
{
  get_parameter("topic.result_topic", result_topic_);
  get_parameter("topic.raw_image_topic", raw_image_topic_);
  get_parameter("topic.result_image_topic", result_image_topic_);
  get_parameter("topic.tuning_topic", tuning_topic_);
  get_parameter("topic.bird_image_topic", bird_image_topic_);
  get_parameter("node_hz", node_hz_);
  get_parameter("ui_apply_hz", ui_apply_hz_);
}

void UiNode::publish_tuning()
{
  const vision_tune::utils::hsv_config &cfg = window_->get_hsv_config();

  vision_tune::msg::TuningValue msg;

  msg.selected_target = static_cast<int>(window_->get_current_target());

  msg.red_h_low = cfg.red.h_low;
  msg.red_h_high = cfg.red.h_high;
  msg.red_s_low = cfg.red.s_low;
  msg.red_s_high = cfg.red.s_high;
  msg.red_v_low = cfg.red.v_low;
  msg.red_v_high = cfg.red.v_high;

  msg.blue_h_low = cfg.blue.h_low;
  msg.blue_h_high = cfg.blue.h_high;
  msg.blue_s_low = cfg.blue.s_low;
  msg.blue_s_high = cfg.blue.s_high;
  msg.blue_v_low = cfg.blue.v_low;
  msg.blue_v_high = cfg.blue.v_high;

  msg.line_h_low = cfg.line.h_low;
  msg.line_h_high = cfg.line.h_high;
  msg.line_s_low = cfg.line.s_low;
  msg.line_s_high = cfg.line.s_high;
  msg.line_v_low = cfg.line.v_low;
  msg.line_v_high = cfg.line.v_high;

  tuning_pub_->publish(msg);
}

void UiNode::result_callback(const vision_tune::msg::ProcessResult::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_result_msg_ = *msg;
  result_msg_dirty_ = true;
}

void UiNode::raw_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{ // 버퍼에 저장만하고 실제 처리는 ui_tick
  try
  {
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_raw_mat_ = frame.clone();
    raw_dirty_ = true;
  }
  catch (const cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "raw image cv_bridge exception: %s", e.what());
  }
}

void UiNode::result_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{ // 버퍼에 저장만하고 실제 처리는 ui_tick
  try
  {
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_result_mat_ = frame.clone();
    result_image_dirty_ = true;
  }
  catch (const cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "result image cv_bridge exception: %s", e.what());
  }
}

void UiNode::bird_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{ // 버퍼에 저장만하고 실제 처리는 ui_tick
  try
  {
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_bird_mat_ = frame.clone();
    bird_image_dirty_ = true;
  }
  catch (const cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "bird image cv_bridge exception: %s", e.what());
  }
}

void UiNode::ui_tick()
{
  cv::Mat raw_mat;
  cv::Mat result_mat;
  cv::Mat bird_mat;
  vision_tune::msg::ProcessResult result_msg;

  bool raw_dirty = false;
  bool result_image_dirty = false;
  bool result_msg_dirty = false;
  bool bird_image_dirty = false;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (raw_dirty_)
    {
      raw_mat = latest_raw_mat_.clone();
      raw_dirty = true;
      raw_dirty_ = false;
    }

    if (result_image_dirty_)
    {
      result_mat = latest_result_mat_.clone();
      result_image_dirty = true;
      result_image_dirty_ = false;
    }

    if (result_msg_dirty_)
    {
      result_msg = latest_result_msg_;
      result_msg_dirty = true;
      result_msg_dirty_ = false;
    }

    if (bird_image_dirty_)
    {
      bird_mat = latest_bird_mat_.clone();
      bird_image_dirty = true;
      bird_image_dirty_ = false;
    }
  } // 업데이트 확인

  //--------------------UI 업데이트는 mainwindow에서, 여기는 emit으로 신호-------------------
  if (raw_dirty && !raw_mat.empty())
  {
    QImage qimg(
        raw_mat.data,
        raw_mat.cols,
        raw_mat.rows,
        static_cast<int>(raw_mat.step),
        QImage::Format_BGR888);

    emit raw_image_received(QPixmap::fromImage(qimg.copy()));
  }

  if (result_image_dirty && !result_mat.empty())
  {
    QImage qimg(
        result_mat.data,
        result_mat.cols,
        result_mat.rows,
        static_cast<int>(result_mat.step),
        QImage::Format_BGR888);

    emit result_image_received(QPixmap::fromImage(qimg.copy()));
  }

  if (bird_image_dirty && !bird_mat.empty())
  {
    QImage qimg(
        bird_mat.data,
        bird_mat.cols,
        bird_mat.rows,
        static_cast<int>(bird_mat.step),
        QImage::Format_BGR888);

    emit bird_image_received(QPixmap::fromImage(qimg.copy()));
  }
  //---------------------------------------------------------------------

  if (result_msg_dirty)
  {
    window_->update_result(result_msg);
  }

  publish_tuning();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  MainWindow window;
  window.show();

  auto node = std::make_shared<UiNode>(&window);

  QObject::connect(
      node.get(),
      &UiNode::raw_image_received,
      &window,
      &MainWindow::update_raw_image);

  QObject::connect(
      node.get(),
      &UiNode::result_image_received,
      &window,
      &MainWindow::update_result_image);

  QObject::connect(
      node.get(),
      &UiNode::bird_image_received,
      &window,
      &MainWindow::update_bird_image);

  QTimer ros_spin_timer;
  QObject::connect(&ros_spin_timer, &QTimer::timeout, [&]()
                   {
        if (rclcpp::ok()) {
            rclcpp::spin_some(node);
        } });
  ros_spin_timer.start(10);

  QObject::connect(&app, &QApplication::aboutToQuit, [&]()
                   { ros_spin_timer.stop(); });

  int ret = app.exec();

  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }

  return ret;
}