#include "vision_tune/process/process_node.hpp"

static hsv_range get_selected_hsv(const hsv_config &cfg, vision_target target)
{
  switch (target)
  {
  case vision_target::red:
    return cfg.red;
  case vision_target::blue:
    return cfg.blue;
  case vision_target::line:
    return cfg.line;
  }
  return cfg.red;
}

ProcessNode::ProcessNode()
    : Node("process_node")
{
  declare_parameters();
  get_parameters();

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      input_topic,
      1,
      std::bind(&ProcessNode::image_callback, this, std::placeholders::_1));

  tuning_sub_ = this->create_subscription<vision_tune::msg::TuningValue>(
      tuning_topic,
      1,
      std::bind(&ProcessNode::tuning_callback, this, std::placeholders::_1));

  result_pub_ = this->create_publisher<vision_tune::msg::ProcessResult>(
      result_topic, 1);

  result_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      result_image_topic, 1);

  bird_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      bird_image_topic, 1);

  timer_ = this->create_wall_timer(
      cal_period(node_hz_),
      std::bind(&ProcessNode::process_tick, this));
}

std::chrono::nanoseconds ProcessNode::cal_period(double hz) // hz계산
{
  if (hz <= 0.0)
  {
    return std::chrono::milliseconds(100);
  }

  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz));
}

void ProcessNode::declare_parameters()
{
  declare_parameter<std::string>("topic.input_topic", "/camera1/camera/image_raw");
  declare_parameter<std::string>("topic.tuning_topic", "/vision/tuning");
  declare_parameter<std::string>("topic.result_topic", "/vision/result");
  declare_parameter<std::string>("topic.result_image_topic", "/vision/result_image");
  declare_parameter<std::string>("topic.bird_image_topic", "/vision/bird_image");
  declare_parameter<double>("node_hz", 30.0);
}

void ProcessNode::get_parameters()
{
  get_parameter("topic.input_topic", input_topic);
  get_parameter("topic.tuning_topic", tuning_topic);
  get_parameter("topic.result_topic", result_topic);
  get_parameter("topic.result_image_topic", result_image_topic);
  get_parameter("topic.bird_image_topic", bird_image_topic);
  get_parameter("node_hz", node_hz_);
}

void ProcessNode::tuning_callback(const vision_tune::msg::TuningValue::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  hsv_config_.red.h_low = msg->red_h_low;
  hsv_config_.red.h_high = msg->red_h_high;
  hsv_config_.red.s_low = msg->red_s_low;
  hsv_config_.red.s_high = msg->red_s_high;
  hsv_config_.red.v_low = msg->red_v_low;
  hsv_config_.red.v_high = msg->red_v_high;

  hsv_config_.blue.h_low = msg->blue_h_low;
  hsv_config_.blue.h_high = msg->blue_h_high;
  hsv_config_.blue.s_low = msg->blue_s_low;
  hsv_config_.blue.s_high = msg->blue_s_high;
  hsv_config_.blue.v_low = msg->blue_v_low;
  hsv_config_.blue.v_high = msg->blue_v_high;

  hsv_config_.line.h_low = msg->line_h_low;
  hsv_config_.line.h_high = msg->line_h_high;
  hsv_config_.line.s_low = msg->line_s_low;
  hsv_config_.line.s_high = msg->line_s_high;
  hsv_config_.line.v_low = msg->line_v_low;
  hsv_config_.line.v_high = msg->line_v_high;

  selected_target_ = static_cast<vision_target>(msg->selected_target);
  tuning_dirty_ = true;
}

void ProcessNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "image_callback called");
  try
  {
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_raw_mat_ = frame.clone();
    raw_dirty_ = true;
  }
  catch (const cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

cv::Mat ProcessNode::make_bird_view(const cv::Mat &frame)
{
  if (frame.empty())
  {
    return cv::Mat();
  }

  // 원본 이미지에서 변환할 4개 점
  std::vector<cv::Point2f> src_points;
  src_points.emplace_back(500.0f, 400.0f);
  src_points.emplace_back(780.0f, 400.0f);
  src_points.emplace_back(200.0f, 700.0f);
  src_points.emplace_back(1080.0f, 700.0f);

  // bird view 결과 이미지에서 대응될 4개 점
  int output_width = 640;
  int output_height = 480;

  std::vector<cv::Point2f> dst_points;
  dst_points.emplace_back(0.0f, 0.0f);
  dst_points.emplace_back(static_cast<float>(output_width - 1), 0.0f);
  dst_points.emplace_back(0.0f, static_cast<float>(output_height - 1));
  dst_points.emplace_back(static_cast<float>(output_width - 1), static_cast<float>(output_height - 1));

  cv::Mat transform_matrix = cv::getPerspectiveTransform(src_points, dst_points);

  cv::Mat bird_view;
  cv::warpPerspective(
      frame,
      bird_view,
      transform_matrix,
      cv::Size(output_width, output_height));

  return bird_view;
}

void ProcessNode::process_tick()
{
  RCLCPP_INFO(this->get_logger(), "process_tick called");
  cv::Mat raw_mat;
  hsv_config cfg;
  vision_target selected_target;

  bool raw_ready = false;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if ((!raw_dirty_ && !tuning_dirty_) || latest_raw_mat_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "process_tick return: raw_dirty=%d tuning_dirty=%d empty=%d",
            raw_dirty_, tuning_dirty_, latest_raw_mat_.empty());
      return;
    }
    raw_mat = latest_raw_mat_.clone();
    cfg = hsv_config_;
    selected_target = selected_target_;
    raw_dirty_ = false;
    tuning_dirty_ = false;
    raw_ready = true;
  }

  if (!raw_ready)
  {
    RCLCPP_WARN(this->get_logger(), "process_tick return: raw_dirty=%d tuning_dirty=%d empty=%d",
            raw_dirty_, tuning_dirty_, latest_raw_mat_.empty());
    return;
  }

  // =========================
  // bird view
  // =========================
  cv::Mat bird_view = make_bird_view(raw_mat);
  if (bird_view.empty())
  {
    RCLCPP_WARN(this->get_logger(), "process_tick return: raw_dirty=%d tuning_dirty=%d empty=%d",
            raw_dirty_, tuning_dirty_, latest_raw_mat_.empty());
    return;
  }

  cv::Mat bird_img;
  cv::cvtColor(bird_view, bird_img, cv::COLOR_BGR2HSV);

  // =========================
  // 현재 선택된 색만 디버그
  // =========================
  hsv_range selected_hsv = get_selected_hsv(cfg, selected_target);

  cv::Mat hsv_img;
  cv::Mat before_img = raw_mat;
  if (selected_target == vision_target::line)
  {
    before_img = bird_view;
  }
  cv::cvtColor(before_img, hsv_img, cv::COLOR_BGR2HSV);

  cv::Scalar lower(
      selected_hsv.h_low,
      selected_hsv.s_low,
      selected_hsv.v_low);

  cv::Scalar upper(
      selected_hsv.h_high,
      selected_hsv.s_high,
      selected_hsv.v_high);

  cv::Mat mask; // 마스크
  cv::inRange(hsv_img, lower, upper, mask);

  cv::Mat result_mat; // 결과 이미지
  cv::cvtColor(mask, result_mat, cv::COLOR_GRAY2BGR);

  vision_tune::msg::ProcessResult result_msg;
  result_msg.detected = cv::countNonZero(mask) > 0;
  result_msg.center_x = 0.0f;
  result_msg.center_y = 0.0f;
  result_msg.area = static_cast<float>(cv::countNonZero(mask));

  auto img_msg = cv_bridge::CvImage(
                     std_msgs::msg::Header(), "bgr8", result_mat)
                     .toImageMsg();

  auto bird_msg = cv_bridge::CvImage(
                      std_msgs::msg::Header(), "bgr8", bird_view)
                      .toImageMsg();

  img_msg->header.stamp = this->now();
  bird_msg->header.stamp = this->now();
  img_msg->header.frame_id = "camera_frame";
  bird_msg->header.frame_id = "bird_frame";

  result_image_pub_->publish(*img_msg);
  bird_image_pub_->publish(*bird_msg);
  result_pub_->publish(result_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProcessNode>());
  rclcpp::shutdown();
  return 0;
}