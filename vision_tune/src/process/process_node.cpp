#include "vision_tune/process/process_node.hpp"
#include "vision_tune/utils/vision_utils.hpp"
#include "vision_tune/utils/common_utils.hpp"

static vision_tune::utils::hsv_range get_selected_hsv(
    const vision_tune::utils::hsv_config &cfg,
    vision_target target);

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
      vision_tune::utils::hz_to_period(node_hz_), // yaml에서 토픽/주기 설정
      std::bind(&ProcessNode::process_tick, this));
}

void ProcessNode::declare_parameters() // ui_config.yaml
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
{ // ui에서 hsv값이 변경될 때마다 업데이트
  RCLCPP_INFO(this->get_logger(), "tuning_callback called");
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
{ // 버퍼에 저장만하고 실제 처리는 process_tick
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

void ProcessNode::process_tick()
{
  RCLCPP_INFO(this->get_logger(), "process_tick called");

  cv::Mat raw_mat;
  vision_tune::utils::hsv_config cfg;
  vision_target selected_target;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if ((!raw_dirty_ && !tuning_dirty_) || latest_raw_mat_.empty())
    {
      RCLCPP_WARN(
          this->get_logger(),
          "process_tick return: raw_dirty=%d tuning_dirty=%d empty=%d",
          raw_dirty_,
          tuning_dirty_,
          latest_raw_mat_.empty());
      return;
    }

    raw_mat = latest_raw_mat_.clone();
    cfg = hsv_config_;
    selected_target = selected_target_;
    raw_dirty_ = false;
    tuning_dirty_ = false;
  }

  // ========================= bird view =========================
  std::vector<cv::Point2f> src_points = {
      {500.0f, 400.0f},
      {780.0f, 400.0f},
      {200.0f, 700.0f},
      {1080.0f, 700.0f}};

  cv::Mat bird_view = vision_tune::utils::make_bird_view(raw_mat, src_points, 640, 480);

  if (bird_view.empty())
  {
    RCLCPP_WARN(this->get_logger(), "bird_view is empty");
    return;
  }
  // ============================================================

  // ========================= selected target =========================
  vision_tune::utils::hsv_range selected_hsv =
      get_selected_hsv(cfg, selected_target);

  cv::Mat source_img = raw_mat;
  if (selected_target == vision_target::line)
  {
    source_img = bird_view;
  }

  cv::Mat mask = vision_tune::utils::make_hsv_mask(source_img, selected_hsv);

  if (mask.empty())
  {
    RCLCPP_WARN(this->get_logger(), "mask is empty");
    return;
  }

  auto detection = vision_tune::utils::find_largest_blob(mask);

  // result_image는 바이너리 마스크를 3채널로 바꾼 화면
  cv::Mat result_mat;
  cv::cvtColor(mask, result_mat, cv::COLOR_GRAY2BGR);

  vision_tune::msg::ProcessResult result_msg;
  result_msg.detected = detection.detected;
  result_msg.center_x = detection.center_x;
  result_msg.center_y = detection.center_y;
  result_msg.area = detection.area;
  // ==================================================================

  // ========================= publish =========================
  auto result_img_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result_mat).toImageMsg();

  auto bird_img_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bird_view).toImageMsg();

  result_img_msg->header.stamp = this->now();
  bird_img_msg->header.stamp = this->now();

  result_img_msg->header.frame_id = "result_frame";
  bird_img_msg->header.frame_id = "bird_frame";

  result_image_pub_->publish(*result_img_msg);
  bird_image_pub_->publish(*bird_img_msg);
  result_pub_->publish(result_msg);
  // ==========================================================
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProcessNode>());
  rclcpp::shutdown();
  return 0;
}

static vision_tune::utils::hsv_range get_selected_hsv(const vision_tune::utils::hsv_config &cfg, vision_target target)
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
