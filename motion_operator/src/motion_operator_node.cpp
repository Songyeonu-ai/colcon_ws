#include "../include/motion_operator/motion_operator_node.hpp"

namespace motion_operator {

MOTION_NODE::MOTION_NODE(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("yolo_libtorch_node", options) {
  set_parameters();
}

MOTION_NODE::~MOTION_NODE() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MOTION_NODE::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "configuring");
  get_parameters();

  motion_operator_sub_ =
      create_subscription<humanoid_interfaces::msg::MotionOperator>(
          "motion_operator", 10,
          std::bind(&MOTION_NODE::motion_operator_callback, this,
                    std::placeholders::_1));

  dxl_status_sub_ =
      create_subscription<dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs>(
          "/dynamixel_status", 10,
          std::bind(&MOTION_NODE::dxl_status_callback, this,
                    std::placeholders::_1));

  dxl_pub_ = create_publisher<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>(
      "/dynamixel_control", 10);

  motion_operator_pub_ =
      create_publisher<humanoid_interfaces::msg::MotionOperator>("motion_end",
                                                                 10);

  motion_operator_ =
      std::make_shared<MOTION>(KickRAddr_, KickLAddr_, FrontRiseAddr_,
                               BackRiseAddr_, RightRiseAddr_, LeftRiseAddr_);

  RCLCPP_INFO(get_logger(), "set init position");
  set_init_position(InitAddr_, dxl_status_);
  RCLCPP_INFO(get_logger(), "set init position complete");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MOTION_NODE::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Activating node");

  dxl_pub_->on_activate();
  motion_operator_pub_->on_activate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MOTION_NODE::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Deactivating node");

  dxl_pub_->on_deactivate();
  motion_operator_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MOTION_NODE::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Cleaning up node");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MOTION_NODE::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down node");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

void MOTION_NODE::set_parameters() {
  declare_parameter("InitAddr", rclcpp::ParameterValue("../motion/init.yaml"));
  declare_parameter("KickRAddr",
                    rclcpp::ParameterValue("../motion/KickR.yaml"));
  declare_parameter("KickLAddr",
                    rclcpp::ParameterValue("../motion/KickL.yaml"));
  declare_parameter("FrontRiseAddr",
                    rclcpp::ParameterValue("../motion/FrontRise.yaml"));
  declare_parameter("BackRiseAddr",
                    rclcpp::ParameterValue("../motion/BackRise.yaml"));
  declare_parameter("RightRiseAddr",
                    rclcpp::ParameterValue("../motion/RightRise.yaml"));
  declare_parameter("LeftRiseAddr",
                    rclcpp::ParameterValue("../motion/LeftRise.yaml"));
}

void MOTION_NODE::get_parameters() {
  InitAddr_ = get_parameter("InitAddr").as_string();
  KickRAddr_ = get_parameter("KickRAddr").as_string();
  KickLAddr_ = get_parameter("KickLAddr").as_string();
  FrontRiseAddr_ = get_parameter("FrontRiseAddr").as_string();
  BackRiseAddr_ = get_parameter("BackRiseAddr").as_string();
  RightRiseAddr_ = get_parameter("RightRiseAddr").as_string();
  LeftRiseAddr_ = get_parameter("LeftRiseAddr").as_string();

  std::cout << "InitAddr: " << InitAddr_ << std::endl;
  std::cout << "KickRAddr: " << KickRAddr_ << std::endl;
  std::cout << "KickLAddr: " << KickLAddr_ << std::endl;
  std::cout << "FrontRiseAddr: " << FrontRiseAddr_ << std::endl;
  std::cout << "BackRiseAddr: " << BackRiseAddr_ << std::endl;
  std::cout << "RightRiseAddr: " << RightRiseAddr_ << std::endl;
  std::cout << "LeftRiseAddr: " << LeftRiseAddr_ << std::endl;
}

void MOTION_NODE::set_init_position(
    std::string motion_path, std::vector<Dynamixel_postition> dxl_status) {
  motion_operator_->read_step(motion_path);
}

void MOTION_NODE::motion_operator_callback(
    const humanoid_interfaces::msg::MotionOperator::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "Received motion operator message");

  int motion_num = msg->motion_num;

  std::cout << "motion_num: " << motion_num << std::endl;

  std::string motion_path;

  if (motion_num == 0) {
    motion_path = KickRAddr_;
    std::cout << "KickR" << std::endl;
  } else if (motion_num == 1) {
    motion_path = KickLAddr_;
    std::cout << "KickL" << std::endl;
  } else if (motion_num == 2) {
    motion_path = FrontRiseAddr_;
    std::cout << "FrontRise" << std::endl;
  } else if (motion_num == 3) {
    motion_path = BackRiseAddr_;
    std::cout << "BackRise" << std::endl;
  } else if (motion_num == 4) {
    motion_path = RightRiseAddr_;
    std::cout << "RightRise" << std::endl;
  } else if (motion_num == 5) {
    motion_path = LeftRiseAddr_;
    std::cout << "LeftRise" << std::endl;
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid motion number: %d", motion_num);
    return;
  }

  dynamixel_rdk_msgs::msg::DynamixelControlMsgs dxl_ctrl;

  if (motion_operator_->read_motion(motion_path)) {
    RCLCPP_INFO(get_logger(), "Read motion file");
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to read motion file");
    return;
  }

  for (int i = 0; i < motion_operator_->steps.size(); i++) {
    current_position_ = motion_operator_->get_before_position(i);
    goal_position_ = motion_operator_->get_goal_position(i);
    if (i == 0) {
      goal_velocity_ = std::vector<double>(goal_position_.size(), 1.71);
    } else {
      goal_velocity_ = motion_operator_->set_velocity(
          goal_position_, current_position_, motion_operator_->steps[i].time);
    }

    dynamixel_rdk_msgs::msg::DynamixelMsgs dxl;
    for (int j = 0; j < goal_position_.size(); j++) {
      dxl.goal_position = goal_position_[j];
      dxl.profile_velocity = goal_velocity_[j];
      dxl.profile_acceleration = 0;
      dxl_ctrl.motor_control.push_back(dxl);
    }
    dxl_pub_->publish(dxl_ctrl);
    std::cout << "step: " << i << std::endl;
    std::cout << "time: " << motion_operator_->steps[i].time << std::endl;
    if (i == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    } else {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(motion_operator_->steps[i].time));
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::cout << "delay: " << motion_operator_->steps[i].delay << std::endl;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(motion_operator_->steps[i].delay));
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    dxl_ctrl.motor_control.clear();
  }

  int endflag = 1;
  humanoid_interfaces::msg::MotionOperator motion_end;
  motion_end.motion_num = motion_num;
  motion_end.motion_end = endflag;
  motion_operator_pub_->publish(motion_end);
}

void MOTION_NODE::dxl_status_callback(
    const dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs::SharedPtr msg) {
  // dxl_status_.clear();
  // current_position_.clear();

  // for (int i = 0; i < msg->status_msgs.size(); i++) {
  //   Dynamixel_postition dxl;

  //   dxl.id = msg->status_msgs[i].id;
  //   dxl.position = msg->status_msgs[i].present_position;
  //   dxl_status_.push_back(dxl);
  // }
}

}  // namespace motion_operator

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motion_operator::MOTION_NODE>();

  auto configure_result = node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (configure_result.id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure PanTiltCamera.");
    return 1;
  }

  auto activate_result = node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (activate_result.id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "Failed to activate PanTiltCamera.");
    return 1;
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
