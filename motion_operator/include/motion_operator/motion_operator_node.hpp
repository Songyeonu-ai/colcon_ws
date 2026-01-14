#ifndef MOTION_OPERATOR_NODE_HPP
#define MOTION_OPERATOR_NODE_HPP

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <vector>

// #include "dynamixel_rdk_msgs/msg/dynamixel_bulk_read_msgs.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_control_msgs.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_msgs.hpp"
#include "dynamixel_rdk_msgs/msg/dynamixel_status_msgs.hpp"
#include "humanoid_interfaces/msg/motion_operator.hpp"
#include "motion_operator/motion_operator.hpp"

namespace motion_operator {

class MOTION_NODE : public rclcpp_lifecycle::LifecycleNode {
 public:  // Functions
  MOTION_NODE(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~MOTION_NODE();

 private:  // Functions
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &new_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state);

  void set_parameters();
  void get_parameters();
  void set_init_position(std::string motion_path,
                         std::vector<Dynamixel_postition> dxl_status);

  void motion_operator_callback(
      const humanoid_interfaces::msg::MotionOperator::SharedPtr msg);
  void dxl_status_callback(
      const dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs::SharedPtr msg);

 public:  // Variables
  std::string InitAddr_;
  std::string KickRAddr_;
  std::string KickLAddr_;
  std::string FrontRiseAddr_;
  std::string BackRiseAddr_;
  std::string RightRiseAddr_;
  std::string LeftRiseAddr_;

  std::vector<Dynamixel_postition> dxl_status_;
  std::vector<double> current_position_;

  std::vector<double> goal_position_;
  std::vector<double> goal_velocity_;

 private:  // Variables
  dynamixel_rdk_msgs::msg::DynamixelControlMsgs dxl_msg_;
  humanoid_interfaces::msg::MotionOperator motion_operator_msg_;

  rclcpp::Subscription<humanoid_interfaces::msg::MotionOperator>::SharedPtr
      motion_operator_sub_;
  rclcpp::Subscription<dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs>::
      SharedPtr dxl_status_sub_;

  rclcpp_lifecycle::LifecyclePublisher<
      dynamixel_rdk_msgs::msg::DynamixelControlMsgs>::SharedPtr dxl_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      humanoid_interfaces::msg::MotionOperator>::SharedPtr motion_operator_pub_;

  std::shared_ptr<MOTION> motion_operator_;
};

}  // namespace motion_operator

#endif  // MOTION_OPERATOR_NODE_HPP
