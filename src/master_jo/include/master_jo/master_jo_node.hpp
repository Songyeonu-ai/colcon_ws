#ifndef MASTER_JO_NODE_HPP
#define MASTER_JO_NODE_HPP

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>

#include "forward.hpp"
#include "master_jo.hpp"

namespace master_jo {

class MasterJoNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  MasterJoNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state);

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter> &parameters);

  std::shared_ptr<master_jo::MasterRcko> master;

 private:
  void init_parameters();
  void get_parameters();

  void imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg);
  void visionCallback(
      const humanoid_interfaces::msg::Robocupvision25::SharedPtr msg);
  void ikCallback(const humanoid_interfaces::msg::IkEndMsg::SharedPtr msg);
  void gamecontrolCallback(
      const humanoid_interfaces::msg::Gamecontroldata::SharedPtr msg);
  void localCallback(
      const humanoid_interfaces::msg::Robocuplocalization25::SharedPtr msg);
  void udpCallback(const humanoid_interfaces::msg::Udp2master::SharedPtr msg);
  void motionCallback(
      const humanoid_interfaces::msg::MotionOperator::SharedPtr msg);
  void pidCallback(const humanoid_interfaces::msg::Pidtuning::SharedPtr msg);

  void robocup_master();

 private:
  std::shared_ptr<master_jo::Player> player;
  rclcpp::TimerBase::SharedPtr timer;


  rclcpp_lifecycle::LifecyclePublisher<humanoid_interfaces::msg::Master2IkMsg>::SharedPtr ik_pub;
  rclcpp_lifecycle::LifecyclePublisher<humanoid_interfaces::msg::Master2vision25>::SharedPtr vision_pub;
  rclcpp_lifecycle::LifecyclePublisher<humanoid_interfaces::msg::Gamecontrolreturndata>::SharedPtr gamecontrol_pub;
  rclcpp_lifecycle::LifecyclePublisher<humanoid_interfaces::msg::Master2localization25>::SharedPtr local_pub;
  rclcpp_lifecycle::LifecyclePublisher<humanoid_interfaces::msg::Master2udp>::SharedPtr udp_pub;
  rclcpp_lifecycle::LifecyclePublisher<humanoid_interfaces::msg::MotionOperator>::SharedPtr motionPub;

  rclcpp::Subscription<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_sub;
  rclcpp::Subscription<humanoid_interfaces::msg::Robocupvision25>::SharedPtr vision_sub;
  rclcpp::Subscription<humanoid_interfaces::msg::IkEndMsg>::SharedPtr ik_sub;
  rclcpp::Subscription<humanoid_interfaces::msg::Gamecontroldata>::SharedPtr gamecontrol_sub;
  rclcpp::Subscription<humanoid_interfaces::msg::Robocuplocalization25>::SharedPtr local_sub;
  rclcpp::Subscription<humanoid_interfaces::msg::Udp2master>::SharedPtr udp_sub;
  rclcpp::Subscription<humanoid_interfaces::msg::Pidtuning>::SharedPtr pid_sub;

  double kp_;
  double kd_;
  double ki_;

  int FRONT_MAX_;
  int REAR_MAX_;
  int RIGHT_MAX_;
  int LEFT_MAX_;
  int R_YAW_MAX_;
  int L_YAW_MAX_;
  int X_MIN_;
  int Y_MIN_;
  int ROUND_Y_;
  int ROUND_YAW_MIN_;

  int In_;
  int Out_;
  int Back_;
  int Front_;

  bool isPenalty;
  bool testFlag = false;

  int position = POSITION_FW;
  int state = STATE_PLAYING;
};

}

#endif