/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date August 2024
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include "std_msgs/msg/string.hpp"
#include "../include/ebimu/qnode.hpp"

extern rclcpp::Publisher<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_publisher_;

namespace e2box_imu {

extern double roll, pitch, yaw;

QNode::QNode()
{
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  qnode = rclcpp::Node::make_shared("ebimu");
  node = std::make_shared<e2box_imu::E2BoxIMUNode>();

  Imu_Sub = node->create_subscription<humanoid_interfaces::msg::ImuMsg>(
    "bridge", 
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().best_effort(),
    std::bind(&QNode::imu_callback, this, std::placeholders::_1));
  imu_publisher_ = node->create_publisher<humanoid_interfaces::msg::ImuMsg>(
    "Imu", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().best_effort());

  imu_flag_sub_ = node->create_subscription<web_control_bridge::msg::ImuflagMsg>(
    "/imuflag", 10,
    std::bind(&QNode::imu_flag_callback, this, std::placeholders::_1));

  this->start();
}

QNode::~QNode()
{
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}
void QNode::imu_callback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg)
{
  pitch = msg->pitch;
  roll = msg->roll;
  yaw = msg->yaw;

  Q_EMIT dial_update();
}


void QNode::clearFlag()
{
  imu_flag_ = -1;
}

void QNode::imu_flag_callback(const web_control_bridge::msg::ImuflagMsg::SharedPtr msg)
{
  imu_flag_ = msg->set;
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(33);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(qnode);
  std::thread executor_thread([&]() { executor.spin(); });
  while (rclcpp::ok())
  {
    //rclcpp::spin(node);
    //std::cout<<imu_data_.yaw<<std::endl<<std::endl;
    loop_rate.sleep();
  }
  executor.cancel();
  executor_thread.join(); 
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

}
