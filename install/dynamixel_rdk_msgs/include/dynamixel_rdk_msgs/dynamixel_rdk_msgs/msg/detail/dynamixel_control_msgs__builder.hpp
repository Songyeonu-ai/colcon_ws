// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dynamixel_rdk_msgs:msg/DynamixelControlMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_CONTROL_MSGS__BUILDER_HPP_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_CONTROL_MSGS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dynamixel_rdk_msgs/msg/detail/dynamixel_control_msgs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dynamixel_rdk_msgs
{

namespace msg
{

namespace builder
{

class Init_DynamixelControlMsgs_motor_control
{
public:
  Init_DynamixelControlMsgs_motor_control()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dynamixel_rdk_msgs::msg::DynamixelControlMsgs motor_control(::dynamixel_rdk_msgs::msg::DynamixelControlMsgs::_motor_control_type arg)
  {
    msg_.motor_control = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelControlMsgs msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamixel_rdk_msgs::msg::DynamixelControlMsgs>()
{
  return dynamixel_rdk_msgs::msg::builder::Init_DynamixelControlMsgs_motor_control();
}

}  // namespace dynamixel_rdk_msgs

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_CONTROL_MSGS__BUILDER_HPP_
