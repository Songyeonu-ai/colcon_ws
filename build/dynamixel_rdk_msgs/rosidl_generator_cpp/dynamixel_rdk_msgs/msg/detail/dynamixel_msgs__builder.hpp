// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dynamixel_rdk_msgs:msg/DynamixelMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_MSGS__BUILDER_HPP_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_MSGS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dynamixel_rdk_msgs/msg/detail/dynamixel_msgs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dynamixel_rdk_msgs
{

namespace msg
{

namespace builder
{

class Init_DynamixelMsgs_profile_velocity
{
public:
  explicit Init_DynamixelMsgs_profile_velocity(::dynamixel_rdk_msgs::msg::DynamixelMsgs & msg)
  : msg_(msg)
  {}
  ::dynamixel_rdk_msgs::msg::DynamixelMsgs profile_velocity(::dynamixel_rdk_msgs::msg::DynamixelMsgs::_profile_velocity_type arg)
  {
    msg_.profile_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelMsgs msg_;
};

class Init_DynamixelMsgs_profile_acceleration
{
public:
  explicit Init_DynamixelMsgs_profile_acceleration(::dynamixel_rdk_msgs::msg::DynamixelMsgs & msg)
  : msg_(msg)
  {}
  Init_DynamixelMsgs_profile_velocity profile_acceleration(::dynamixel_rdk_msgs::msg::DynamixelMsgs::_profile_acceleration_type arg)
  {
    msg_.profile_acceleration = std::move(arg);
    return Init_DynamixelMsgs_profile_velocity(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelMsgs msg_;
};

class Init_DynamixelMsgs_goal_position
{
public:
  explicit Init_DynamixelMsgs_goal_position(::dynamixel_rdk_msgs::msg::DynamixelMsgs & msg)
  : msg_(msg)
  {}
  Init_DynamixelMsgs_profile_acceleration goal_position(::dynamixel_rdk_msgs::msg::DynamixelMsgs::_goal_position_type arg)
  {
    msg_.goal_position = std::move(arg);
    return Init_DynamixelMsgs_profile_acceleration(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelMsgs msg_;
};

class Init_DynamixelMsgs_header
{
public:
  Init_DynamixelMsgs_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DynamixelMsgs_goal_position header(::dynamixel_rdk_msgs::msg::DynamixelMsgs::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DynamixelMsgs_goal_position(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelMsgs msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamixel_rdk_msgs::msg::DynamixelMsgs>()
{
  return dynamixel_rdk_msgs::msg::builder::Init_DynamixelMsgs_header();
}

}  // namespace dynamixel_rdk_msgs

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_MSGS__BUILDER_HPP_
