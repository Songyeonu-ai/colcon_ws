// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dynamixel_rdk_msgs:msg/DynamixelStatusMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_STATUS_MSGS__BUILDER_HPP_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_STATUS_MSGS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dynamixel_rdk_msgs/msg/detail/dynamixel_status_msgs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dynamixel_rdk_msgs
{

namespace msg
{

namespace builder
{

class Init_DynamixelStatusMsgs_min_max_position
{
public:
  explicit Init_DynamixelStatusMsgs_min_max_position(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs & msg)
  : msg_(msg)
  {}
  ::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs min_max_position(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs::_min_max_position_type arg)
  {
    msg_.min_max_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs msg_;
};

class Init_DynamixelStatusMsgs_present_temperature
{
public:
  explicit Init_DynamixelStatusMsgs_present_temperature(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs & msg)
  : msg_(msg)
  {}
  Init_DynamixelStatusMsgs_min_max_position present_temperature(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs::_present_temperature_type arg)
  {
    msg_.present_temperature = std::move(arg);
    return Init_DynamixelStatusMsgs_min_max_position(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs msg_;
};

class Init_DynamixelStatusMsgs_present_voltage
{
public:
  explicit Init_DynamixelStatusMsgs_present_voltage(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs & msg)
  : msg_(msg)
  {}
  Init_DynamixelStatusMsgs_present_temperature present_voltage(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs::_present_voltage_type arg)
  {
    msg_.present_voltage = std::move(arg);
    return Init_DynamixelStatusMsgs_present_temperature(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs msg_;
};

class Init_DynamixelStatusMsgs_present_current
{
public:
  explicit Init_DynamixelStatusMsgs_present_current(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs & msg)
  : msg_(msg)
  {}
  Init_DynamixelStatusMsgs_present_voltage present_current(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs::_present_current_type arg)
  {
    msg_.present_current = std::move(arg);
    return Init_DynamixelStatusMsgs_present_voltage(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs msg_;
};

class Init_DynamixelStatusMsgs_present_acceleration
{
public:
  explicit Init_DynamixelStatusMsgs_present_acceleration(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs & msg)
  : msg_(msg)
  {}
  Init_DynamixelStatusMsgs_present_current present_acceleration(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs::_present_acceleration_type arg)
  {
    msg_.present_acceleration = std::move(arg);
    return Init_DynamixelStatusMsgs_present_current(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs msg_;
};

class Init_DynamixelStatusMsgs_present_velocity
{
public:
  explicit Init_DynamixelStatusMsgs_present_velocity(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs & msg)
  : msg_(msg)
  {}
  Init_DynamixelStatusMsgs_present_acceleration present_velocity(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs::_present_velocity_type arg)
  {
    msg_.present_velocity = std::move(arg);
    return Init_DynamixelStatusMsgs_present_acceleration(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs msg_;
};

class Init_DynamixelStatusMsgs_present_position
{
public:
  explicit Init_DynamixelStatusMsgs_present_position(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs & msg)
  : msg_(msg)
  {}
  Init_DynamixelStatusMsgs_present_velocity present_position(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs::_present_position_type arg)
  {
    msg_.present_position = std::move(arg);
    return Init_DynamixelStatusMsgs_present_velocity(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs msg_;
};

class Init_DynamixelStatusMsgs_error_status
{
public:
  explicit Init_DynamixelStatusMsgs_error_status(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs & msg)
  : msg_(msg)
  {}
  Init_DynamixelStatusMsgs_present_position error_status(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs::_error_status_type arg)
  {
    msg_.error_status = std::move(arg);
    return Init_DynamixelStatusMsgs_present_position(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs msg_;
};

class Init_DynamixelStatusMsgs_torque_enabled
{
public:
  explicit Init_DynamixelStatusMsgs_torque_enabled(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs & msg)
  : msg_(msg)
  {}
  Init_DynamixelStatusMsgs_error_status torque_enabled(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs::_torque_enabled_type arg)
  {
    msg_.torque_enabled = std::move(arg);
    return Init_DynamixelStatusMsgs_error_status(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs msg_;
};

class Init_DynamixelStatusMsgs_id
{
public:
  explicit Init_DynamixelStatusMsgs_id(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs & msg)
  : msg_(msg)
  {}
  Init_DynamixelStatusMsgs_torque_enabled id(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_DynamixelStatusMsgs_torque_enabled(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs msg_;
};

class Init_DynamixelStatusMsgs_header
{
public:
  Init_DynamixelStatusMsgs_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DynamixelStatusMsgs_id header(::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DynamixelStatusMsgs_id(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamixel_rdk_msgs::msg::DynamixelStatusMsgs>()
{
  return dynamixel_rdk_msgs::msg::builder::Init_DynamixelStatusMsgs_header();
}

}  // namespace dynamixel_rdk_msgs

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_STATUS_MSGS__BUILDER_HPP_
