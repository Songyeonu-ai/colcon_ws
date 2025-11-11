// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/MotorMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MOTOR_MSG__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MOTOR_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/motor_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_MotorMsg_acceleration
{
public:
  explicit Init_MotorMsg_acceleration(::humanoid_interfaces::msg::MotorMsg & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::MotorMsg acceleration(::humanoid_interfaces::msg::MotorMsg::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::MotorMsg msg_;
};

class Init_MotorMsg_type
{
public:
  explicit Init_MotorMsg_type(::humanoid_interfaces::msg::MotorMsg & msg)
  : msg_(msg)
  {}
  Init_MotorMsg_acceleration type(::humanoid_interfaces::msg::MotorMsg::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_MotorMsg_acceleration(msg_);
  }

private:
  ::humanoid_interfaces::msg::MotorMsg msg_;
};

class Init_MotorMsg_position
{
public:
  explicit Init_MotorMsg_position(::humanoid_interfaces::msg::MotorMsg & msg)
  : msg_(msg)
  {}
  Init_MotorMsg_type position(::humanoid_interfaces::msg::MotorMsg::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_MotorMsg_type(msg_);
  }

private:
  ::humanoid_interfaces::msg::MotorMsg msg_;
};

class Init_MotorMsg_speed
{
public:
  explicit Init_MotorMsg_speed(::humanoid_interfaces::msg::MotorMsg & msg)
  : msg_(msg)
  {}
  Init_MotorMsg_position speed(::humanoid_interfaces::msg::MotorMsg::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_MotorMsg_position(msg_);
  }

private:
  ::humanoid_interfaces::msg::MotorMsg msg_;
};

class Init_MotorMsg_id
{
public:
  explicit Init_MotorMsg_id(::humanoid_interfaces::msg::MotorMsg & msg)
  : msg_(msg)
  {}
  Init_MotorMsg_speed id(::humanoid_interfaces::msg::MotorMsg::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_MotorMsg_speed(msg_);
  }

private:
  ::humanoid_interfaces::msg::MotorMsg msg_;
};

class Init_MotorMsg_mode
{
public:
  explicit Init_MotorMsg_mode(::humanoid_interfaces::msg::MotorMsg & msg)
  : msg_(msg)
  {}
  Init_MotorMsg_id mode(::humanoid_interfaces::msg::MotorMsg::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_MotorMsg_id(msg_);
  }

private:
  ::humanoid_interfaces::msg::MotorMsg msg_;
};

class Init_MotorMsg_length
{
public:
  Init_MotorMsg_length()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorMsg_mode length(::humanoid_interfaces::msg::MotorMsg::_length_type arg)
  {
    msg_.length = std::move(arg);
    return Init_MotorMsg_mode(msg_);
  }

private:
  ::humanoid_interfaces::msg::MotorMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::MotorMsg>()
{
  return humanoid_interfaces::msg::builder::Init_MotorMsg_length();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MOTOR_MSG__BUILDER_HPP_
