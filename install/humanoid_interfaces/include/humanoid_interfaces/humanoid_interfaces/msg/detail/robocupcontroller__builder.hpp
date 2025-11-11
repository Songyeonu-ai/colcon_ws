// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/Robocupcontroller.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPCONTROLLER__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPCONTROLLER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/robocupcontroller__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_Robocupcontroller_secondinfo
{
public:
  explicit Init_Robocupcontroller_secondinfo(::humanoid_interfaces::msg::Robocupcontroller & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::Robocupcontroller secondinfo(::humanoid_interfaces::msg::Robocupcontroller::_secondinfo_type arg)
  {
    msg_.secondinfo = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupcontroller msg_;
};

class Init_Robocupcontroller_secondstate
{
public:
  explicit Init_Robocupcontroller_secondstate(::humanoid_interfaces::msg::Robocupcontroller & msg)
  : msg_(msg)
  {}
  Init_Robocupcontroller_secondinfo secondstate(::humanoid_interfaces::msg::Robocupcontroller::_secondstate_type arg)
  {
    msg_.secondstate = std::move(arg);
    return Init_Robocupcontroller_secondinfo(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupcontroller msg_;
};

class Init_Robocupcontroller_kickoffteam
{
public:
  explicit Init_Robocupcontroller_kickoffteam(::humanoid_interfaces::msg::Robocupcontroller & msg)
  : msg_(msg)
  {}
  Init_Robocupcontroller_secondstate kickoffteam(::humanoid_interfaces::msg::Robocupcontroller::_kickoffteam_type arg)
  {
    msg_.kickoffteam = std::move(arg);
    return Init_Robocupcontroller_secondstate(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupcontroller msg_;
};

class Init_Robocupcontroller_firstside
{
public:
  explicit Init_Robocupcontroller_firstside(::humanoid_interfaces::msg::Robocupcontroller & msg)
  : msg_(msg)
  {}
  Init_Robocupcontroller_kickoffteam firstside(::humanoid_interfaces::msg::Robocupcontroller::_firstside_type arg)
  {
    msg_.firstside = std::move(arg);
    return Init_Robocupcontroller_kickoffteam(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupcontroller msg_;
};

class Init_Robocupcontroller_firsthalf
{
public:
  explicit Init_Robocupcontroller_firsthalf(::humanoid_interfaces::msg::Robocupcontroller & msg)
  : msg_(msg)
  {}
  Init_Robocupcontroller_firstside firsthalf(::humanoid_interfaces::msg::Robocupcontroller::_firsthalf_type arg)
  {
    msg_.firsthalf = std::move(arg);
    return Init_Robocupcontroller_firstside(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupcontroller msg_;
};

class Init_Robocupcontroller_penalty
{
public:
  explicit Init_Robocupcontroller_penalty(::humanoid_interfaces::msg::Robocupcontroller & msg)
  : msg_(msg)
  {}
  Init_Robocupcontroller_firsthalf penalty(::humanoid_interfaces::msg::Robocupcontroller::_penalty_type arg)
  {
    msg_.penalty = std::move(arg);
    return Init_Robocupcontroller_firsthalf(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupcontroller msg_;
};

class Init_Robocupcontroller_readytime
{
public:
  explicit Init_Robocupcontroller_readytime(::humanoid_interfaces::msg::Robocupcontroller & msg)
  : msg_(msg)
  {}
  Init_Robocupcontroller_penalty readytime(::humanoid_interfaces::msg::Robocupcontroller::_readytime_type arg)
  {
    msg_.readytime = std::move(arg);
    return Init_Robocupcontroller_penalty(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupcontroller msg_;
};

class Init_Robocupcontroller_state
{
public:
  explicit Init_Robocupcontroller_state(::humanoid_interfaces::msg::Robocupcontroller & msg)
  : msg_(msg)
  {}
  Init_Robocupcontroller_readytime state(::humanoid_interfaces::msg::Robocupcontroller::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_Robocupcontroller_readytime(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupcontroller msg_;
};

class Init_Robocupcontroller_robotnum
{
public:
  Init_Robocupcontroller_robotnum()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Robocupcontroller_state robotnum(::humanoid_interfaces::msg::Robocupcontroller::_robotnum_type arg)
  {
    msg_.robotnum = std::move(arg);
    return Init_Robocupcontroller_state(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupcontroller msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::Robocupcontroller>()
{
  return humanoid_interfaces::msg::builder::Init_Robocupcontroller_robotnum();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPCONTROLLER__BUILDER_HPP_
