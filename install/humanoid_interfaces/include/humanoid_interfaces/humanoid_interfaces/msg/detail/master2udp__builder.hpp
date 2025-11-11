// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/Master2udp.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MASTER2UDP__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MASTER2UDP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/master2udp__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_Master2udp_myteam
{
public:
  explicit Init_Master2udp_myteam(::humanoid_interfaces::msg::Master2udp & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::Master2udp myteam(::humanoid_interfaces::msg::Master2udp::_myteam_type arg)
  {
    msg_.myteam = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2udp msg_;
};

class Init_Master2udp_ballcoory
{
public:
  explicit Init_Master2udp_ballcoory(::humanoid_interfaces::msg::Master2udp & msg)
  : msg_(msg)
  {}
  Init_Master2udp_myteam ballcoory(::humanoid_interfaces::msg::Master2udp::_ballcoory_type arg)
  {
    msg_.ballcoory = std::move(arg);
    return Init_Master2udp_myteam(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2udp msg_;
};

class Init_Master2udp_ballcoorx
{
public:
  explicit Init_Master2udp_ballcoorx(::humanoid_interfaces::msg::Master2udp & msg)
  : msg_(msg)
  {}
  Init_Master2udp_ballcoory ballcoorx(::humanoid_interfaces::msg::Master2udp::_ballcoorx_type arg)
  {
    msg_.ballcoorx = std::move(arg);
    return Init_Master2udp_ballcoory(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2udp msg_;
};

class Init_Master2udp_balldist
{
public:
  explicit Init_Master2udp_balldist(::humanoid_interfaces::msg::Master2udp & msg)
  : msg_(msg)
  {}
  Init_Master2udp_ballcoorx balldist(::humanoid_interfaces::msg::Master2udp::_balldist_type arg)
  {
    msg_.balldist = std::move(arg);
    return Init_Master2udp_ballcoorx(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2udp msg_;
};

class Init_Master2udp_robotimuyaw
{
public:
  explicit Init_Master2udp_robotimuyaw(::humanoid_interfaces::msg::Master2udp & msg)
  : msg_(msg)
  {}
  Init_Master2udp_balldist robotimuyaw(::humanoid_interfaces::msg::Master2udp::_robotimuyaw_type arg)
  {
    msg_.robotimuyaw = std::move(arg);
    return Init_Master2udp_balldist(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2udp msg_;
};

class Init_Master2udp_robotcoory
{
public:
  explicit Init_Master2udp_robotcoory(::humanoid_interfaces::msg::Master2udp & msg)
  : msg_(msg)
  {}
  Init_Master2udp_robotimuyaw robotcoory(::humanoid_interfaces::msg::Master2udp::_robotcoory_type arg)
  {
    msg_.robotcoory = std::move(arg);
    return Init_Master2udp_robotimuyaw(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2udp msg_;
};

class Init_Master2udp_robotcoorx
{
public:
  explicit Init_Master2udp_robotcoorx(::humanoid_interfaces::msg::Master2udp & msg)
  : msg_(msg)
  {}
  Init_Master2udp_robotcoory robotcoorx(::humanoid_interfaces::msg::Master2udp::_robotcoorx_type arg)
  {
    msg_.robotcoorx = std::move(arg);
    return Init_Master2udp_robotcoory(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2udp msg_;
};

class Init_Master2udp_robotstate
{
public:
  explicit Init_Master2udp_robotstate(::humanoid_interfaces::msg::Master2udp & msg)
  : msg_(msg)
  {}
  Init_Master2udp_robotcoorx robotstate(::humanoid_interfaces::msg::Master2udp::_robotstate_type arg)
  {
    msg_.robotstate = std::move(arg);
    return Init_Master2udp_robotcoorx(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2udp msg_;
};

class Init_Master2udp_robotnum
{
public:
  Init_Master2udp_robotnum()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Master2udp_robotstate robotnum(::humanoid_interfaces::msg::Master2udp::_robotnum_type arg)
  {
    msg_.robotnum = std::move(arg);
    return Init_Master2udp_robotstate(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2udp msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::Master2udp>()
{
  return humanoid_interfaces::msg::builder::Init_Master2udp_robotnum();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MASTER2UDP__BUILDER_HPP_
