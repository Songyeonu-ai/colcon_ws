// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/Udp2master.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__UDP2MASTER__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__UDP2MASTER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/udp2master__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_Udp2master_balltheta
{
public:
  explicit Init_Udp2master_balltheta(::humanoid_interfaces::msg::Udp2master & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::Udp2master balltheta(::humanoid_interfaces::msg::Udp2master::_balltheta_type arg)
  {
    msg_.balltheta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::Udp2master msg_;
};

class Init_Udp2master_balldist
{
public:
  explicit Init_Udp2master_balldist(::humanoid_interfaces::msg::Udp2master & msg)
  : msg_(msg)
  {}
  Init_Udp2master_balltheta balldist(::humanoid_interfaces::msg::Udp2master::_balldist_type arg)
  {
    msg_.balldist = std::move(arg);
    return Init_Udp2master_balltheta(msg_);
  }

private:
  ::humanoid_interfaces::msg::Udp2master msg_;
};

class Init_Udp2master_bally
{
public:
  explicit Init_Udp2master_bally(::humanoid_interfaces::msg::Udp2master & msg)
  : msg_(msg)
  {}
  Init_Udp2master_balldist bally(::humanoid_interfaces::msg::Udp2master::_bally_type arg)
  {
    msg_.bally = std::move(arg);
    return Init_Udp2master_balldist(msg_);
  }

private:
  ::humanoid_interfaces::msg::Udp2master msg_;
};

class Init_Udp2master_ballx
{
public:
  explicit Init_Udp2master_ballx(::humanoid_interfaces::msg::Udp2master & msg)
  : msg_(msg)
  {}
  Init_Udp2master_bally ballx(::humanoid_interfaces::msg::Udp2master::_ballx_type arg)
  {
    msg_.ballx = std::move(arg);
    return Init_Udp2master_bally(msg_);
  }

private:
  ::humanoid_interfaces::msg::Udp2master msg_;
};

class Init_Udp2master_localyaw
{
public:
  explicit Init_Udp2master_localyaw(::humanoid_interfaces::msg::Udp2master & msg)
  : msg_(msg)
  {}
  Init_Udp2master_ballx localyaw(::humanoid_interfaces::msg::Udp2master::_localyaw_type arg)
  {
    msg_.localyaw = std::move(arg);
    return Init_Udp2master_ballx(msg_);
  }

private:
  ::humanoid_interfaces::msg::Udp2master msg_;
};

class Init_Udp2master_localy
{
public:
  explicit Init_Udp2master_localy(::humanoid_interfaces::msg::Udp2master & msg)
  : msg_(msg)
  {}
  Init_Udp2master_localyaw localy(::humanoid_interfaces::msg::Udp2master::_localy_type arg)
  {
    msg_.localy = std::move(arg);
    return Init_Udp2master_localyaw(msg_);
  }

private:
  ::humanoid_interfaces::msg::Udp2master msg_;
};

class Init_Udp2master_localx
{
public:
  explicit Init_Udp2master_localx(::humanoid_interfaces::msg::Udp2master & msg)
  : msg_(msg)
  {}
  Init_Udp2master_localy localx(::humanoid_interfaces::msg::Udp2master::_localx_type arg)
  {
    msg_.localx = std::move(arg);
    return Init_Udp2master_localy(msg_);
  }

private:
  ::humanoid_interfaces::msg::Udp2master msg_;
};

class Init_Udp2master_robotcase
{
public:
  explicit Init_Udp2master_robotcase(::humanoid_interfaces::msg::Udp2master & msg)
  : msg_(msg)
  {}
  Init_Udp2master_localx robotcase(::humanoid_interfaces::msg::Udp2master::_robotcase_type arg)
  {
    msg_.robotcase = std::move(arg);
    return Init_Udp2master_localx(msg_);
  }

private:
  ::humanoid_interfaces::msg::Udp2master msg_;
};

class Init_Udp2master_robotnum
{
public:
  Init_Udp2master_robotnum()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Udp2master_robotcase robotnum(::humanoid_interfaces::msg::Udp2master::_robotnum_type arg)
  {
    msg_.robotnum = std::move(arg);
    return Init_Udp2master_robotcase(msg_);
  }

private:
  ::humanoid_interfaces::msg::Udp2master msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::Udp2master>()
{
  return humanoid_interfaces::msg::builder::Init_Udp2master_robotnum();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__UDP2MASTER__BUILDER_HPP_
