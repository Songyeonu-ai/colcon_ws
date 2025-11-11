// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/Pidtuning.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__PIDTUNING__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__PIDTUNING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/pidtuning__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_Pidtuning_kd
{
public:
  explicit Init_Pidtuning_kd(::humanoid_interfaces::msg::Pidtuning & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::Pidtuning kd(::humanoid_interfaces::msg::Pidtuning::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::Pidtuning msg_;
};

class Init_Pidtuning_ki
{
public:
  explicit Init_Pidtuning_ki(::humanoid_interfaces::msg::Pidtuning & msg)
  : msg_(msg)
  {}
  Init_Pidtuning_kd ki(::humanoid_interfaces::msg::Pidtuning::_ki_type arg)
  {
    msg_.ki = std::move(arg);
    return Init_Pidtuning_kd(msg_);
  }

private:
  ::humanoid_interfaces::msg::Pidtuning msg_;
};

class Init_Pidtuning_kp
{
public:
  Init_Pidtuning_kp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pidtuning_ki kp(::humanoid_interfaces::msg::Pidtuning::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_Pidtuning_ki(msg_);
  }

private:
  ::humanoid_interfaces::msg::Pidtuning msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::Pidtuning>()
{
  return humanoid_interfaces::msg::builder::Init_Pidtuning_kp();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__PIDTUNING__BUILDER_HPP_
