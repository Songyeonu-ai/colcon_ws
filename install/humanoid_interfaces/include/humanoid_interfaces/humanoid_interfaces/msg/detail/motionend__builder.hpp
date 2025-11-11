// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/Motionend.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MOTIONEND__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MOTIONEND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/motionend__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_Motionend_motionnum
{
public:
  explicit Init_Motionend_motionnum(::humanoid_interfaces::msg::Motionend & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::Motionend motionnum(::humanoid_interfaces::msg::Motionend::_motionnum_type arg)
  {
    msg_.motionnum = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::Motionend msg_;
};

class Init_Motionend_motionending
{
public:
  explicit Init_Motionend_motionending(::humanoid_interfaces::msg::Motionend & msg)
  : msg_(msg)
  {}
  Init_Motionend_motionnum motionending(::humanoid_interfaces::msg::Motionend::_motionending_type arg)
  {
    msg_.motionending = std::move(arg);
    return Init_Motionend_motionnum(msg_);
  }

private:
  ::humanoid_interfaces::msg::Motionend msg_;
};

class Init_Motionend_motionstarting
{
public:
  explicit Init_Motionend_motionstarting(::humanoid_interfaces::msg::Motionend & msg)
  : msg_(msg)
  {}
  Init_Motionend_motionending motionstarting(::humanoid_interfaces::msg::Motionend::_motionstarting_type arg)
  {
    msg_.motionstarting = std::move(arg);
    return Init_Motionend_motionending(msg_);
  }

private:
  ::humanoid_interfaces::msg::Motionend msg_;
};

class Init_Motionend_motioning
{
public:
  explicit Init_Motionend_motioning(::humanoid_interfaces::msg::Motionend & msg)
  : msg_(msg)
  {}
  Init_Motionend_motionstarting motioning(::humanoid_interfaces::msg::Motionend::_motioning_type arg)
  {
    msg_.motioning = std::move(arg);
    return Init_Motionend_motionstarting(msg_);
  }

private:
  ::humanoid_interfaces::msg::Motionend msg_;
};

class Init_Motionend_motionend
{
public:
  Init_Motionend_motionend()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Motionend_motioning motionend(::humanoid_interfaces::msg::Motionend::_motionend_type arg)
  {
    msg_.motionend = std::move(arg);
    return Init_Motionend_motioning(msg_);
  }

private:
  ::humanoid_interfaces::msg::Motionend msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::Motionend>()
{
  return humanoid_interfaces::msg::builder::Init_Motionend_motionend();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MOTIONEND__BUILDER_HPP_
