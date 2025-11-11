// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/Motionnum.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MOTIONNUM__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MOTIONNUM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/motionnum__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_Motionnum_motionnum
{
public:
  explicit Init_Motionnum_motionnum(::humanoid_interfaces::msg::Motionnum & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::Motionnum motionnum(::humanoid_interfaces::msg::Motionnum::_motionnum_type arg)
  {
    msg_.motionnum = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::Motionnum msg_;
};

class Init_Motionnum_motionmode
{
public:
  Init_Motionnum_motionmode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Motionnum_motionnum motionmode(::humanoid_interfaces::msg::Motionnum::_motionmode_type arg)
  {
    msg_.motionmode = std::move(arg);
    return Init_Motionnum_motionnum(msg_);
  }

private:
  ::humanoid_interfaces::msg::Motionnum msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::Motionnum>()
{
  return humanoid_interfaces::msg::builder::Init_Motionnum_motionmode();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MOTIONNUM__BUILDER_HPP_
