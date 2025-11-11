// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/Master2vision25.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MASTER2VISION25__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MASTER2VISION25__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/master2vision25__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_Master2vision25_tilt
{
public:
  explicit Init_Master2vision25_tilt(::humanoid_interfaces::msg::Master2vision25 & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::Master2vision25 tilt(::humanoid_interfaces::msg::Master2vision25::_tilt_type arg)
  {
    msg_.tilt = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2vision25 msg_;
};

class Init_Master2vision25_pan
{
public:
  explicit Init_Master2vision25_pan(::humanoid_interfaces::msg::Master2vision25 & msg)
  : msg_(msg)
  {}
  Init_Master2vision25_tilt pan(::humanoid_interfaces::msg::Master2vision25::_pan_type arg)
  {
    msg_.pan = std::move(arg);
    return Init_Master2vision25_tilt(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2vision25 msg_;
};

class Init_Master2vision25_scanmode
{
public:
  Init_Master2vision25_scanmode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Master2vision25_pan scanmode(::humanoid_interfaces::msg::Master2vision25::_scanmode_type arg)
  {
    msg_.scanmode = std::move(arg);
    return Init_Master2vision25_pan(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2vision25 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::Master2vision25>()
{
  return humanoid_interfaces::msg::builder::Init_Master2vision25_scanmode();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MASTER2VISION25__BUILDER_HPP_
