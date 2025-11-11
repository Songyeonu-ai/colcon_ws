// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/Gamecontrolreturndata.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLRETURNDATA__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLRETURNDATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/gamecontrolreturndata__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_Gamecontrolreturndata_message
{
public:
  Init_Gamecontrolreturndata_message()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::humanoid_interfaces::msg::Gamecontrolreturndata message(::humanoid_interfaces::msg::Gamecontrolreturndata::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::Gamecontrolreturndata msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::Gamecontrolreturndata>()
{
  return humanoid_interfaces::msg::builder::Init_Gamecontrolreturndata_message();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLRETURNDATA__BUILDER_HPP_
