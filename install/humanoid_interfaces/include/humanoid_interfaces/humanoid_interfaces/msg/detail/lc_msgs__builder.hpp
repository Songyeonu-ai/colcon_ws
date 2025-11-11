// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/LCMsgs.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__LC_MSGS__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__LC_MSGS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/lc_msgs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_LCMsgs_l_lc_data
{
public:
  explicit Init_LCMsgs_l_lc_data(::humanoid_interfaces::msg::LCMsgs & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::LCMsgs l_lc_data(::humanoid_interfaces::msg::LCMsgs::_l_lc_data_type arg)
  {
    msg_.l_lc_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::LCMsgs msg_;
};

class Init_LCMsgs_r_lc_data
{
public:
  Init_LCMsgs_r_lc_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LCMsgs_l_lc_data r_lc_data(::humanoid_interfaces::msg::LCMsgs::_r_lc_data_type arg)
  {
    msg_.r_lc_data = std::move(arg);
    return Init_LCMsgs_l_lc_data(msg_);
  }

private:
  ::humanoid_interfaces::msg::LCMsgs msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::LCMsgs>()
{
  return humanoid_interfaces::msg::builder::Init_LCMsgs_r_lc_data();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__LC_MSGS__BUILDER_HPP_
