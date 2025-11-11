// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/MotionOperator.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MOTION_OPERATOR__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MOTION_OPERATOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/motion_operator__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_MotionOperator_motion_end
{
public:
  explicit Init_MotionOperator_motion_end(::humanoid_interfaces::msg::MotionOperator & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::MotionOperator motion_end(::humanoid_interfaces::msg::MotionOperator::_motion_end_type arg)
  {
    msg_.motion_end = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::MotionOperator msg_;
};

class Init_MotionOperator_motion_num
{
public:
  Init_MotionOperator_motion_num()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotionOperator_motion_end motion_num(::humanoid_interfaces::msg::MotionOperator::_motion_num_type arg)
  {
    msg_.motion_num = std::move(arg);
    return Init_MotionOperator_motion_end(msg_);
  }

private:
  ::humanoid_interfaces::msg::MotionOperator msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::MotionOperator>()
{
  return humanoid_interfaces::msg::builder::Init_MotionOperator_motion_num();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MOTION_OPERATOR__BUILDER_HPP_
