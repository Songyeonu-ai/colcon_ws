// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/Robocupvision25feature.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25FEATURE__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25FEATURE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/robocupvision25feature__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_Robocupvision25feature_point_vec_y
{
public:
  explicit Init_Robocupvision25feature_point_vec_y(::humanoid_interfaces::msg::Robocupvision25feature & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::Robocupvision25feature point_vec_y(::humanoid_interfaces::msg::Robocupvision25feature::_point_vec_y_type arg)
  {
    msg_.point_vec_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25feature msg_;
};

class Init_Robocupvision25feature_point_vec_x
{
public:
  explicit Init_Robocupvision25feature_point_vec_x(::humanoid_interfaces::msg::Robocupvision25feature & msg)
  : msg_(msg)
  {}
  Init_Robocupvision25feature_point_vec_y point_vec_x(::humanoid_interfaces::msg::Robocupvision25feature::_point_vec_x_type arg)
  {
    msg_.point_vec_x = std::move(arg);
    return Init_Robocupvision25feature_point_vec_y(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25feature msg_;
};

class Init_Robocupvision25feature_distance
{
public:
  explicit Init_Robocupvision25feature_distance(::humanoid_interfaces::msg::Robocupvision25feature & msg)
  : msg_(msg)
  {}
  Init_Robocupvision25feature_point_vec_x distance(::humanoid_interfaces::msg::Robocupvision25feature::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_Robocupvision25feature_point_vec_x(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25feature msg_;
};

class Init_Robocupvision25feature_confidence
{
public:
  Init_Robocupvision25feature_confidence()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Robocupvision25feature_distance confidence(::humanoid_interfaces::msg::Robocupvision25feature::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Robocupvision25feature_distance(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25feature msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::Robocupvision25feature>()
{
  return humanoid_interfaces::msg::builder::Init_Robocupvision25feature_confidence();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25FEATURE__BUILDER_HPP_
