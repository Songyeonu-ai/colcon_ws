// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/ZmpMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ZMP_MSG__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__ZMP_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/zmp_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_ZmpMsg_both_feet
{
public:
  explicit Init_ZmpMsg_both_feet(::humanoid_interfaces::msg::ZmpMsg & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::ZmpMsg both_feet(::humanoid_interfaces::msg::ZmpMsg::_both_feet_type arg)
  {
    msg_.both_feet = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::ZmpMsg msg_;
};

class Init_ZmpMsg_right_foot
{
public:
  explicit Init_ZmpMsg_right_foot(::humanoid_interfaces::msg::ZmpMsg & msg)
  : msg_(msg)
  {}
  Init_ZmpMsg_both_feet right_foot(::humanoid_interfaces::msg::ZmpMsg::_right_foot_type arg)
  {
    msg_.right_foot = std::move(arg);
    return Init_ZmpMsg_both_feet(msg_);
  }

private:
  ::humanoid_interfaces::msg::ZmpMsg msg_;
};

class Init_ZmpMsg_left_foot
{
public:
  explicit Init_ZmpMsg_left_foot(::humanoid_interfaces::msg::ZmpMsg & msg)
  : msg_(msg)
  {}
  Init_ZmpMsg_right_foot left_foot(::humanoid_interfaces::msg::ZmpMsg::_left_foot_type arg)
  {
    msg_.left_foot = std::move(arg);
    return Init_ZmpMsg_right_foot(msg_);
  }

private:
  ::humanoid_interfaces::msg::ZmpMsg msg_;
};

class Init_ZmpMsg_total_y_zmp
{
public:
  explicit Init_ZmpMsg_total_y_zmp(::humanoid_interfaces::msg::ZmpMsg & msg)
  : msg_(msg)
  {}
  Init_ZmpMsg_left_foot total_y_zmp(::humanoid_interfaces::msg::ZmpMsg::_total_y_zmp_type arg)
  {
    msg_.total_y_zmp = std::move(arg);
    return Init_ZmpMsg_left_foot(msg_);
  }

private:
  ::humanoid_interfaces::msg::ZmpMsg msg_;
};

class Init_ZmpMsg_total_x_zmp
{
public:
  explicit Init_ZmpMsg_total_x_zmp(::humanoid_interfaces::msg::ZmpMsg & msg)
  : msg_(msg)
  {}
  Init_ZmpMsg_total_y_zmp total_x_zmp(::humanoid_interfaces::msg::ZmpMsg::_total_x_zmp_type arg)
  {
    msg_.total_x_zmp = std::move(arg);
    return Init_ZmpMsg_total_y_zmp(msg_);
  }

private:
  ::humanoid_interfaces::msg::ZmpMsg msg_;
};

class Init_ZmpMsg_right_y_zmp
{
public:
  explicit Init_ZmpMsg_right_y_zmp(::humanoid_interfaces::msg::ZmpMsg & msg)
  : msg_(msg)
  {}
  Init_ZmpMsg_total_x_zmp right_y_zmp(::humanoid_interfaces::msg::ZmpMsg::_right_y_zmp_type arg)
  {
    msg_.right_y_zmp = std::move(arg);
    return Init_ZmpMsg_total_x_zmp(msg_);
  }

private:
  ::humanoid_interfaces::msg::ZmpMsg msg_;
};

class Init_ZmpMsg_right_x_zmp
{
public:
  explicit Init_ZmpMsg_right_x_zmp(::humanoid_interfaces::msg::ZmpMsg & msg)
  : msg_(msg)
  {}
  Init_ZmpMsg_right_y_zmp right_x_zmp(::humanoid_interfaces::msg::ZmpMsg::_right_x_zmp_type arg)
  {
    msg_.right_x_zmp = std::move(arg);
    return Init_ZmpMsg_right_y_zmp(msg_);
  }

private:
  ::humanoid_interfaces::msg::ZmpMsg msg_;
};

class Init_ZmpMsg_left_y_zmp
{
public:
  explicit Init_ZmpMsg_left_y_zmp(::humanoid_interfaces::msg::ZmpMsg & msg)
  : msg_(msg)
  {}
  Init_ZmpMsg_right_x_zmp left_y_zmp(::humanoid_interfaces::msg::ZmpMsg::_left_y_zmp_type arg)
  {
    msg_.left_y_zmp = std::move(arg);
    return Init_ZmpMsg_right_x_zmp(msg_);
  }

private:
  ::humanoid_interfaces::msg::ZmpMsg msg_;
};

class Init_ZmpMsg_left_x_zmp
{
public:
  Init_ZmpMsg_left_x_zmp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ZmpMsg_left_y_zmp left_x_zmp(::humanoid_interfaces::msg::ZmpMsg::_left_x_zmp_type arg)
  {
    msg_.left_x_zmp = std::move(arg);
    return Init_ZmpMsg_left_y_zmp(msg_);
  }

private:
  ::humanoid_interfaces::msg::ZmpMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::ZmpMsg>()
{
  return humanoid_interfaces::msg::builder::Init_ZmpMsg_left_x_zmp();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ZMP_MSG__BUILDER_HPP_
