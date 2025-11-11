// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/Master2IkMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MASTER2_IK_MSG__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MASTER2_IK_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/master2_ik_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_Master2IkMsg_one_step_flag
{
public:
  explicit Init_Master2IkMsg_one_step_flag(::humanoid_interfaces::msg::Master2IkMsg & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::Master2IkMsg one_step_flag(::humanoid_interfaces::msg::Master2IkMsg::_one_step_flag_type arg)
  {
    msg_.one_step_flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2IkMsg msg_;
};

class Init_Master2IkMsg_one_yaw
{
public:
  explicit Init_Master2IkMsg_one_yaw(::humanoid_interfaces::msg::Master2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Master2IkMsg_one_step_flag one_yaw(::humanoid_interfaces::msg::Master2IkMsg::_one_yaw_type arg)
  {
    msg_.one_yaw = std::move(arg);
    return Init_Master2IkMsg_one_step_flag(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2IkMsg msg_;
};

class Init_Master2IkMsg_one_y_length
{
public:
  explicit Init_Master2IkMsg_one_y_length(::humanoid_interfaces::msg::Master2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Master2IkMsg_one_yaw one_y_length(::humanoid_interfaces::msg::Master2IkMsg::_one_y_length_type arg)
  {
    msg_.one_y_length = std::move(arg);
    return Init_Master2IkMsg_one_yaw(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2IkMsg msg_;
};

class Init_Master2IkMsg_one_x_length
{
public:
  explicit Init_Master2IkMsg_one_x_length(::humanoid_interfaces::msg::Master2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Master2IkMsg_one_y_length one_x_length(::humanoid_interfaces::msg::Master2IkMsg::_one_x_length_type arg)
  {
    msg_.one_x_length = std::move(arg);
    return Init_Master2IkMsg_one_y_length(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2IkMsg msg_;
};

class Init_Master2IkMsg_flag
{
public:
  explicit Init_Master2IkMsg_flag(::humanoid_interfaces::msg::Master2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Master2IkMsg_one_x_length flag(::humanoid_interfaces::msg::Master2IkMsg::_flag_type arg)
  {
    msg_.flag = std::move(arg);
    return Init_Master2IkMsg_one_x_length(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2IkMsg msg_;
};

class Init_Master2IkMsg_yaw
{
public:
  explicit Init_Master2IkMsg_yaw(::humanoid_interfaces::msg::Master2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Master2IkMsg_flag yaw(::humanoid_interfaces::msg::Master2IkMsg::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_Master2IkMsg_flag(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2IkMsg msg_;
};

class Init_Master2IkMsg_y_length
{
public:
  explicit Init_Master2IkMsg_y_length(::humanoid_interfaces::msg::Master2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Master2IkMsg_yaw y_length(::humanoid_interfaces::msg::Master2IkMsg::_y_length_type arg)
  {
    msg_.y_length = std::move(arg);
    return Init_Master2IkMsg_yaw(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2IkMsg msg_;
};

class Init_Master2IkMsg_x_length
{
public:
  Init_Master2IkMsg_x_length()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Master2IkMsg_y_length x_length(::humanoid_interfaces::msg::Master2IkMsg::_x_length_type arg)
  {
    msg_.x_length = std::move(arg);
    return Init_Master2IkMsg_y_length(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2IkMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::Master2IkMsg>()
{
  return humanoid_interfaces::msg::builder::Init_Master2IkMsg_x_length();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MASTER2_IK_MSG__BUILDER_HPP_
