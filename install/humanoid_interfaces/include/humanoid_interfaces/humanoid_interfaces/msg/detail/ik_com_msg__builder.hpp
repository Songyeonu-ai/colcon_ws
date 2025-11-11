// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/IkComMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_COM_MSG__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_COM_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/ik_com_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_IkComMsg_y_com
{
public:
  explicit Init_IkComMsg_y_com(::humanoid_interfaces::msg::IkComMsg & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::IkComMsg y_com(::humanoid_interfaces::msg::IkComMsg::_y_com_type arg)
  {
    msg_.y_com = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkComMsg msg_;
};

class Init_IkComMsg_x_com
{
public:
  Init_IkComMsg_x_com()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IkComMsg_y_com x_com(::humanoid_interfaces::msg::IkComMsg::_x_com_type arg)
  {
    msg_.x_com = std::move(arg);
    return Init_IkComMsg_y_com(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkComMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::IkComMsg>()
{
  return humanoid_interfaces::msg::builder::Init_IkComMsg_x_com();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_COM_MSG__BUILDER_HPP_
