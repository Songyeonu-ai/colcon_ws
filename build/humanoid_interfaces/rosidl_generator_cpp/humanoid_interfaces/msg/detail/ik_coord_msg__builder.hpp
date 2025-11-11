// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/IkCoordMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_COORD_MSG__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_COORD_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/ik_coord_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_IkCoordMsg_y
{
public:
  explicit Init_IkCoordMsg_y(::humanoid_interfaces::msg::IkCoordMsg & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::IkCoordMsg y(::humanoid_interfaces::msg::IkCoordMsg::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkCoordMsg msg_;
};

class Init_IkCoordMsg_x
{
public:
  Init_IkCoordMsg_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IkCoordMsg_y x(::humanoid_interfaces::msg::IkCoordMsg::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_IkCoordMsg_y(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkCoordMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::IkCoordMsg>()
{
  return humanoid_interfaces::msg::builder::Init_IkCoordMsg_x();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_COORD_MSG__BUILDER_HPP_
