// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/IkEndMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_END_MSG__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_END_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/ik_end_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_IkEndMsg_onestep_end
{
public:
  explicit Init_IkEndMsg_onestep_end(::humanoid_interfaces::msg::IkEndMsg & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::IkEndMsg onestep_end(::humanoid_interfaces::msg::IkEndMsg::_onestep_end_type arg)
  {
    msg_.onestep_end = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkEndMsg msg_;
};

class Init_IkEndMsg_ikend
{
public:
  Init_IkEndMsg_ikend()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IkEndMsg_onestep_end ikend(::humanoid_interfaces::msg::IkEndMsg::_ikend_type arg)
  {
    msg_.ikend = std::move(arg);
    return Init_IkEndMsg_onestep_end(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkEndMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::IkEndMsg>()
{
  return humanoid_interfaces::msg::builder::Init_IkEndMsg_ikend();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_END_MSG__BUILDER_HPP_
