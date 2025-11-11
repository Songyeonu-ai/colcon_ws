// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/IkPatternMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_PATTERN_MSG__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_PATTERN_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/ik_pattern_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_IkPatternMsg_trpattern
{
public:
  explicit Init_IkPatternMsg_trpattern(::humanoid_interfaces::msg::IkPatternMsg & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::IkPatternMsg trpattern(::humanoid_interfaces::msg::IkPatternMsg::_trpattern_type arg)
  {
    msg_.trpattern = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkPatternMsg msg_;
};

class Init_IkPatternMsg_srpattern
{
public:
  explicit Init_IkPatternMsg_srpattern(::humanoid_interfaces::msg::IkPatternMsg & msg)
  : msg_(msg)
  {}
  Init_IkPatternMsg_trpattern srpattern(::humanoid_interfaces::msg::IkPatternMsg::_srpattern_type arg)
  {
    msg_.srpattern = std::move(arg);
    return Init_IkPatternMsg_trpattern(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkPatternMsg msg_;
};

class Init_IkPatternMsg_zrpattern
{
public:
  explicit Init_IkPatternMsg_zrpattern(::humanoid_interfaces::msg::IkPatternMsg & msg)
  : msg_(msg)
  {}
  Init_IkPatternMsg_srpattern zrpattern(::humanoid_interfaces::msg::IkPatternMsg::_zrpattern_type arg)
  {
    msg_.zrpattern = std::move(arg);
    return Init_IkPatternMsg_srpattern(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkPatternMsg msg_;
};

class Init_IkPatternMsg_yrpattern
{
public:
  explicit Init_IkPatternMsg_yrpattern(::humanoid_interfaces::msg::IkPatternMsg & msg)
  : msg_(msg)
  {}
  Init_IkPatternMsg_zrpattern yrpattern(::humanoid_interfaces::msg::IkPatternMsg::_yrpattern_type arg)
  {
    msg_.yrpattern = std::move(arg);
    return Init_IkPatternMsg_zrpattern(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkPatternMsg msg_;
};

class Init_IkPatternMsg_xrpattern
{
public:
  explicit Init_IkPatternMsg_xrpattern(::humanoid_interfaces::msg::IkPatternMsg & msg)
  : msg_(msg)
  {}
  Init_IkPatternMsg_yrpattern xrpattern(::humanoid_interfaces::msg::IkPatternMsg::_xrpattern_type arg)
  {
    msg_.xrpattern = std::move(arg);
    return Init_IkPatternMsg_yrpattern(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkPatternMsg msg_;
};

class Init_IkPatternMsg_tlpattern
{
public:
  explicit Init_IkPatternMsg_tlpattern(::humanoid_interfaces::msg::IkPatternMsg & msg)
  : msg_(msg)
  {}
  Init_IkPatternMsg_xrpattern tlpattern(::humanoid_interfaces::msg::IkPatternMsg::_tlpattern_type arg)
  {
    msg_.tlpattern = std::move(arg);
    return Init_IkPatternMsg_xrpattern(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkPatternMsg msg_;
};

class Init_IkPatternMsg_slpattern
{
public:
  explicit Init_IkPatternMsg_slpattern(::humanoid_interfaces::msg::IkPatternMsg & msg)
  : msg_(msg)
  {}
  Init_IkPatternMsg_tlpattern slpattern(::humanoid_interfaces::msg::IkPatternMsg::_slpattern_type arg)
  {
    msg_.slpattern = std::move(arg);
    return Init_IkPatternMsg_tlpattern(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkPatternMsg msg_;
};

class Init_IkPatternMsg_zlpattern
{
public:
  explicit Init_IkPatternMsg_zlpattern(::humanoid_interfaces::msg::IkPatternMsg & msg)
  : msg_(msg)
  {}
  Init_IkPatternMsg_slpattern zlpattern(::humanoid_interfaces::msg::IkPatternMsg::_zlpattern_type arg)
  {
    msg_.zlpattern = std::move(arg);
    return Init_IkPatternMsg_slpattern(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkPatternMsg msg_;
};

class Init_IkPatternMsg_ylpattern
{
public:
  explicit Init_IkPatternMsg_ylpattern(::humanoid_interfaces::msg::IkPatternMsg & msg)
  : msg_(msg)
  {}
  Init_IkPatternMsg_zlpattern ylpattern(::humanoid_interfaces::msg::IkPatternMsg::_ylpattern_type arg)
  {
    msg_.ylpattern = std::move(arg);
    return Init_IkPatternMsg_zlpattern(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkPatternMsg msg_;
};

class Init_IkPatternMsg_xlpattern
{
public:
  Init_IkPatternMsg_xlpattern()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IkPatternMsg_ylpattern xlpattern(::humanoid_interfaces::msg::IkPatternMsg::_xlpattern_type arg)
  {
    msg_.xlpattern = std::move(arg);
    return Init_IkPatternMsg_ylpattern(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkPatternMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::IkPatternMsg>()
{
  return humanoid_interfaces::msg::builder::Init_IkPatternMsg_xlpattern();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_PATTERN_MSG__BUILDER_HPP_
