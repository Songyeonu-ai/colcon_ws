// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/IkLTCMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_LTC_MSG__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_LTC_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/ik_ltc_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_IkLTCMsg_landing_error_r
{
public:
  explicit Init_IkLTCMsg_landing_error_r(::humanoid_interfaces::msg::IkLTCMsg & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::IkLTCMsg landing_error_r(::humanoid_interfaces::msg::IkLTCMsg::_landing_error_r_type arg)
  {
    msg_.landing_error_r = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkLTCMsg msg_;
};

class Init_IkLTCMsg_landing_error_l
{
public:
  explicit Init_IkLTCMsg_landing_error_l(::humanoid_interfaces::msg::IkLTCMsg & msg)
  : msg_(msg)
  {}
  Init_IkLTCMsg_landing_error_r landing_error_l(::humanoid_interfaces::msg::IkLTCMsg::_landing_error_l_type arg)
  {
    msg_.landing_error_l = std::move(arg);
    return Init_IkLTCMsg_landing_error_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkLTCMsg msg_;
};

class Init_IkLTCMsg_landing_time_r
{
public:
  explicit Init_IkLTCMsg_landing_time_r(::humanoid_interfaces::msg::IkLTCMsg & msg)
  : msg_(msg)
  {}
  Init_IkLTCMsg_landing_error_l landing_time_r(::humanoid_interfaces::msg::IkLTCMsg::_landing_time_r_type arg)
  {
    msg_.landing_time_r = std::move(arg);
    return Init_IkLTCMsg_landing_error_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkLTCMsg msg_;
};

class Init_IkLTCMsg_landing_time_l
{
public:
  explicit Init_IkLTCMsg_landing_time_l(::humanoid_interfaces::msg::IkLTCMsg & msg)
  : msg_(msg)
  {}
  Init_IkLTCMsg_landing_time_r landing_time_l(::humanoid_interfaces::msg::IkLTCMsg::_landing_time_l_type arg)
  {
    msg_.landing_time_l = std::move(arg);
    return Init_IkLTCMsg_landing_time_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkLTCMsg msg_;
};

class Init_IkLTCMsg_safe
{
public:
  explicit Init_IkLTCMsg_safe(::humanoid_interfaces::msg::IkLTCMsg & msg)
  : msg_(msg)
  {}
  Init_IkLTCMsg_landing_time_l safe(::humanoid_interfaces::msg::IkLTCMsg::_safe_type arg)
  {
    msg_.safe = std::move(arg);
    return Init_IkLTCMsg_landing_time_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkLTCMsg msg_;
};

class Init_IkLTCMsg_warning
{
public:
  explicit Init_IkLTCMsg_warning(::humanoid_interfaces::msg::IkLTCMsg & msg)
  : msg_(msg)
  {}
  Init_IkLTCMsg_safe warning(::humanoid_interfaces::msg::IkLTCMsg::_warning_type arg)
  {
    msg_.warning = std::move(arg);
    return Init_IkLTCMsg_safe(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkLTCMsg msg_;
};

class Init_IkLTCMsg_swing_gain_r
{
public:
  explicit Init_IkLTCMsg_swing_gain_r(::humanoid_interfaces::msg::IkLTCMsg & msg)
  : msg_(msg)
  {}
  Init_IkLTCMsg_warning swing_gain_r(::humanoid_interfaces::msg::IkLTCMsg::_swing_gain_r_type arg)
  {
    msg_.swing_gain_r = std::move(arg);
    return Init_IkLTCMsg_warning(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkLTCMsg msg_;
};

class Init_IkLTCMsg_swing_gain_l
{
public:
  explicit Init_IkLTCMsg_swing_gain_l(::humanoid_interfaces::msg::IkLTCMsg & msg)
  : msg_(msg)
  {}
  Init_IkLTCMsg_swing_gain_r swing_gain_l(::humanoid_interfaces::msg::IkLTCMsg::_swing_gain_l_type arg)
  {
    msg_.swing_gain_l = std::move(arg);
    return Init_IkLTCMsg_swing_gain_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkLTCMsg msg_;
};

class Init_IkLTCMsg_entire_time
{
public:
  Init_IkLTCMsg_entire_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IkLTCMsg_swing_gain_l entire_time(::humanoid_interfaces::msg::IkLTCMsg::_entire_time_type arg)
  {
    msg_.entire_time = std::move(arg);
    return Init_IkLTCMsg_swing_gain_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::IkLTCMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::IkLTCMsg>()
{
  return humanoid_interfaces::msg::builder::Init_IkLTCMsg_entire_time();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_LTC_MSG__BUILDER_HPP_
