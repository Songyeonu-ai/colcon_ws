// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/Tune2IkMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__TUNE2_IK_MSG__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__TUNE2_IK_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/tune2_ik_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_Tune2IkMsg_control_system_flag
{
public:
  explicit Init_Tune2IkMsg_control_system_flag(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::Tune2IkMsg control_system_flag(::humanoid_interfaces::msg::Tune2IkMsg::_control_system_flag_type arg)
  {
    msg_.control_system_flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_landing_time_control_flag
{
public:
  explicit Init_Tune2IkMsg_landing_time_control_flag(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_control_system_flag landing_time_control_flag(::humanoid_interfaces::msg::Tune2IkMsg::_landing_time_control_flag_type arg)
  {
    msg_.landing_time_control_flag = std::move(arg);
    return Init_Tune2IkMsg_control_system_flag(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_max
{
public:
  explicit Init_Tune2IkMsg_fourth_max(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_landing_time_control_flag fourth_max(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_max_type arg)
  {
    msg_.fourth_max = std::move(arg);
    return Init_Tune2IkMsg_landing_time_control_flag(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_min
{
public:
  explicit Init_Tune2IkMsg_fourth_min(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_max fourth_min(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_min_type arg)
  {
    msg_.fourth_min = std::move(arg);
    return Init_Tune2IkMsg_fourth_max(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_neg_side_l_swing_minus
{
public:
  explicit Init_Tune2IkMsg_fourth_neg_side_l_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_min fourth_neg_side_l_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_neg_side_l_swing_minus_type arg)
  {
    msg_.fourth_neg_side_l_swing_minus = std::move(arg);
    return Init_Tune2IkMsg_fourth_min(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_pos_side_l_swing_minus
{
public:
  explicit Init_Tune2IkMsg_fourth_pos_side_l_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_neg_side_l_swing_minus fourth_pos_side_l_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_pos_side_l_swing_minus_type arg)
  {
    msg_.fourth_pos_side_l_swing_minus = std::move(arg);
    return Init_Tune2IkMsg_fourth_neg_side_l_swing_minus(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_neg_side_r_swing_minus
{
public:
  explicit Init_Tune2IkMsg_fourth_neg_side_r_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_pos_side_l_swing_minus fourth_neg_side_r_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_neg_side_r_swing_minus_type arg)
  {
    msg_.fourth_neg_side_r_swing_minus = std::move(arg);
    return Init_Tune2IkMsg_fourth_pos_side_l_swing_minus(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_pos_side_r_swing_minus
{
public:
  explicit Init_Tune2IkMsg_fourth_pos_side_r_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_neg_side_r_swing_minus fourth_pos_side_r_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_pos_side_r_swing_minus_type arg)
  {
    msg_.fourth_pos_side_r_swing_minus = std::move(arg);
    return Init_Tune2IkMsg_fourth_neg_side_r_swing_minus(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_neg_yaw_l
{
public:
  explicit Init_Tune2IkMsg_fourth_neg_yaw_l(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_pos_side_r_swing_minus fourth_neg_yaw_l(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_neg_yaw_l_type arg)
  {
    msg_.fourth_neg_yaw_l = std::move(arg);
    return Init_Tune2IkMsg_fourth_pos_side_r_swing_minus(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_pos_yaw_l
{
public:
  explicit Init_Tune2IkMsg_fourth_pos_yaw_l(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_neg_yaw_l fourth_pos_yaw_l(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_pos_yaw_l_type arg)
  {
    msg_.fourth_pos_yaw_l = std::move(arg);
    return Init_Tune2IkMsg_fourth_neg_yaw_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_neg_side_l
{
public:
  explicit Init_Tune2IkMsg_fourth_neg_side_l(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_pos_yaw_l fourth_neg_side_l(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_neg_side_l_type arg)
  {
    msg_.fourth_neg_side_l = std::move(arg);
    return Init_Tune2IkMsg_fourth_pos_yaw_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_pos_side_l
{
public:
  explicit Init_Tune2IkMsg_fourth_pos_side_l(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_neg_side_l fourth_pos_side_l(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_pos_side_l_type arg)
  {
    msg_.fourth_pos_side_l = std::move(arg);
    return Init_Tune2IkMsg_fourth_neg_side_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_neg_xl
{
public:
  explicit Init_Tune2IkMsg_fourth_neg_xl(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_pos_side_l fourth_neg_xl(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_neg_xl_type arg)
  {
    msg_.fourth_neg_xl = std::move(arg);
    return Init_Tune2IkMsg_fourth_pos_side_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_pos_xl
{
public:
  explicit Init_Tune2IkMsg_fourth_pos_xl(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_neg_xl fourth_pos_xl(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_pos_xl_type arg)
  {
    msg_.fourth_pos_xl = std::move(arg);
    return Init_Tune2IkMsg_fourth_neg_xl(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_neg_yaw_r
{
public:
  explicit Init_Tune2IkMsg_fourth_neg_yaw_r(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_pos_xl fourth_neg_yaw_r(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_neg_yaw_r_type arg)
  {
    msg_.fourth_neg_yaw_r = std::move(arg);
    return Init_Tune2IkMsg_fourth_pos_xl(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_pos_yaw_r
{
public:
  explicit Init_Tune2IkMsg_fourth_pos_yaw_r(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_neg_yaw_r fourth_pos_yaw_r(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_pos_yaw_r_type arg)
  {
    msg_.fourth_pos_yaw_r = std::move(arg);
    return Init_Tune2IkMsg_fourth_neg_yaw_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_neg_side_r
{
public:
  explicit Init_Tune2IkMsg_fourth_neg_side_r(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_pos_yaw_r fourth_neg_side_r(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_neg_side_r_type arg)
  {
    msg_.fourth_neg_side_r = std::move(arg);
    return Init_Tune2IkMsg_fourth_pos_yaw_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_pos_side_r
{
public:
  explicit Init_Tune2IkMsg_fourth_pos_side_r(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_neg_side_r fourth_pos_side_r(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_pos_side_r_type arg)
  {
    msg_.fourth_pos_side_r = std::move(arg);
    return Init_Tune2IkMsg_fourth_neg_side_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_neg_xr
{
public:
  explicit Init_Tune2IkMsg_fourth_neg_xr(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_pos_side_r fourth_neg_xr(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_neg_xr_type arg)
  {
    msg_.fourth_neg_xr = std::move(arg);
    return Init_Tune2IkMsg_fourth_pos_side_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_fourth_pos_xr
{
public:
  explicit Init_Tune2IkMsg_fourth_pos_xr(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_neg_xr fourth_pos_xr(::humanoid_interfaces::msg::Tune2IkMsg::_fourth_pos_xr_type arg)
  {
    msg_.fourth_pos_xr = std::move(arg);
    return Init_Tune2IkMsg_fourth_neg_xr(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_max
{
public:
  explicit Init_Tune2IkMsg_third_max(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_fourth_pos_xr third_max(::humanoid_interfaces::msg::Tune2IkMsg::_third_max_type arg)
  {
    msg_.third_max = std::move(arg);
    return Init_Tune2IkMsg_fourth_pos_xr(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_min
{
public:
  explicit Init_Tune2IkMsg_third_min(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_max third_min(::humanoid_interfaces::msg::Tune2IkMsg::_third_min_type arg)
  {
    msg_.third_min = std::move(arg);
    return Init_Tune2IkMsg_third_max(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_neg_side_l_swing_minus
{
public:
  explicit Init_Tune2IkMsg_third_neg_side_l_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_min third_neg_side_l_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg::_third_neg_side_l_swing_minus_type arg)
  {
    msg_.third_neg_side_l_swing_minus = std::move(arg);
    return Init_Tune2IkMsg_third_min(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_pos_side_l_swing_minus
{
public:
  explicit Init_Tune2IkMsg_third_pos_side_l_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_neg_side_l_swing_minus third_pos_side_l_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg::_third_pos_side_l_swing_minus_type arg)
  {
    msg_.third_pos_side_l_swing_minus = std::move(arg);
    return Init_Tune2IkMsg_third_neg_side_l_swing_minus(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_neg_side_r_swing_minus
{
public:
  explicit Init_Tune2IkMsg_third_neg_side_r_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_pos_side_l_swing_minus third_neg_side_r_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg::_third_neg_side_r_swing_minus_type arg)
  {
    msg_.third_neg_side_r_swing_minus = std::move(arg);
    return Init_Tune2IkMsg_third_pos_side_l_swing_minus(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_pos_side_r_swing_minus
{
public:
  explicit Init_Tune2IkMsg_third_pos_side_r_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_neg_side_r_swing_minus third_pos_side_r_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg::_third_pos_side_r_swing_minus_type arg)
  {
    msg_.third_pos_side_r_swing_minus = std::move(arg);
    return Init_Tune2IkMsg_third_neg_side_r_swing_minus(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_neg_yaw_l
{
public:
  explicit Init_Tune2IkMsg_third_neg_yaw_l(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_pos_side_r_swing_minus third_neg_yaw_l(::humanoid_interfaces::msg::Tune2IkMsg::_third_neg_yaw_l_type arg)
  {
    msg_.third_neg_yaw_l = std::move(arg);
    return Init_Tune2IkMsg_third_pos_side_r_swing_minus(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_pos_yaw_l
{
public:
  explicit Init_Tune2IkMsg_third_pos_yaw_l(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_neg_yaw_l third_pos_yaw_l(::humanoid_interfaces::msg::Tune2IkMsg::_third_pos_yaw_l_type arg)
  {
    msg_.third_pos_yaw_l = std::move(arg);
    return Init_Tune2IkMsg_third_neg_yaw_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_neg_side_l
{
public:
  explicit Init_Tune2IkMsg_third_neg_side_l(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_pos_yaw_l third_neg_side_l(::humanoid_interfaces::msg::Tune2IkMsg::_third_neg_side_l_type arg)
  {
    msg_.third_neg_side_l = std::move(arg);
    return Init_Tune2IkMsg_third_pos_yaw_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_pos_side_l
{
public:
  explicit Init_Tune2IkMsg_third_pos_side_l(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_neg_side_l third_pos_side_l(::humanoid_interfaces::msg::Tune2IkMsg::_third_pos_side_l_type arg)
  {
    msg_.third_pos_side_l = std::move(arg);
    return Init_Tune2IkMsg_third_neg_side_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_neg_xl
{
public:
  explicit Init_Tune2IkMsg_third_neg_xl(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_pos_side_l third_neg_xl(::humanoid_interfaces::msg::Tune2IkMsg::_third_neg_xl_type arg)
  {
    msg_.third_neg_xl = std::move(arg);
    return Init_Tune2IkMsg_third_pos_side_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_pos_xl
{
public:
  explicit Init_Tune2IkMsg_third_pos_xl(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_neg_xl third_pos_xl(::humanoid_interfaces::msg::Tune2IkMsg::_third_pos_xl_type arg)
  {
    msg_.third_pos_xl = std::move(arg);
    return Init_Tune2IkMsg_third_neg_xl(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_neg_yaw_r
{
public:
  explicit Init_Tune2IkMsg_third_neg_yaw_r(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_pos_xl third_neg_yaw_r(::humanoid_interfaces::msg::Tune2IkMsg::_third_neg_yaw_r_type arg)
  {
    msg_.third_neg_yaw_r = std::move(arg);
    return Init_Tune2IkMsg_third_pos_xl(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_pos_yaw_r
{
public:
  explicit Init_Tune2IkMsg_third_pos_yaw_r(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_neg_yaw_r third_pos_yaw_r(::humanoid_interfaces::msg::Tune2IkMsg::_third_pos_yaw_r_type arg)
  {
    msg_.third_pos_yaw_r = std::move(arg);
    return Init_Tune2IkMsg_third_neg_yaw_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_neg_side_r
{
public:
  explicit Init_Tune2IkMsg_third_neg_side_r(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_pos_yaw_r third_neg_side_r(::humanoid_interfaces::msg::Tune2IkMsg::_third_neg_side_r_type arg)
  {
    msg_.third_neg_side_r = std::move(arg);
    return Init_Tune2IkMsg_third_pos_yaw_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_pos_side_r
{
public:
  explicit Init_Tune2IkMsg_third_pos_side_r(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_neg_side_r third_pos_side_r(::humanoid_interfaces::msg::Tune2IkMsg::_third_pos_side_r_type arg)
  {
    msg_.third_pos_side_r = std::move(arg);
    return Init_Tune2IkMsg_third_neg_side_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_neg_xr
{
public:
  explicit Init_Tune2IkMsg_third_neg_xr(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_pos_side_r third_neg_xr(::humanoid_interfaces::msg::Tune2IkMsg::_third_neg_xr_type arg)
  {
    msg_.third_neg_xr = std::move(arg);
    return Init_Tune2IkMsg_third_pos_side_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_third_pos_xr
{
public:
  explicit Init_Tune2IkMsg_third_pos_xr(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_neg_xr third_pos_xr(::humanoid_interfaces::msg::Tune2IkMsg::_third_pos_xr_type arg)
  {
    msg_.third_pos_xr = std::move(arg);
    return Init_Tune2IkMsg_third_neg_xr(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_max
{
public:
  explicit Init_Tune2IkMsg_second_max(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_third_pos_xr second_max(::humanoid_interfaces::msg::Tune2IkMsg::_second_max_type arg)
  {
    msg_.second_max = std::move(arg);
    return Init_Tune2IkMsg_third_pos_xr(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_min
{
public:
  explicit Init_Tune2IkMsg_second_min(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_max second_min(::humanoid_interfaces::msg::Tune2IkMsg::_second_min_type arg)
  {
    msg_.second_min = std::move(arg);
    return Init_Tune2IkMsg_second_max(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_neg_side_l_swing_minus
{
public:
  explicit Init_Tune2IkMsg_second_neg_side_l_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_min second_neg_side_l_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg::_second_neg_side_l_swing_minus_type arg)
  {
    msg_.second_neg_side_l_swing_minus = std::move(arg);
    return Init_Tune2IkMsg_second_min(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_pos_side_l_swing_minus
{
public:
  explicit Init_Tune2IkMsg_second_pos_side_l_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_neg_side_l_swing_minus second_pos_side_l_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg::_second_pos_side_l_swing_minus_type arg)
  {
    msg_.second_pos_side_l_swing_minus = std::move(arg);
    return Init_Tune2IkMsg_second_neg_side_l_swing_minus(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_neg_side_r_swing_minus
{
public:
  explicit Init_Tune2IkMsg_second_neg_side_r_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_pos_side_l_swing_minus second_neg_side_r_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg::_second_neg_side_r_swing_minus_type arg)
  {
    msg_.second_neg_side_r_swing_minus = std::move(arg);
    return Init_Tune2IkMsg_second_pos_side_l_swing_minus(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_pos_side_r_swing_minus
{
public:
  explicit Init_Tune2IkMsg_second_pos_side_r_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_neg_side_r_swing_minus second_pos_side_r_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg::_second_pos_side_r_swing_minus_type arg)
  {
    msg_.second_pos_side_r_swing_minus = std::move(arg);
    return Init_Tune2IkMsg_second_neg_side_r_swing_minus(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_neg_yaw_l
{
public:
  explicit Init_Tune2IkMsg_second_neg_yaw_l(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_pos_side_r_swing_minus second_neg_yaw_l(::humanoid_interfaces::msg::Tune2IkMsg::_second_neg_yaw_l_type arg)
  {
    msg_.second_neg_yaw_l = std::move(arg);
    return Init_Tune2IkMsg_second_pos_side_r_swing_minus(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_pos_yaw_l
{
public:
  explicit Init_Tune2IkMsg_second_pos_yaw_l(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_neg_yaw_l second_pos_yaw_l(::humanoid_interfaces::msg::Tune2IkMsg::_second_pos_yaw_l_type arg)
  {
    msg_.second_pos_yaw_l = std::move(arg);
    return Init_Tune2IkMsg_second_neg_yaw_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_neg_side_l
{
public:
  explicit Init_Tune2IkMsg_second_neg_side_l(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_pos_yaw_l second_neg_side_l(::humanoid_interfaces::msg::Tune2IkMsg::_second_neg_side_l_type arg)
  {
    msg_.second_neg_side_l = std::move(arg);
    return Init_Tune2IkMsg_second_pos_yaw_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_pos_side_l
{
public:
  explicit Init_Tune2IkMsg_second_pos_side_l(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_neg_side_l second_pos_side_l(::humanoid_interfaces::msg::Tune2IkMsg::_second_pos_side_l_type arg)
  {
    msg_.second_pos_side_l = std::move(arg);
    return Init_Tune2IkMsg_second_neg_side_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_neg_xl
{
public:
  explicit Init_Tune2IkMsg_second_neg_xl(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_pos_side_l second_neg_xl(::humanoid_interfaces::msg::Tune2IkMsg::_second_neg_xl_type arg)
  {
    msg_.second_neg_xl = std::move(arg);
    return Init_Tune2IkMsg_second_pos_side_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_pos_xl
{
public:
  explicit Init_Tune2IkMsg_second_pos_xl(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_neg_xl second_pos_xl(::humanoid_interfaces::msg::Tune2IkMsg::_second_pos_xl_type arg)
  {
    msg_.second_pos_xl = std::move(arg);
    return Init_Tune2IkMsg_second_neg_xl(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_neg_yaw_r
{
public:
  explicit Init_Tune2IkMsg_second_neg_yaw_r(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_pos_xl second_neg_yaw_r(::humanoid_interfaces::msg::Tune2IkMsg::_second_neg_yaw_r_type arg)
  {
    msg_.second_neg_yaw_r = std::move(arg);
    return Init_Tune2IkMsg_second_pos_xl(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_pos_yaw_r
{
public:
  explicit Init_Tune2IkMsg_second_pos_yaw_r(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_neg_yaw_r second_pos_yaw_r(::humanoid_interfaces::msg::Tune2IkMsg::_second_pos_yaw_r_type arg)
  {
    msg_.second_pos_yaw_r = std::move(arg);
    return Init_Tune2IkMsg_second_neg_yaw_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_neg_side_r
{
public:
  explicit Init_Tune2IkMsg_second_neg_side_r(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_pos_yaw_r second_neg_side_r(::humanoid_interfaces::msg::Tune2IkMsg::_second_neg_side_r_type arg)
  {
    msg_.second_neg_side_r = std::move(arg);
    return Init_Tune2IkMsg_second_pos_yaw_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_pos_side_r
{
public:
  explicit Init_Tune2IkMsg_second_pos_side_r(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_neg_side_r second_pos_side_r(::humanoid_interfaces::msg::Tune2IkMsg::_second_pos_side_r_type arg)
  {
    msg_.second_pos_side_r = std::move(arg);
    return Init_Tune2IkMsg_second_neg_side_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_neg_xr
{
public:
  explicit Init_Tune2IkMsg_second_neg_xr(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_pos_side_r second_neg_xr(::humanoid_interfaces::msg::Tune2IkMsg::_second_neg_xr_type arg)
  {
    msg_.second_neg_xr = std::move(arg);
    return Init_Tune2IkMsg_second_pos_side_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_second_pos_xr
{
public:
  explicit Init_Tune2IkMsg_second_pos_xr(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_neg_xr second_pos_xr(::humanoid_interfaces::msg::Tune2IkMsg::_second_pos_xr_type arg)
  {
    msg_.second_pos_xr = std::move(arg);
    return Init_Tune2IkMsg_second_neg_xr(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_max
{
public:
  explicit Init_Tune2IkMsg_first_max(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_second_pos_xr first_max(::humanoid_interfaces::msg::Tune2IkMsg::_first_max_type arg)
  {
    msg_.first_max = std::move(arg);
    return Init_Tune2IkMsg_second_pos_xr(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_min
{
public:
  explicit Init_Tune2IkMsg_first_min(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_max first_min(::humanoid_interfaces::msg::Tune2IkMsg::_first_min_type arg)
  {
    msg_.first_min = std::move(arg);
    return Init_Tune2IkMsg_first_max(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_neg_side_l_swing_minus
{
public:
  explicit Init_Tune2IkMsg_first_neg_side_l_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_min first_neg_side_l_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg::_first_neg_side_l_swing_minus_type arg)
  {
    msg_.first_neg_side_l_swing_minus = std::move(arg);
    return Init_Tune2IkMsg_first_min(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_pos_side_l_swing_minus
{
public:
  explicit Init_Tune2IkMsg_first_pos_side_l_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_neg_side_l_swing_minus first_pos_side_l_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg::_first_pos_side_l_swing_minus_type arg)
  {
    msg_.first_pos_side_l_swing_minus = std::move(arg);
    return Init_Tune2IkMsg_first_neg_side_l_swing_minus(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_neg_side_r_swing_minus
{
public:
  explicit Init_Tune2IkMsg_first_neg_side_r_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_pos_side_l_swing_minus first_neg_side_r_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg::_first_neg_side_r_swing_minus_type arg)
  {
    msg_.first_neg_side_r_swing_minus = std::move(arg);
    return Init_Tune2IkMsg_first_pos_side_l_swing_minus(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_pos_side_r_swing_minus
{
public:
  explicit Init_Tune2IkMsg_first_pos_side_r_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_neg_side_r_swing_minus first_pos_side_r_swing_minus(::humanoid_interfaces::msg::Tune2IkMsg::_first_pos_side_r_swing_minus_type arg)
  {
    msg_.first_pos_side_r_swing_minus = std::move(arg);
    return Init_Tune2IkMsg_first_neg_side_r_swing_minus(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_neg_yaw_l
{
public:
  explicit Init_Tune2IkMsg_first_neg_yaw_l(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_pos_side_r_swing_minus first_neg_yaw_l(::humanoid_interfaces::msg::Tune2IkMsg::_first_neg_yaw_l_type arg)
  {
    msg_.first_neg_yaw_l = std::move(arg);
    return Init_Tune2IkMsg_first_pos_side_r_swing_minus(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_pos_yaw_l
{
public:
  explicit Init_Tune2IkMsg_first_pos_yaw_l(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_neg_yaw_l first_pos_yaw_l(::humanoid_interfaces::msg::Tune2IkMsg::_first_pos_yaw_l_type arg)
  {
    msg_.first_pos_yaw_l = std::move(arg);
    return Init_Tune2IkMsg_first_neg_yaw_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_neg_side_l
{
public:
  explicit Init_Tune2IkMsg_first_neg_side_l(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_pos_yaw_l first_neg_side_l(::humanoid_interfaces::msg::Tune2IkMsg::_first_neg_side_l_type arg)
  {
    msg_.first_neg_side_l = std::move(arg);
    return Init_Tune2IkMsg_first_pos_yaw_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_pos_side_l
{
public:
  explicit Init_Tune2IkMsg_first_pos_side_l(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_neg_side_l first_pos_side_l(::humanoid_interfaces::msg::Tune2IkMsg::_first_pos_side_l_type arg)
  {
    msg_.first_pos_side_l = std::move(arg);
    return Init_Tune2IkMsg_first_neg_side_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_neg_xl
{
public:
  explicit Init_Tune2IkMsg_first_neg_xl(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_pos_side_l first_neg_xl(::humanoid_interfaces::msg::Tune2IkMsg::_first_neg_xl_type arg)
  {
    msg_.first_neg_xl = std::move(arg);
    return Init_Tune2IkMsg_first_pos_side_l(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_pos_xl
{
public:
  explicit Init_Tune2IkMsg_first_pos_xl(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_neg_xl first_pos_xl(::humanoid_interfaces::msg::Tune2IkMsg::_first_pos_xl_type arg)
  {
    msg_.first_pos_xl = std::move(arg);
    return Init_Tune2IkMsg_first_neg_xl(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_neg_yaw_r
{
public:
  explicit Init_Tune2IkMsg_first_neg_yaw_r(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_pos_xl first_neg_yaw_r(::humanoid_interfaces::msg::Tune2IkMsg::_first_neg_yaw_r_type arg)
  {
    msg_.first_neg_yaw_r = std::move(arg);
    return Init_Tune2IkMsg_first_pos_xl(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_pos_yaw_r
{
public:
  explicit Init_Tune2IkMsg_first_pos_yaw_r(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_neg_yaw_r first_pos_yaw_r(::humanoid_interfaces::msg::Tune2IkMsg::_first_pos_yaw_r_type arg)
  {
    msg_.first_pos_yaw_r = std::move(arg);
    return Init_Tune2IkMsg_first_neg_yaw_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_neg_side_r
{
public:
  explicit Init_Tune2IkMsg_first_neg_side_r(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_pos_yaw_r first_neg_side_r(::humanoid_interfaces::msg::Tune2IkMsg::_first_neg_side_r_type arg)
  {
    msg_.first_neg_side_r = std::move(arg);
    return Init_Tune2IkMsg_first_pos_yaw_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_pos_side_r
{
public:
  explicit Init_Tune2IkMsg_first_pos_side_r(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_neg_side_r first_pos_side_r(::humanoid_interfaces::msg::Tune2IkMsg::_first_pos_side_r_type arg)
  {
    msg_.first_pos_side_r = std::move(arg);
    return Init_Tune2IkMsg_first_neg_side_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_neg_xr
{
public:
  explicit Init_Tune2IkMsg_first_neg_xr(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_pos_side_r first_neg_xr(::humanoid_interfaces::msg::Tune2IkMsg::_first_neg_xr_type arg)
  {
    msg_.first_neg_xr = std::move(arg);
    return Init_Tune2IkMsg_first_pos_side_r(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_first_pos_xr
{
public:
  explicit Init_Tune2IkMsg_first_pos_xr(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_neg_xr first_pos_xr(::humanoid_interfaces::msg::Tune2IkMsg::_first_pos_xr_type arg)
  {
    msg_.first_pos_xr = std::move(arg);
    return Init_Tune2IkMsg_first_neg_xr(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_percentage_of_ik_motor
{
public:
  explicit Init_Tune2IkMsg_percentage_of_ik_motor(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_first_pos_xr percentage_of_ik_motor(::humanoid_interfaces::msg::Tune2IkMsg::_percentage_of_ik_motor_type arg)
  {
    msg_.percentage_of_ik_motor = std::move(arg);
    return Init_Tune2IkMsg_first_pos_xr(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_init_z_up
{
public:
  explicit Init_Tune2IkMsg_init_z_up(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_percentage_of_ik_motor init_z_up(::humanoid_interfaces::msg::Tune2IkMsg::_init_z_up_type arg)
  {
    msg_.init_z_up = std::move(arg);
    return Init_Tune2IkMsg_percentage_of_ik_motor(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_link2link
{
public:
  explicit Init_Tune2IkMsg_link2link(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_init_z_up link2link(::humanoid_interfaces::msg::Tune2IkMsg::_link2link_type arg)
  {
    msg_.link2link = std::move(arg);
    return Init_Tune2IkMsg_init_z_up(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_center2leg
{
public:
  explicit Init_Tune2IkMsg_center2leg(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_link2link center2leg(::humanoid_interfaces::msg::Tune2IkMsg::_center2leg_type arg)
  {
    msg_.center2leg = std::move(arg);
    return Init_Tune2IkMsg_link2link(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_ratio_check_flag
{
public:
  explicit Init_Tune2IkMsg_ratio_check_flag(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_center2leg ratio_check_flag(::humanoid_interfaces::msg::Tune2IkMsg::_ratio_check_flag_type arg)
  {
    msg_.ratio_check_flag = std::move(arg);
    return Init_Tune2IkMsg_center2leg(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_roll_flag
{
public:
  explicit Init_Tune2IkMsg_balance_roll_flag(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_ratio_check_flag balance_roll_flag(::humanoid_interfaces::msg::Tune2IkMsg::_balance_roll_flag_type arg)
  {
    msg_.balance_roll_flag = std::move(arg);
    return Init_Tune2IkMsg_ratio_check_flag(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_ankle_pitch_flag
{
public:
  explicit Init_Tune2IkMsg_balance_ankle_pitch_flag(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_roll_flag balance_ankle_pitch_flag(::humanoid_interfaces::msg::Tune2IkMsg::_balance_ankle_pitch_flag_type arg)
  {
    msg_.balance_ankle_pitch_flag = std::move(arg);
    return Init_Tune2IkMsg_balance_roll_flag(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_pitch_flag
{
public:
  explicit Init_Tune2IkMsg_balance_pitch_flag(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_ankle_pitch_flag balance_pitch_flag(::humanoid_interfaces::msg::Tune2IkMsg::_balance_pitch_flag_type arg)
  {
    msg_.balance_pitch_flag = std::move(arg);
    return Init_Tune2IkMsg_balance_ankle_pitch_flag(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_roll_olimit_imu
{
public:
  explicit Init_Tune2IkMsg_balance_roll_olimit_imu(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_pitch_flag balance_roll_olimit_imu(::humanoid_interfaces::msg::Tune2IkMsg::_balance_roll_olimit_imu_type arg)
  {
    msg_.balance_roll_olimit_imu = std::move(arg);
    return Init_Tune2IkMsg_balance_pitch_flag(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_roll_elimit_imu
{
public:
  explicit Init_Tune2IkMsg_balance_roll_elimit_imu(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_roll_olimit_imu balance_roll_elimit_imu(::humanoid_interfaces::msg::Tune2IkMsg::_balance_roll_elimit_imu_type arg)
  {
    msg_.balance_roll_elimit_imu = std::move(arg);
    return Init_Tune2IkMsg_balance_roll_olimit_imu(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_roll_pos_target_imu
{
public:
  explicit Init_Tune2IkMsg_balance_roll_pos_target_imu(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_roll_elimit_imu balance_roll_pos_target_imu(::humanoid_interfaces::msg::Tune2IkMsg::_balance_roll_pos_target_imu_type arg)
  {
    msg_.balance_roll_pos_target_imu = std::move(arg);
    return Init_Tune2IkMsg_balance_roll_elimit_imu(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_roll_neg_target_imu
{
public:
  explicit Init_Tune2IkMsg_balance_roll_neg_target_imu(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_roll_pos_target_imu balance_roll_neg_target_imu(::humanoid_interfaces::msg::Tune2IkMsg::_balance_roll_neg_target_imu_type arg)
  {
    msg_.balance_roll_neg_target_imu = std::move(arg);
    return Init_Tune2IkMsg_balance_roll_pos_target_imu(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_roll_gd_imu
{
public:
  explicit Init_Tune2IkMsg_balance_roll_gd_imu(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_roll_neg_target_imu balance_roll_gd_imu(::humanoid_interfaces::msg::Tune2IkMsg::_balance_roll_gd_imu_type arg)
  {
    msg_.balance_roll_gd_imu = std::move(arg);
    return Init_Tune2IkMsg_balance_roll_neg_target_imu(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_roll_gi_imu
{
public:
  explicit Init_Tune2IkMsg_balance_roll_gi_imu(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_roll_gd_imu balance_roll_gi_imu(::humanoid_interfaces::msg::Tune2IkMsg::_balance_roll_gi_imu_type arg)
  {
    msg_.balance_roll_gi_imu = std::move(arg);
    return Init_Tune2IkMsg_balance_roll_gd_imu(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_roll_gp_imu
{
public:
  explicit Init_Tune2IkMsg_balance_roll_gp_imu(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_roll_gi_imu balance_roll_gp_imu(::humanoid_interfaces::msg::Tune2IkMsg::_balance_roll_gp_imu_type arg)
  {
    msg_.balance_roll_gp_imu = std::move(arg);
    return Init_Tune2IkMsg_balance_roll_gi_imu(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_roll_flag_imu
{
public:
  explicit Init_Tune2IkMsg_balance_roll_flag_imu(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_roll_gp_imu balance_roll_flag_imu(::humanoid_interfaces::msg::Tune2IkMsg::_balance_roll_flag_imu_type arg)
  {
    msg_.balance_roll_flag_imu = std::move(arg);
    return Init_Tune2IkMsg_balance_roll_gp_imu(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_pitch_olimit_imu
{
public:
  explicit Init_Tune2IkMsg_balance_pitch_olimit_imu(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_roll_flag_imu balance_pitch_olimit_imu(::humanoid_interfaces::msg::Tune2IkMsg::_balance_pitch_olimit_imu_type arg)
  {
    msg_.balance_pitch_olimit_imu = std::move(arg);
    return Init_Tune2IkMsg_balance_roll_flag_imu(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_pitch_elimit_imu
{
public:
  explicit Init_Tune2IkMsg_balance_pitch_elimit_imu(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_pitch_olimit_imu balance_pitch_elimit_imu(::humanoid_interfaces::msg::Tune2IkMsg::_balance_pitch_elimit_imu_type arg)
  {
    msg_.balance_pitch_elimit_imu = std::move(arg);
    return Init_Tune2IkMsg_balance_pitch_olimit_imu(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_pitch_pos_target_imu
{
public:
  explicit Init_Tune2IkMsg_balance_pitch_pos_target_imu(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_pitch_elimit_imu balance_pitch_pos_target_imu(::humanoid_interfaces::msg::Tune2IkMsg::_balance_pitch_pos_target_imu_type arg)
  {
    msg_.balance_pitch_pos_target_imu = std::move(arg);
    return Init_Tune2IkMsg_balance_pitch_elimit_imu(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_pitch_neg_target_imu
{
public:
  explicit Init_Tune2IkMsg_balance_pitch_neg_target_imu(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_pitch_pos_target_imu balance_pitch_neg_target_imu(::humanoid_interfaces::msg::Tune2IkMsg::_balance_pitch_neg_target_imu_type arg)
  {
    msg_.balance_pitch_neg_target_imu = std::move(arg);
    return Init_Tune2IkMsg_balance_pitch_pos_target_imu(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_pitch_gd_imu
{
public:
  explicit Init_Tune2IkMsg_balance_pitch_gd_imu(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_pitch_neg_target_imu balance_pitch_gd_imu(::humanoid_interfaces::msg::Tune2IkMsg::_balance_pitch_gd_imu_type arg)
  {
    msg_.balance_pitch_gd_imu = std::move(arg);
    return Init_Tune2IkMsg_balance_pitch_neg_target_imu(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_pitch_gi_imu
{
public:
  explicit Init_Tune2IkMsg_balance_pitch_gi_imu(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_pitch_gd_imu balance_pitch_gi_imu(::humanoid_interfaces::msg::Tune2IkMsg::_balance_pitch_gi_imu_type arg)
  {
    msg_.balance_pitch_gi_imu = std::move(arg);
    return Init_Tune2IkMsg_balance_pitch_gd_imu(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_pitch_gp_imu
{
public:
  explicit Init_Tune2IkMsg_balance_pitch_gp_imu(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_pitch_gi_imu balance_pitch_gp_imu(::humanoid_interfaces::msg::Tune2IkMsg::_balance_pitch_gp_imu_type arg)
  {
    msg_.balance_pitch_gp_imu = std::move(arg);
    return Init_Tune2IkMsg_balance_pitch_gi_imu(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_pitch_flag_imu
{
public:
  explicit Init_Tune2IkMsg_balance_pitch_flag_imu(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_pitch_gp_imu balance_pitch_flag_imu(::humanoid_interfaces::msg::Tune2IkMsg::_balance_pitch_flag_imu_type arg)
  {
    msg_.balance_pitch_flag_imu = std::move(arg);
    return Init_Tune2IkMsg_balance_pitch_gp_imu(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_value_5
{
public:
  explicit Init_Tune2IkMsg_balance_value_5(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_pitch_flag_imu balance_value_5(::humanoid_interfaces::msg::Tune2IkMsg::_balance_value_5_type arg)
  {
    msg_.balance_value_5 = std::move(arg);
    return Init_Tune2IkMsg_balance_pitch_flag_imu(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_value_4
{
public:
  explicit Init_Tune2IkMsg_balance_value_4(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_value_5 balance_value_4(::humanoid_interfaces::msg::Tune2IkMsg::_balance_value_4_type arg)
  {
    msg_.balance_value_4 = std::move(arg);
    return Init_Tune2IkMsg_balance_value_5(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_angle_pitch_pos_target
{
public:
  explicit Init_Tune2IkMsg_balance_angle_pitch_pos_target(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_value_4 balance_angle_pitch_pos_target(::humanoid_interfaces::msg::Tune2IkMsg::_balance_angle_pitch_pos_target_type arg)
  {
    msg_.balance_angle_pitch_pos_target = std::move(arg);
    return Init_Tune2IkMsg_balance_value_4(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_angle_pitch_neg_target
{
public:
  explicit Init_Tune2IkMsg_balance_angle_pitch_neg_target(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_angle_pitch_pos_target balance_angle_pitch_neg_target(::humanoid_interfaces::msg::Tune2IkMsg::_balance_angle_pitch_neg_target_type arg)
  {
    msg_.balance_angle_pitch_neg_target = std::move(arg);
    return Init_Tune2IkMsg_balance_angle_pitch_pos_target(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_angle_pitch_olimit
{
public:
  explicit Init_Tune2IkMsg_balance_angle_pitch_olimit(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_angle_pitch_neg_target balance_angle_pitch_olimit(::humanoid_interfaces::msg::Tune2IkMsg::_balance_angle_pitch_olimit_type arg)
  {
    msg_.balance_angle_pitch_olimit = std::move(arg);
    return Init_Tune2IkMsg_balance_angle_pitch_neg_target(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_angle_pitch_elimit
{
public:
  explicit Init_Tune2IkMsg_balance_angle_pitch_elimit(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_angle_pitch_olimit balance_angle_pitch_elimit(::humanoid_interfaces::msg::Tune2IkMsg::_balance_angle_pitch_elimit_type arg)
  {
    msg_.balance_angle_pitch_elimit = std::move(arg);
    return Init_Tune2IkMsg_balance_angle_pitch_olimit(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_angle_pitch_gd
{
public:
  explicit Init_Tune2IkMsg_balance_angle_pitch_gd(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_angle_pitch_elimit balance_angle_pitch_gd(::humanoid_interfaces::msg::Tune2IkMsg::_balance_angle_pitch_gd_type arg)
  {
    msg_.balance_angle_pitch_gd = std::move(arg);
    return Init_Tune2IkMsg_balance_angle_pitch_elimit(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_angle_pitch_gi
{
public:
  explicit Init_Tune2IkMsg_balance_angle_pitch_gi(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_angle_pitch_gd balance_angle_pitch_gi(::humanoid_interfaces::msg::Tune2IkMsg::_balance_angle_pitch_gi_type arg)
  {
    msg_.balance_angle_pitch_gi = std::move(arg);
    return Init_Tune2IkMsg_balance_angle_pitch_gd(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_angle_pitch_gp
{
public:
  explicit Init_Tune2IkMsg_balance_angle_pitch_gp(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_angle_pitch_gi balance_angle_pitch_gp(::humanoid_interfaces::msg::Tune2IkMsg::_balance_angle_pitch_gp_type arg)
  {
    msg_.balance_angle_pitch_gp = std::move(arg);
    return Init_Tune2IkMsg_balance_angle_pitch_gi(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_roll_pos_target
{
public:
  explicit Init_Tune2IkMsg_balance_roll_pos_target(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_angle_pitch_gp balance_roll_pos_target(::humanoid_interfaces::msg::Tune2IkMsg::_balance_roll_pos_target_type arg)
  {
    msg_.balance_roll_pos_target = std::move(arg);
    return Init_Tune2IkMsg_balance_angle_pitch_gp(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_roll_neg_target
{
public:
  explicit Init_Tune2IkMsg_balance_roll_neg_target(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_roll_pos_target balance_roll_neg_target(::humanoid_interfaces::msg::Tune2IkMsg::_balance_roll_neg_target_type arg)
  {
    msg_.balance_roll_neg_target = std::move(arg);
    return Init_Tune2IkMsg_balance_roll_pos_target(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_roll_olimit
{
public:
  explicit Init_Tune2IkMsg_balance_roll_olimit(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_roll_neg_target balance_roll_olimit(::humanoid_interfaces::msg::Tune2IkMsg::_balance_roll_olimit_type arg)
  {
    msg_.balance_roll_olimit = std::move(arg);
    return Init_Tune2IkMsg_balance_roll_neg_target(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_roll_elimit
{
public:
  explicit Init_Tune2IkMsg_balance_roll_elimit(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_roll_olimit balance_roll_elimit(::humanoid_interfaces::msg::Tune2IkMsg::_balance_roll_elimit_type arg)
  {
    msg_.balance_roll_elimit = std::move(arg);
    return Init_Tune2IkMsg_balance_roll_olimit(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_roll_gd
{
public:
  explicit Init_Tune2IkMsg_balance_roll_gd(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_roll_elimit balance_roll_gd(::humanoid_interfaces::msg::Tune2IkMsg::_balance_roll_gd_type arg)
  {
    msg_.balance_roll_gd = std::move(arg);
    return Init_Tune2IkMsg_balance_roll_elimit(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_roll_gi
{
public:
  explicit Init_Tune2IkMsg_balance_roll_gi(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_roll_gd balance_roll_gi(::humanoid_interfaces::msg::Tune2IkMsg::_balance_roll_gi_type arg)
  {
    msg_.balance_roll_gi = std::move(arg);
    return Init_Tune2IkMsg_balance_roll_gd(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_roll_gp
{
public:
  explicit Init_Tune2IkMsg_balance_roll_gp(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_roll_gi balance_roll_gp(::humanoid_interfaces::msg::Tune2IkMsg::_balance_roll_gp_type arg)
  {
    msg_.balance_roll_gp = std::move(arg);
    return Init_Tune2IkMsg_balance_roll_gi(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_pitch_pos_target
{
public:
  explicit Init_Tune2IkMsg_balance_pitch_pos_target(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_roll_gp balance_pitch_pos_target(::humanoid_interfaces::msg::Tune2IkMsg::_balance_pitch_pos_target_type arg)
  {
    msg_.balance_pitch_pos_target = std::move(arg);
    return Init_Tune2IkMsg_balance_roll_gp(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_pitch_neg_target
{
public:
  explicit Init_Tune2IkMsg_balance_pitch_neg_target(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_pitch_pos_target balance_pitch_neg_target(::humanoid_interfaces::msg::Tune2IkMsg::_balance_pitch_neg_target_type arg)
  {
    msg_.balance_pitch_neg_target = std::move(arg);
    return Init_Tune2IkMsg_balance_pitch_pos_target(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_pitch_olimit
{
public:
  explicit Init_Tune2IkMsg_balance_pitch_olimit(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_pitch_neg_target balance_pitch_olimit(::humanoid_interfaces::msg::Tune2IkMsg::_balance_pitch_olimit_type arg)
  {
    msg_.balance_pitch_olimit = std::move(arg);
    return Init_Tune2IkMsg_balance_pitch_neg_target(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_pitch_elimit
{
public:
  explicit Init_Tune2IkMsg_balance_pitch_elimit(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_pitch_olimit balance_pitch_elimit(::humanoid_interfaces::msg::Tune2IkMsg::_balance_pitch_elimit_type arg)
  {
    msg_.balance_pitch_elimit = std::move(arg);
    return Init_Tune2IkMsg_balance_pitch_olimit(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_pitch_gd
{
public:
  explicit Init_Tune2IkMsg_balance_pitch_gd(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_pitch_elimit balance_pitch_gd(::humanoid_interfaces::msg::Tune2IkMsg::_balance_pitch_gd_type arg)
  {
    msg_.balance_pitch_gd = std::move(arg);
    return Init_Tune2IkMsg_balance_pitch_elimit(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_pitch_gi
{
public:
  explicit Init_Tune2IkMsg_balance_pitch_gi(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_pitch_gd balance_pitch_gi(::humanoid_interfaces::msg::Tune2IkMsg::_balance_pitch_gi_type arg)
  {
    msg_.balance_pitch_gi = std::move(arg);
    return Init_Tune2IkMsg_balance_pitch_gd(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_pitch_gp
{
public:
  explicit Init_Tune2IkMsg_balance_pitch_gp(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_pitch_gi balance_pitch_gp(::humanoid_interfaces::msg::Tune2IkMsg::_balance_pitch_gp_type arg)
  {
    msg_.balance_pitch_gp = std::move(arg);
    return Init_Tune2IkMsg_balance_pitch_gi(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_value_3
{
public:
  explicit Init_Tune2IkMsg_balance_value_3(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_pitch_gp balance_value_3(::humanoid_interfaces::msg::Tune2IkMsg::_balance_value_3_type arg)
  {
    msg_.balance_value_3 = std::move(arg);
    return Init_Tune2IkMsg_balance_pitch_gp(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_value_2
{
public:
  explicit Init_Tune2IkMsg_balance_value_2(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_value_3 balance_value_2(::humanoid_interfaces::msg::Tune2IkMsg::_balance_value_2_type arg)
  {
    msg_.balance_value_2 = std::move(arg);
    return Init_Tune2IkMsg_balance_value_3(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_value_1
{
public:
  explicit Init_Tune2IkMsg_balance_value_1(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_value_2 balance_value_1(::humanoid_interfaces::msg::Tune2IkMsg::_balance_value_1_type arg)
  {
    msg_.balance_value_1 = std::move(arg);
    return Init_Tune2IkMsg_balance_value_2(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_balance_value_0
{
public:
  explicit Init_Tune2IkMsg_balance_value_0(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_value_1 balance_value_0(::humanoid_interfaces::msg::Tune2IkMsg::_balance_value_0_type arg)
  {
    msg_.balance_value_0 = std::move(arg);
    return Init_Tune2IkMsg_balance_value_1(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_tuning_yaw
{
public:
  explicit Init_Tune2IkMsg_tuning_yaw(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_balance_value_0 tuning_yaw(::humanoid_interfaces::msg::Tune2IkMsg::_tuning_yaw_type arg)
  {
    msg_.tuning_yaw = std::move(arg);
    return Init_Tune2IkMsg_balance_value_0(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_tuning_side
{
public:
  explicit Init_Tune2IkMsg_tuning_side(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_tuning_yaw tuning_side(::humanoid_interfaces::msg::Tune2IkMsg::_tuning_side_type arg)
  {
    msg_.tuning_side = std::move(arg);
    return Init_Tune2IkMsg_tuning_yaw(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_tuning_x
{
public:
  explicit Init_Tune2IkMsg_tuning_x(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_tuning_side tuning_x(::humanoid_interfaces::msg::Tune2IkMsg::_tuning_x_type arg)
  {
    msg_.tuning_x = std::move(arg);
    return Init_Tune2IkMsg_tuning_side(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_test_yaw
{
public:
  explicit Init_Tune2IkMsg_test_yaw(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_tuning_x test_yaw(::humanoid_interfaces::msg::Tune2IkMsg::_test_yaw_type arg)
  {
    msg_.test_yaw = std::move(arg);
    return Init_Tune2IkMsg_tuning_x(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_test_side
{
public:
  explicit Init_Tune2IkMsg_test_side(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_test_yaw test_side(::humanoid_interfaces::msg::Tune2IkMsg::_test_side_type arg)
  {
    msg_.test_side = std::move(arg);
    return Init_Tune2IkMsg_test_yaw(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_test_x
{
public:
  explicit Init_Tune2IkMsg_test_x(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_test_side test_x(::humanoid_interfaces::msg::Tune2IkMsg::_test_x_type arg)
  {
    msg_.test_x = std::move(arg);
    return Init_Tune2IkMsg_test_side(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_end_rise
{
public:
  explicit Init_Tune2IkMsg_end_rise(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_test_x end_rise(::humanoid_interfaces::msg::Tune2IkMsg::_end_rise_type arg)
  {
    msg_.end_rise = std::move(arg);
    return Init_Tune2IkMsg_test_x(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_end_swing
{
public:
  explicit Init_Tune2IkMsg_end_swing(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_end_rise end_swing(::humanoid_interfaces::msg::Tune2IkMsg::_end_swing_type arg)
  {
    msg_.end_swing = std::move(arg);
    return Init_Tune2IkMsg_end_rise(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_end_entire_time
{
public:
  explicit Init_Tune2IkMsg_end_entire_time(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_end_swing end_entire_time(::humanoid_interfaces::msg::Tune2IkMsg::_end_entire_time_type arg)
  {
    msg_.end_entire_time = std::move(arg);
    return Init_Tune2IkMsg_end_swing(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_start_rise
{
public:
  explicit Init_Tune2IkMsg_start_rise(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_end_entire_time start_rise(::humanoid_interfaces::msg::Tune2IkMsg::_start_rise_type arg)
  {
    msg_.start_rise = std::move(arg);
    return Init_Tune2IkMsg_end_entire_time(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_start_swing
{
public:
  explicit Init_Tune2IkMsg_start_swing(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_start_rise start_swing(::humanoid_interfaces::msg::Tune2IkMsg::_start_swing_type arg)
  {
    msg_.start_swing = std::move(arg);
    return Init_Tune2IkMsg_start_rise(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_start_entire_time
{
public:
  explicit Init_Tune2IkMsg_start_entire_time(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_start_swing start_entire_time(::humanoid_interfaces::msg::Tune2IkMsg::_start_entire_time_type arg)
  {
    msg_.start_entire_time = std::move(arg);
    return Init_Tune2IkMsg_start_swing(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_rise_leg_right
{
public:
  explicit Init_Tune2IkMsg_rise_leg_right(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_start_entire_time rise_leg_right(::humanoid_interfaces::msg::Tune2IkMsg::_rise_leg_right_type arg)
  {
    msg_.rise_leg_right = std::move(arg);
    return Init_Tune2IkMsg_start_entire_time(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_rise_leg_left
{
public:
  explicit Init_Tune2IkMsg_rise_leg_left(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_rise_leg_right rise_leg_left(::humanoid_interfaces::msg::Tune2IkMsg::_rise_leg_left_type arg)
  {
    msg_.rise_leg_left = std::move(arg);
    return Init_Tune2IkMsg_rise_leg_right(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_swing_leg_right
{
public:
  explicit Init_Tune2IkMsg_swing_leg_right(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_rise_leg_left swing_leg_right(::humanoid_interfaces::msg::Tune2IkMsg::_swing_leg_right_type arg)
  {
    msg_.swing_leg_right = std::move(arg);
    return Init_Tune2IkMsg_rise_leg_left(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_swing_leg_left
{
public:
  explicit Init_Tune2IkMsg_swing_leg_left(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_swing_leg_right swing_leg_left(::humanoid_interfaces::msg::Tune2IkMsg::_swing_leg_left_type arg)
  {
    msg_.swing_leg_left = std::move(arg);
    return Init_Tune2IkMsg_swing_leg_right(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_swing_shoulder_right
{
public:
  explicit Init_Tune2IkMsg_swing_shoulder_right(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_swing_leg_left swing_shoulder_right(::humanoid_interfaces::msg::Tune2IkMsg::_swing_shoulder_right_type arg)
  {
    msg_.swing_shoulder_right = std::move(arg);
    return Init_Tune2IkMsg_swing_leg_left(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_swing_shoulder_left
{
public:
  explicit Init_Tune2IkMsg_swing_shoulder_left(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_swing_shoulder_right swing_shoulder_left(::humanoid_interfaces::msg::Tune2IkMsg::_swing_shoulder_left_type arg)
  {
    msg_.swing_shoulder_left = std::move(arg);
    return Init_Tune2IkMsg_swing_shoulder_right(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_offset_motor
{
public:
  explicit Init_Tune2IkMsg_offset_motor(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_swing_shoulder_left offset_motor(::humanoid_interfaces::msg::Tune2IkMsg::_offset_motor_type arg)
  {
    msg_.offset_motor = std::move(arg);
    return Init_Tune2IkMsg_swing_shoulder_left(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_default_z_right
{
public:
  explicit Init_Tune2IkMsg_default_z_right(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_offset_motor default_z_right(::humanoid_interfaces::msg::Tune2IkMsg::_default_z_right_type arg)
  {
    msg_.default_z_right = std::move(arg);
    return Init_Tune2IkMsg_offset_motor(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_default_y_right
{
public:
  explicit Init_Tune2IkMsg_default_y_right(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_default_z_right default_y_right(::humanoid_interfaces::msg::Tune2IkMsg::_default_y_right_type arg)
  {
    msg_.default_y_right = std::move(arg);
    return Init_Tune2IkMsg_default_z_right(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_default_x_right
{
public:
  explicit Init_Tune2IkMsg_default_x_right(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_default_y_right default_x_right(::humanoid_interfaces::msg::Tune2IkMsg::_default_x_right_type arg)
  {
    msg_.default_x_right = std::move(arg);
    return Init_Tune2IkMsg_default_y_right(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_default_z_left
{
public:
  explicit Init_Tune2IkMsg_default_z_left(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_default_x_right default_z_left(::humanoid_interfaces::msg::Tune2IkMsg::_default_z_left_type arg)
  {
    msg_.default_z_left = std::move(arg);
    return Init_Tune2IkMsg_default_x_right(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_default_y_left
{
public:
  explicit Init_Tune2IkMsg_default_y_left(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_default_z_left default_y_left(::humanoid_interfaces::msg::Tune2IkMsg::_default_y_left_type arg)
  {
    msg_.default_y_left = std::move(arg);
    return Init_Tune2IkMsg_default_z_left(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_default_x_left
{
public:
  explicit Init_Tune2IkMsg_default_x_left(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_default_y_left default_x_left(::humanoid_interfaces::msg::Tune2IkMsg::_default_x_left_type arg)
  {
    msg_.default_x_left = std::move(arg);
    return Init_Tune2IkMsg_default_y_left(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_frequency
{
public:
  explicit Init_Tune2IkMsg_frequency(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_default_x_left frequency(::humanoid_interfaces::msg::Tune2IkMsg::_frequency_type arg)
  {
    msg_.frequency = std::move(arg);
    return Init_Tune2IkMsg_default_x_left(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_entire_time
{
public:
  explicit Init_Tune2IkMsg_entire_time(::humanoid_interfaces::msg::Tune2IkMsg & msg)
  : msg_(msg)
  {}
  Init_Tune2IkMsg_frequency entire_time(::humanoid_interfaces::msg::Tune2IkMsg::_entire_time_type arg)
  {
    msg_.entire_time = std::move(arg);
    return Init_Tune2IkMsg_frequency(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

class Init_Tune2IkMsg_ik_flag
{
public:
  Init_Tune2IkMsg_ik_flag()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Tune2IkMsg_entire_time ik_flag(::humanoid_interfaces::msg::Tune2IkMsg::_ik_flag_type arg)
  {
    msg_.ik_flag = std::move(arg);
    return Init_Tune2IkMsg_entire_time(msg_);
  }

private:
  ::humanoid_interfaces::msg::Tune2IkMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::Tune2IkMsg>()
{
  return humanoid_interfaces::msg::builder::Init_Tune2IkMsg_ik_flag();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__TUNE2_IK_MSG__BUILDER_HPP_
