// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/Robocupvision25.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/robocupvision25__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_Robocupvision25_scan_mode
{
public:
  explicit Init_Robocupvision25_scan_mode(::humanoid_interfaces::msg::Robocupvision25 & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::Robocupvision25 scan_mode(::humanoid_interfaces::msg::Robocupvision25::_scan_mode_type arg)
  {
    msg_.scan_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25 msg_;
};

class Init_Robocupvision25_ball_speed_level
{
public:
  explicit Init_Robocupvision25_ball_speed_level(::humanoid_interfaces::msg::Robocupvision25 & msg)
  : msg_(msg)
  {}
  Init_Robocupvision25_scan_mode ball_speed_level(::humanoid_interfaces::msg::Robocupvision25::_ball_speed_level_type arg)
  {
    msg_.ball_speed_level = std::move(arg);
    return Init_Robocupvision25_scan_mode(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25 msg_;
};

class Init_Robocupvision25_robot_vec_y
{
public:
  explicit Init_Robocupvision25_robot_vec_y(::humanoid_interfaces::msg::Robocupvision25 & msg)
  : msg_(msg)
  {}
  Init_Robocupvision25_ball_speed_level robot_vec_y(::humanoid_interfaces::msg::Robocupvision25::_robot_vec_y_type arg)
  {
    msg_.robot_vec_y = std::move(arg);
    return Init_Robocupvision25_ball_speed_level(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25 msg_;
};

class Init_Robocupvision25_robot_vec_x
{
public:
  explicit Init_Robocupvision25_robot_vec_x(::humanoid_interfaces::msg::Robocupvision25 & msg)
  : msg_(msg)
  {}
  Init_Robocupvision25_robot_vec_y robot_vec_x(::humanoid_interfaces::msg::Robocupvision25::_robot_vec_x_type arg)
  {
    msg_.robot_vec_x = std::move(arg);
    return Init_Robocupvision25_robot_vec_y(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25 msg_;
};

class Init_Robocupvision25_ball_speed_y
{
public:
  explicit Init_Robocupvision25_ball_speed_y(::humanoid_interfaces::msg::Robocupvision25 & msg)
  : msg_(msg)
  {}
  Init_Robocupvision25_robot_vec_x ball_speed_y(::humanoid_interfaces::msg::Robocupvision25::_ball_speed_y_type arg)
  {
    msg_.ball_speed_y = std::move(arg);
    return Init_Robocupvision25_robot_vec_x(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25 msg_;
};

class Init_Robocupvision25_ball_speed_x
{
public:
  explicit Init_Robocupvision25_ball_speed_x(::humanoid_interfaces::msg::Robocupvision25 & msg)
  : msg_(msg)
  {}
  Init_Robocupvision25_ball_speed_y ball_speed_x(::humanoid_interfaces::msg::Robocupvision25::_ball_speed_x_type arg)
  {
    msg_.ball_speed_x = std::move(arg);
    return Init_Robocupvision25_ball_speed_y(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25 msg_;
};

class Init_Robocupvision25_tilt
{
public:
  explicit Init_Robocupvision25_tilt(::humanoid_interfaces::msg::Robocupvision25 & msg)
  : msg_(msg)
  {}
  Init_Robocupvision25_ball_speed_x tilt(::humanoid_interfaces::msg::Robocupvision25::_tilt_type arg)
  {
    msg_.tilt = std::move(arg);
    return Init_Robocupvision25_ball_speed_x(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25 msg_;
};

class Init_Robocupvision25_pan
{
public:
  explicit Init_Robocupvision25_pan(::humanoid_interfaces::msg::Robocupvision25 & msg)
  : msg_(msg)
  {}
  Init_Robocupvision25_tilt pan(::humanoid_interfaces::msg::Robocupvision25::_pan_type arg)
  {
    msg_.pan = std::move(arg);
    return Init_Robocupvision25_tilt(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25 msg_;
};

class Init_Robocupvision25_ball_d
{
public:
  explicit Init_Robocupvision25_ball_d(::humanoid_interfaces::msg::Robocupvision25 & msg)
  : msg_(msg)
  {}
  Init_Robocupvision25_pan ball_d(::humanoid_interfaces::msg::Robocupvision25::_ball_d_type arg)
  {
    msg_.ball_d = std::move(arg);
    return Init_Robocupvision25_pan(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25 msg_;
};

class Init_Robocupvision25_ball_2d_y
{
public:
  explicit Init_Robocupvision25_ball_2d_y(::humanoid_interfaces::msg::Robocupvision25 & msg)
  : msg_(msg)
  {}
  Init_Robocupvision25_ball_d ball_2d_y(::humanoid_interfaces::msg::Robocupvision25::_ball_2d_y_type arg)
  {
    msg_.ball_2d_y = std::move(arg);
    return Init_Robocupvision25_ball_d(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25 msg_;
};

class Init_Robocupvision25_ball_2d_x
{
public:
  explicit Init_Robocupvision25_ball_2d_x(::humanoid_interfaces::msg::Robocupvision25 & msg)
  : msg_(msg)
  {}
  Init_Robocupvision25_ball_2d_y ball_2d_x(::humanoid_interfaces::msg::Robocupvision25::_ball_2d_x_type arg)
  {
    msg_.ball_2d_x = std::move(arg);
    return Init_Robocupvision25_ball_2d_y(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25 msg_;
};

class Init_Robocupvision25_ball_cam_y
{
public:
  explicit Init_Robocupvision25_ball_cam_y(::humanoid_interfaces::msg::Robocupvision25 & msg)
  : msg_(msg)
  {}
  Init_Robocupvision25_ball_2d_x ball_cam_y(::humanoid_interfaces::msg::Robocupvision25::_ball_cam_y_type arg)
  {
    msg_.ball_cam_y = std::move(arg);
    return Init_Robocupvision25_ball_2d_x(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25 msg_;
};

class Init_Robocupvision25_ball_cam_x
{
public:
  Init_Robocupvision25_ball_cam_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Robocupvision25_ball_cam_y ball_cam_x(::humanoid_interfaces::msg::Robocupvision25::_ball_cam_x_type arg)
  {
    msg_.ball_cam_x = std::move(arg);
    return Init_Robocupvision25_ball_cam_y(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocupvision25 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::Robocupvision25>()
{
  return humanoid_interfaces::msg::builder::Init_Robocupvision25_ball_cam_x();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25__BUILDER_HPP_
