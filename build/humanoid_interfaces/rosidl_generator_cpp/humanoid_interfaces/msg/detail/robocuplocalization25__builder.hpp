// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/Robocuplocalization25.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPLOCALIZATION25__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPLOCALIZATION25__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/robocuplocalization25__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_Robocuplocalization25_obstacle3_y
{
public:
  explicit Init_Robocuplocalization25_obstacle3_y(::humanoid_interfaces::msg::Robocuplocalization25 & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::Robocuplocalization25 obstacle3_y(::humanoid_interfaces::msg::Robocuplocalization25::_obstacle3_y_type arg)
  {
    msg_.obstacle3_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocuplocalization25 msg_;
};

class Init_Robocuplocalization25_obstacle3_x
{
public:
  explicit Init_Robocuplocalization25_obstacle3_x(::humanoid_interfaces::msg::Robocuplocalization25 & msg)
  : msg_(msg)
  {}
  Init_Robocuplocalization25_obstacle3_y obstacle3_x(::humanoid_interfaces::msg::Robocuplocalization25::_obstacle3_x_type arg)
  {
    msg_.obstacle3_x = std::move(arg);
    return Init_Robocuplocalization25_obstacle3_y(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocuplocalization25 msg_;
};

class Init_Robocuplocalization25_obstacle2_y
{
public:
  explicit Init_Robocuplocalization25_obstacle2_y(::humanoid_interfaces::msg::Robocuplocalization25 & msg)
  : msg_(msg)
  {}
  Init_Robocuplocalization25_obstacle3_x obstacle2_y(::humanoid_interfaces::msg::Robocuplocalization25::_obstacle2_y_type arg)
  {
    msg_.obstacle2_y = std::move(arg);
    return Init_Robocuplocalization25_obstacle3_x(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocuplocalization25 msg_;
};

class Init_Robocuplocalization25_obstacle2_x
{
public:
  explicit Init_Robocuplocalization25_obstacle2_x(::humanoid_interfaces::msg::Robocuplocalization25 & msg)
  : msg_(msg)
  {}
  Init_Robocuplocalization25_obstacle2_y obstacle2_x(::humanoid_interfaces::msg::Robocuplocalization25::_obstacle2_x_type arg)
  {
    msg_.obstacle2_x = std::move(arg);
    return Init_Robocuplocalization25_obstacle2_y(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocuplocalization25 msg_;
};

class Init_Robocuplocalization25_obstacle1_y
{
public:
  explicit Init_Robocuplocalization25_obstacle1_y(::humanoid_interfaces::msg::Robocuplocalization25 & msg)
  : msg_(msg)
  {}
  Init_Robocuplocalization25_obstacle2_x obstacle1_y(::humanoid_interfaces::msg::Robocuplocalization25::_obstacle1_y_type arg)
  {
    msg_.obstacle1_y = std::move(arg);
    return Init_Robocuplocalization25_obstacle2_x(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocuplocalization25 msg_;
};

class Init_Robocuplocalization25_obstacle1_x
{
public:
  explicit Init_Robocuplocalization25_obstacle1_x(::humanoid_interfaces::msg::Robocuplocalization25 & msg)
  : msg_(msg)
  {}
  Init_Robocuplocalization25_obstacle1_y obstacle1_x(::humanoid_interfaces::msg::Robocuplocalization25::_obstacle1_x_type arg)
  {
    msg_.obstacle1_x = std::move(arg);
    return Init_Robocuplocalization25_obstacle1_y(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocuplocalization25 msg_;
};

class Init_Robocuplocalization25_obstacle0_y
{
public:
  explicit Init_Robocuplocalization25_obstacle0_y(::humanoid_interfaces::msg::Robocuplocalization25 & msg)
  : msg_(msg)
  {}
  Init_Robocuplocalization25_obstacle1_x obstacle0_y(::humanoid_interfaces::msg::Robocuplocalization25::_obstacle0_y_type arg)
  {
    msg_.obstacle0_y = std::move(arg);
    return Init_Robocuplocalization25_obstacle1_x(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocuplocalization25 msg_;
};

class Init_Robocuplocalization25_obstacle0_x
{
public:
  explicit Init_Robocuplocalization25_obstacle0_x(::humanoid_interfaces::msg::Robocuplocalization25 & msg)
  : msg_(msg)
  {}
  Init_Robocuplocalization25_obstacle0_y obstacle0_x(::humanoid_interfaces::msg::Robocuplocalization25::_obstacle0_x_type arg)
  {
    msg_.obstacle0_x = std::move(arg);
    return Init_Robocuplocalization25_obstacle0_y(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocuplocalization25 msg_;
};

class Init_Robocuplocalization25_obstacle_y
{
public:
  explicit Init_Robocuplocalization25_obstacle_y(::humanoid_interfaces::msg::Robocuplocalization25 & msg)
  : msg_(msg)
  {}
  Init_Robocuplocalization25_obstacle0_x obstacle_y(::humanoid_interfaces::msg::Robocuplocalization25::_obstacle_y_type arg)
  {
    msg_.obstacle_y = std::move(arg);
    return Init_Robocuplocalization25_obstacle0_x(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocuplocalization25 msg_;
};

class Init_Robocuplocalization25_obstacle_x
{
public:
  explicit Init_Robocuplocalization25_obstacle_x(::humanoid_interfaces::msg::Robocuplocalization25 & msg)
  : msg_(msg)
  {}
  Init_Robocuplocalization25_obstacle_y obstacle_x(::humanoid_interfaces::msg::Robocuplocalization25::_obstacle_x_type arg)
  {
    msg_.obstacle_x = std::move(arg);
    return Init_Robocuplocalization25_obstacle_y(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocuplocalization25 msg_;
};

class Init_Robocuplocalization25_robot_y
{
public:
  explicit Init_Robocuplocalization25_robot_y(::humanoid_interfaces::msg::Robocuplocalization25 & msg)
  : msg_(msg)
  {}
  Init_Robocuplocalization25_obstacle_x robot_y(::humanoid_interfaces::msg::Robocuplocalization25::_robot_y_type arg)
  {
    msg_.robot_y = std::move(arg);
    return Init_Robocuplocalization25_obstacle_x(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocuplocalization25 msg_;
};

class Init_Robocuplocalization25_robot_x
{
public:
  explicit Init_Robocuplocalization25_robot_x(::humanoid_interfaces::msg::Robocuplocalization25 & msg)
  : msg_(msg)
  {}
  Init_Robocuplocalization25_robot_y robot_x(::humanoid_interfaces::msg::Robocuplocalization25::_robot_x_type arg)
  {
    msg_.robot_x = std::move(arg);
    return Init_Robocuplocalization25_robot_y(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocuplocalization25 msg_;
};

class Init_Robocuplocalization25_ball_speed_y
{
public:
  explicit Init_Robocuplocalization25_ball_speed_y(::humanoid_interfaces::msg::Robocuplocalization25 & msg)
  : msg_(msg)
  {}
  Init_Robocuplocalization25_robot_x ball_speed_y(::humanoid_interfaces::msg::Robocuplocalization25::_ball_speed_y_type arg)
  {
    msg_.ball_speed_y = std::move(arg);
    return Init_Robocuplocalization25_robot_x(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocuplocalization25 msg_;
};

class Init_Robocuplocalization25_ball_speed_x
{
public:
  explicit Init_Robocuplocalization25_ball_speed_x(::humanoid_interfaces::msg::Robocuplocalization25 & msg)
  : msg_(msg)
  {}
  Init_Robocuplocalization25_ball_speed_y ball_speed_x(::humanoid_interfaces::msg::Robocuplocalization25::_ball_speed_x_type arg)
  {
    msg_.ball_speed_x = std::move(arg);
    return Init_Robocuplocalization25_ball_speed_y(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocuplocalization25 msg_;
};

class Init_Robocuplocalization25_ball_y
{
public:
  explicit Init_Robocuplocalization25_ball_y(::humanoid_interfaces::msg::Robocuplocalization25 & msg)
  : msg_(msg)
  {}
  Init_Robocuplocalization25_ball_speed_x ball_y(::humanoid_interfaces::msg::Robocuplocalization25::_ball_y_type arg)
  {
    msg_.ball_y = std::move(arg);
    return Init_Robocuplocalization25_ball_speed_x(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocuplocalization25 msg_;
};

class Init_Robocuplocalization25_ball_x
{
public:
  Init_Robocuplocalization25_ball_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Robocuplocalization25_ball_y ball_x(::humanoid_interfaces::msg::Robocuplocalization25::_ball_x_type arg)
  {
    msg_.ball_x = std::move(arg);
    return Init_Robocuplocalization25_ball_y(msg_);
  }

private:
  ::humanoid_interfaces::msg::Robocuplocalization25 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::Robocuplocalization25>()
{
  return humanoid_interfaces::msg::builder::Init_Robocuplocalization25_ball_x();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPLOCALIZATION25__BUILDER_HPP_
