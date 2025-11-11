// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/ImuMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IMU_MSG__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IMU_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/imu_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_ImuMsg_z_pos
{
public:
  explicit Init_ImuMsg_z_pos(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::ImuMsg z_pos(::humanoid_interfaces::msg::ImuMsg::_z_pos_type arg)
  {
    msg_.z_pos = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_y_pos
{
public:
  explicit Init_ImuMsg_y_pos(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_z_pos y_pos(::humanoid_interfaces::msg::ImuMsg::_y_pos_type arg)
  {
    msg_.y_pos = std::move(arg);
    return Init_ImuMsg_z_pos(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_x_pos
{
public:
  explicit Init_ImuMsg_x_pos(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_y_pos x_pos(::humanoid_interfaces::msg::ImuMsg::_x_pos_type arg)
  {
    msg_.x_pos = std::move(arg);
    return Init_ImuMsg_y_pos(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_z_vel
{
public:
  explicit Init_ImuMsg_z_vel(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_x_pos z_vel(::humanoid_interfaces::msg::ImuMsg::_z_vel_type arg)
  {
    msg_.z_vel = std::move(arg);
    return Init_ImuMsg_x_pos(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_y_vel
{
public:
  explicit Init_ImuMsg_y_vel(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_z_vel y_vel(::humanoid_interfaces::msg::ImuMsg::_y_vel_type arg)
  {
    msg_.y_vel = std::move(arg);
    return Init_ImuMsg_z_vel(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_x_vel
{
public:
  explicit Init_ImuMsg_x_vel(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_y_vel x_vel(::humanoid_interfaces::msg::ImuMsg::_x_vel_type arg)
  {
    msg_.x_vel = std::move(arg);
    return Init_ImuMsg_y_vel(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_z_acc
{
public:
  explicit Init_ImuMsg_z_acc(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_x_vel z_acc(::humanoid_interfaces::msg::ImuMsg::_z_acc_type arg)
  {
    msg_.z_acc = std::move(arg);
    return Init_ImuMsg_x_vel(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_y_acc
{
public:
  explicit Init_ImuMsg_y_acc(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_z_acc y_acc(::humanoid_interfaces::msg::ImuMsg::_y_acc_type arg)
  {
    msg_.y_acc = std::move(arg);
    return Init_ImuMsg_z_acc(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_x_acc
{
public:
  explicit Init_ImuMsg_x_acc(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_y_acc x_acc(::humanoid_interfaces::msg::ImuMsg::_x_acc_type arg)
  {
    msg_.x_acc = std::move(arg);
    return Init_ImuMsg_y_acc(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_z_gyro
{
public:
  explicit Init_ImuMsg_z_gyro(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_x_acc z_gyro(::humanoid_interfaces::msg::ImuMsg::_z_gyro_type arg)
  {
    msg_.z_gyro = std::move(arg);
    return Init_ImuMsg_x_acc(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_y_gyro
{
public:
  explicit Init_ImuMsg_y_gyro(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_z_gyro y_gyro(::humanoid_interfaces::msg::ImuMsg::_y_gyro_type arg)
  {
    msg_.y_gyro = std::move(arg);
    return Init_ImuMsg_z_gyro(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_x_gyro
{
public:
  explicit Init_ImuMsg_x_gyro(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_y_gyro x_gyro(::humanoid_interfaces::msg::ImuMsg::_x_gyro_type arg)
  {
    msg_.x_gyro = std::move(arg);
    return Init_ImuMsg_y_gyro(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_q_w
{
public:
  explicit Init_ImuMsg_q_w(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_x_gyro q_w(::humanoid_interfaces::msg::ImuMsg::_q_w_type arg)
  {
    msg_.q_w = std::move(arg);
    return Init_ImuMsg_x_gyro(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_q_z
{
public:
  explicit Init_ImuMsg_q_z(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_q_w q_z(::humanoid_interfaces::msg::ImuMsg::_q_z_type arg)
  {
    msg_.q_z = std::move(arg);
    return Init_ImuMsg_q_w(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_q_y
{
public:
  explicit Init_ImuMsg_q_y(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_q_z q_y(::humanoid_interfaces::msg::ImuMsg::_q_y_type arg)
  {
    msg_.q_y = std::move(arg);
    return Init_ImuMsg_q_z(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_q_x
{
public:
  explicit Init_ImuMsg_q_x(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_q_y q_x(::humanoid_interfaces::msg::ImuMsg::_q_x_type arg)
  {
    msg_.q_x = std::move(arg);
    return Init_ImuMsg_q_y(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_yaw
{
public:
  explicit Init_ImuMsg_yaw(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_q_x yaw(::humanoid_interfaces::msg::ImuMsg::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_ImuMsg_q_x(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_roll
{
public:
  explicit Init_ImuMsg_roll(::humanoid_interfaces::msg::ImuMsg & msg)
  : msg_(msg)
  {}
  Init_ImuMsg_yaw roll(::humanoid_interfaces::msg::ImuMsg::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_ImuMsg_yaw(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

class Init_ImuMsg_pitch
{
public:
  Init_ImuMsg_pitch()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ImuMsg_roll pitch(::humanoid_interfaces::msg::ImuMsg::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_ImuMsg_roll(msg_);
  }

private:
  ::humanoid_interfaces::msg::ImuMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::ImuMsg>()
{
  return humanoid_interfaces::msg::builder::Init_ImuMsg_pitch();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IMU_MSG__BUILDER_HPP_
