// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/Gamecontroldata.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLDATA__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLDATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/gamecontroldata__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_Gamecontroldata_secondinfo
{
public:
  explicit Init_Gamecontroldata_secondinfo(::humanoid_interfaces::msg::Gamecontroldata & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::Gamecontroldata secondinfo(::humanoid_interfaces::msg::Gamecontroldata::_secondinfo_type arg)
  {
    msg_.secondinfo = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::Gamecontroldata msg_;
};

class Init_Gamecontroldata_secondstate
{
public:
  explicit Init_Gamecontroldata_secondstate(::humanoid_interfaces::msg::Gamecontroldata & msg)
  : msg_(msg)
  {}
  Init_Gamecontroldata_secondinfo secondstate(::humanoid_interfaces::msg::Gamecontroldata::_secondstate_type arg)
  {
    msg_.secondstate = std::move(arg);
    return Init_Gamecontroldata_secondinfo(msg_);
  }

private:
  ::humanoid_interfaces::msg::Gamecontroldata msg_;
};

class Init_Gamecontroldata_penalty
{
public:
  explicit Init_Gamecontroldata_penalty(::humanoid_interfaces::msg::Gamecontroldata & msg)
  : msg_(msg)
  {}
  Init_Gamecontroldata_secondstate penalty(::humanoid_interfaces::msg::Gamecontroldata::_penalty_type arg)
  {
    msg_.penalty = std::move(arg);
    return Init_Gamecontroldata_secondstate(msg_);
  }

private:
  ::humanoid_interfaces::msg::Gamecontroldata msg_;
};

class Init_Gamecontroldata_readytime
{
public:
  explicit Init_Gamecontroldata_readytime(::humanoid_interfaces::msg::Gamecontroldata & msg)
  : msg_(msg)
  {}
  Init_Gamecontroldata_penalty readytime(::humanoid_interfaces::msg::Gamecontroldata::_readytime_type arg)
  {
    msg_.readytime = std::move(arg);
    return Init_Gamecontroldata_penalty(msg_);
  }

private:
  ::humanoid_interfaces::msg::Gamecontroldata msg_;
};

class Init_Gamecontroldata_iskickoff
{
public:
  explicit Init_Gamecontroldata_iskickoff(::humanoid_interfaces::msg::Gamecontroldata & msg)
  : msg_(msg)
  {}
  Init_Gamecontroldata_readytime iskickoff(::humanoid_interfaces::msg::Gamecontroldata::_iskickoff_type arg)
  {
    msg_.iskickoff = std::move(arg);
    return Init_Gamecontroldata_readytime(msg_);
  }

private:
  ::humanoid_interfaces::msg::Gamecontroldata msg_;
};

class Init_Gamecontroldata_myside
{
public:
  explicit Init_Gamecontroldata_myside(::humanoid_interfaces::msg::Gamecontroldata & msg)
  : msg_(msg)
  {}
  Init_Gamecontroldata_iskickoff myside(::humanoid_interfaces::msg::Gamecontroldata::_myside_type arg)
  {
    msg_.myside = std::move(arg);
    return Init_Gamecontroldata_iskickoff(msg_);
  }

private:
  ::humanoid_interfaces::msg::Gamecontroldata msg_;
};

class Init_Gamecontroldata_state
{
public:
  explicit Init_Gamecontroldata_state(::humanoid_interfaces::msg::Gamecontroldata & msg)
  : msg_(msg)
  {}
  Init_Gamecontroldata_myside state(::humanoid_interfaces::msg::Gamecontroldata::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_Gamecontroldata_myside(msg_);
  }

private:
  ::humanoid_interfaces::msg::Gamecontroldata msg_;
};

class Init_Gamecontroldata_myteam
{
public:
  explicit Init_Gamecontroldata_myteam(::humanoid_interfaces::msg::Gamecontroldata & msg)
  : msg_(msg)
  {}
  Init_Gamecontroldata_state myteam(::humanoid_interfaces::msg::Gamecontroldata::_myteam_type arg)
  {
    msg_.myteam = std::move(arg);
    return Init_Gamecontroldata_state(msg_);
  }

private:
  ::humanoid_interfaces::msg::Gamecontroldata msg_;
};

class Init_Gamecontroldata_position
{
public:
  explicit Init_Gamecontroldata_position(::humanoid_interfaces::msg::Gamecontroldata & msg)
  : msg_(msg)
  {}
  Init_Gamecontroldata_myteam position(::humanoid_interfaces::msg::Gamecontroldata::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Gamecontroldata_myteam(msg_);
  }

private:
  ::humanoid_interfaces::msg::Gamecontroldata msg_;
};

class Init_Gamecontroldata_robotnum
{
public:
  Init_Gamecontroldata_robotnum()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Gamecontroldata_position robotnum(::humanoid_interfaces::msg::Gamecontroldata::_robotnum_type arg)
  {
    msg_.robotnum = std::move(arg);
    return Init_Gamecontroldata_position(msg_);
  }

private:
  ::humanoid_interfaces::msg::Gamecontroldata msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::Gamecontroldata>()
{
  return humanoid_interfaces::msg::builder::Init_Gamecontroldata_robotnum();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLDATA__BUILDER_HPP_
