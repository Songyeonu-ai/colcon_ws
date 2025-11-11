// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/CPWalk.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__CP_WALK__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__CP_WALK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/cp_walk__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_CPWalk_goal_dist
{
public:
  explicit Init_CPWalk_goal_dist(::humanoid_interfaces::msg::CPWalk & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::CPWalk goal_dist(::humanoid_interfaces::msg::CPWalk::_goal_dist_type arg)
  {
    msg_.goal_dist = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::CPWalk msg_;
};

class Init_CPWalk_start
{
public:
  explicit Init_CPWalk_start(::humanoid_interfaces::msg::CPWalk & msg)
  : msg_(msg)
  {}
  Init_CPWalk_goal_dist start(::humanoid_interfaces::msg::CPWalk::_start_type arg)
  {
    msg_.start = std::move(arg);
    return Init_CPWalk_goal_dist(msg_);
  }

private:
  ::humanoid_interfaces::msg::CPWalk msg_;
};

class Init_CPWalk_goal
{
public:
  explicit Init_CPWalk_goal(::humanoid_interfaces::msg::CPWalk & msg)
  : msg_(msg)
  {}
  Init_CPWalk_start goal(::humanoid_interfaces::msg::CPWalk::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return Init_CPWalk_start(msg_);
  }

private:
  ::humanoid_interfaces::msg::CPWalk msg_;
};

class Init_CPWalk_cpflag
{
public:
  explicit Init_CPWalk_cpflag(::humanoid_interfaces::msg::CPWalk & msg)
  : msg_(msg)
  {}
  Init_CPWalk_goal cpflag(::humanoid_interfaces::msg::CPWalk::_cpflag_type arg)
  {
    msg_.cpflag = std::move(arg);
    return Init_CPWalk_goal(msg_);
  }

private:
  ::humanoid_interfaces::msg::CPWalk msg_;
};

class Init_CPWalk_cpend
{
public:
  Init_CPWalk_cpend()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CPWalk_cpflag cpend(::humanoid_interfaces::msg::CPWalk::_cpend_type arg)
  {
    msg_.cpend = std::move(arg);
    return Init_CPWalk_cpflag(msg_);
  }

private:
  ::humanoid_interfaces::msg::CPWalk msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::CPWalk>()
{
  return humanoid_interfaces::msg::builder::Init_CPWalk_cpend();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__CP_WALK__BUILDER_HPP_
