// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from humanoid_interfaces:msg/Master2localization25.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MASTER2LOCALIZATION25__BUILDER_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MASTER2LOCALIZATION25__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "humanoid_interfaces/msg/detail/master2localization25__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace humanoid_interfaces
{

namespace msg
{

namespace builder
{

class Init_Master2localization25_robotcaliradius
{
public:
  explicit Init_Master2localization25_robotcaliradius(::humanoid_interfaces::msg::Master2localization25 & msg)
  : msg_(msg)
  {}
  ::humanoid_interfaces::msg::Master2localization25 robotcaliradius(::humanoid_interfaces::msg::Master2localization25::_robotcaliradius_type arg)
  {
    msg_.robotcaliradius = std::move(arg);
    return std::move(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2localization25 msg_;
};

class Init_Master2localization25_robotallowanceerror
{
public:
  explicit Init_Master2localization25_robotallowanceerror(::humanoid_interfaces::msg::Master2localization25 & msg)
  : msg_(msg)
  {}
  Init_Master2localization25_robotcaliradius robotallowanceerror(::humanoid_interfaces::msg::Master2localization25::_robotallowanceerror_type arg)
  {
    msg_.robotallowanceerror = std::move(arg);
    return Init_Master2localization25_robotcaliradius(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2localization25 msg_;
};

class Init_Master2localization25_targety
{
public:
  explicit Init_Master2localization25_targety(::humanoid_interfaces::msg::Master2localization25 & msg)
  : msg_(msg)
  {}
  Init_Master2localization25_robotallowanceerror targety(::humanoid_interfaces::msg::Master2localization25::_targety_type arg)
  {
    msg_.targety = std::move(arg);
    return Init_Master2localization25_robotallowanceerror(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2localization25 msg_;
};

class Init_Master2localization25_targetx
{
public:
  explicit Init_Master2localization25_targetx(::humanoid_interfaces::msg::Master2localization25 & msg)
  : msg_(msg)
  {}
  Init_Master2localization25_targety targetx(::humanoid_interfaces::msg::Master2localization25::_targetx_type arg)
  {
    msg_.targetx = std::move(arg);
    return Init_Master2localization25_targety(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2localization25 msg_;
};

class Init_Master2localization25_goaly
{
public:
  explicit Init_Master2localization25_goaly(::humanoid_interfaces::msg::Master2localization25 & msg)
  : msg_(msg)
  {}
  Init_Master2localization25_targetx goaly(::humanoid_interfaces::msg::Master2localization25::_goaly_type arg)
  {
    msg_.goaly = std::move(arg);
    return Init_Master2localization25_targetx(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2localization25 msg_;
};

class Init_Master2localization25_goalx
{
public:
  Init_Master2localization25_goalx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Master2localization25_goaly goalx(::humanoid_interfaces::msg::Master2localization25::_goalx_type arg)
  {
    msg_.goalx = std::move(arg);
    return Init_Master2localization25_goaly(msg_);
  }

private:
  ::humanoid_interfaces::msg::Master2localization25 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::humanoid_interfaces::msg::Master2localization25>()
{
  return humanoid_interfaces::msg::builder::Init_Master2localization25_goalx();
}

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MASTER2LOCALIZATION25__BUILDER_HPP_
