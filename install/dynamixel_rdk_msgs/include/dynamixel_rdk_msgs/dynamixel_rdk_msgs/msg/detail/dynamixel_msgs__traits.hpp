// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dynamixel_rdk_msgs:msg/DynamixelMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_MSGS__TRAITS_HPP_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_MSGS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dynamixel_rdk_msgs/msg/detail/dynamixel_msgs__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace dynamixel_rdk_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DynamixelMsgs & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: goal_position
  {
    out << "goal_position: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_position, out);
    out << ", ";
  }

  // member: profile_acceleration
  {
    out << "profile_acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.profile_acceleration, out);
    out << ", ";
  }

  // member: profile_velocity
  {
    out << "profile_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.profile_velocity, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DynamixelMsgs & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: goal_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_position: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_position, out);
    out << "\n";
  }

  // member: profile_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "profile_acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.profile_acceleration, out);
    out << "\n";
  }

  // member: profile_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "profile_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.profile_velocity, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DynamixelMsgs & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace dynamixel_rdk_msgs

namespace rosidl_generator_traits
{

[[deprecated("use dynamixel_rdk_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dynamixel_rdk_msgs::msg::DynamixelMsgs & msg,
  std::ostream & out, size_t indentation = 0)
{
  dynamixel_rdk_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dynamixel_rdk_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dynamixel_rdk_msgs::msg::DynamixelMsgs & msg)
{
  return dynamixel_rdk_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dynamixel_rdk_msgs::msg::DynamixelMsgs>()
{
  return "dynamixel_rdk_msgs::msg::DynamixelMsgs";
}

template<>
inline const char * name<dynamixel_rdk_msgs::msg::DynamixelMsgs>()
{
  return "dynamixel_rdk_msgs/msg/DynamixelMsgs";
}

template<>
struct has_fixed_size<dynamixel_rdk_msgs::msg::DynamixelMsgs>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<dynamixel_rdk_msgs::msg::DynamixelMsgs>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<dynamixel_rdk_msgs::msg::DynamixelMsgs>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_MSGS__TRAITS_HPP_
