// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dynamixel_rdk_msgs:msg/DynamixelControlMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_CONTROL_MSGS__TRAITS_HPP_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_CONTROL_MSGS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dynamixel_rdk_msgs/msg/detail/dynamixel_control_msgs__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'motor_control'
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_msgs__traits.hpp"

namespace dynamixel_rdk_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DynamixelControlMsgs & msg,
  std::ostream & out)
{
  out << "{";
  // member: motor_control
  {
    if (msg.motor_control.size() == 0) {
      out << "motor_control: []";
    } else {
      out << "motor_control: [";
      size_t pending_items = msg.motor_control.size();
      for (auto item : msg.motor_control) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DynamixelControlMsgs & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: motor_control
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.motor_control.size() == 0) {
      out << "motor_control: []\n";
    } else {
      out << "motor_control:\n";
      for (auto item : msg.motor_control) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DynamixelControlMsgs & msg, bool use_flow_style = false)
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
  const dynamixel_rdk_msgs::msg::DynamixelControlMsgs & msg,
  std::ostream & out, size_t indentation = 0)
{
  dynamixel_rdk_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dynamixel_rdk_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dynamixel_rdk_msgs::msg::DynamixelControlMsgs & msg)
{
  return dynamixel_rdk_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>()
{
  return "dynamixel_rdk_msgs::msg::DynamixelControlMsgs";
}

template<>
inline const char * name<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>()
{
  return "dynamixel_rdk_msgs/msg/DynamixelControlMsgs";
}

template<>
struct has_fixed_size<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_CONTROL_MSGS__TRAITS_HPP_
