// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dynamixel_rdk_msgs:msg/DynamixelBulkReadMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_BULK_READ_MSGS__TRAITS_HPP_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_BULK_READ_MSGS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dynamixel_rdk_msgs/msg/detail/dynamixel_bulk_read_msgs__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'status_msgs'
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_status_msgs__traits.hpp"

namespace dynamixel_rdk_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DynamixelBulkReadMsgs & msg,
  std::ostream & out)
{
  out << "{";
  // member: status_msgs
  {
    if (msg.status_msgs.size() == 0) {
      out << "status_msgs: []";
    } else {
      out << "status_msgs: [";
      size_t pending_items = msg.status_msgs.size();
      for (auto item : msg.status_msgs) {
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
  const DynamixelBulkReadMsgs & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status_msgs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.status_msgs.size() == 0) {
      out << "status_msgs: []\n";
    } else {
      out << "status_msgs:\n";
      for (auto item : msg.status_msgs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DynamixelBulkReadMsgs & msg, bool use_flow_style = false)
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
  const dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs & msg,
  std::ostream & out, size_t indentation = 0)
{
  dynamixel_rdk_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dynamixel_rdk_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs & msg)
{
  return dynamixel_rdk_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs>()
{
  return "dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs";
}

template<>
inline const char * name<dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs>()
{
  return "dynamixel_rdk_msgs/msg/DynamixelBulkReadMsgs";
}

template<>
struct has_fixed_size<dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_BULK_READ_MSGS__TRAITS_HPP_
