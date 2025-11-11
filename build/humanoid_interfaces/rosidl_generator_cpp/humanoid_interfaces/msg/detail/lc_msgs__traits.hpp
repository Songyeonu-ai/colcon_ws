// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/LCMsgs.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__LC_MSGS__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__LC_MSGS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/lc_msgs__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const LCMsgs & msg,
  std::ostream & out)
{
  out << "{";
  // member: r_lc_data
  {
    if (msg.r_lc_data.size() == 0) {
      out << "r_lc_data: []";
    } else {
      out << "r_lc_data: [";
      size_t pending_items = msg.r_lc_data.size();
      for (auto item : msg.r_lc_data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: l_lc_data
  {
    if (msg.l_lc_data.size() == 0) {
      out << "l_lc_data: []";
    } else {
      out << "l_lc_data: [";
      size_t pending_items = msg.l_lc_data.size();
      for (auto item : msg.l_lc_data) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const LCMsgs & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: r_lc_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.r_lc_data.size() == 0) {
      out << "r_lc_data: []\n";
    } else {
      out << "r_lc_data:\n";
      for (auto item : msg.r_lc_data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: l_lc_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.l_lc_data.size() == 0) {
      out << "l_lc_data: []\n";
    } else {
      out << "l_lc_data:\n";
      for (auto item : msg.l_lc_data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LCMsgs & msg, bool use_flow_style = false)
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

}  // namespace humanoid_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use humanoid_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const humanoid_interfaces::msg::LCMsgs & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::LCMsgs & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::LCMsgs>()
{
  return "humanoid_interfaces::msg::LCMsgs";
}

template<>
inline const char * name<humanoid_interfaces::msg::LCMsgs>()
{
  return "humanoid_interfaces/msg/LCMsgs";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::LCMsgs>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::LCMsgs>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<humanoid_interfaces::msg::LCMsgs>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__LC_MSGS__TRAITS_HPP_
