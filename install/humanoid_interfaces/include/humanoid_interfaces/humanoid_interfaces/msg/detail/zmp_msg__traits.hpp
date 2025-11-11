// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/ZmpMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ZMP_MSG__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__ZMP_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/zmp_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ZmpMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: left_x_zmp
  {
    out << "left_x_zmp: ";
    rosidl_generator_traits::value_to_yaml(msg.left_x_zmp, out);
    out << ", ";
  }

  // member: left_y_zmp
  {
    out << "left_y_zmp: ";
    rosidl_generator_traits::value_to_yaml(msg.left_y_zmp, out);
    out << ", ";
  }

  // member: right_x_zmp
  {
    out << "right_x_zmp: ";
    rosidl_generator_traits::value_to_yaml(msg.right_x_zmp, out);
    out << ", ";
  }

  // member: right_y_zmp
  {
    out << "right_y_zmp: ";
    rosidl_generator_traits::value_to_yaml(msg.right_y_zmp, out);
    out << ", ";
  }

  // member: total_x_zmp
  {
    out << "total_x_zmp: ";
    rosidl_generator_traits::value_to_yaml(msg.total_x_zmp, out);
    out << ", ";
  }

  // member: total_y_zmp
  {
    out << "total_y_zmp: ";
    rosidl_generator_traits::value_to_yaml(msg.total_y_zmp, out);
    out << ", ";
  }

  // member: left_foot
  {
    out << "left_foot: ";
    rosidl_generator_traits::value_to_yaml(msg.left_foot, out);
    out << ", ";
  }

  // member: right_foot
  {
    out << "right_foot: ";
    rosidl_generator_traits::value_to_yaml(msg.right_foot, out);
    out << ", ";
  }

  // member: both_feet
  {
    out << "both_feet: ";
    rosidl_generator_traits::value_to_yaml(msg.both_feet, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ZmpMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: left_x_zmp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_x_zmp: ";
    rosidl_generator_traits::value_to_yaml(msg.left_x_zmp, out);
    out << "\n";
  }

  // member: left_y_zmp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_y_zmp: ";
    rosidl_generator_traits::value_to_yaml(msg.left_y_zmp, out);
    out << "\n";
  }

  // member: right_x_zmp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_x_zmp: ";
    rosidl_generator_traits::value_to_yaml(msg.right_x_zmp, out);
    out << "\n";
  }

  // member: right_y_zmp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_y_zmp: ";
    rosidl_generator_traits::value_to_yaml(msg.right_y_zmp, out);
    out << "\n";
  }

  // member: total_x_zmp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_x_zmp: ";
    rosidl_generator_traits::value_to_yaml(msg.total_x_zmp, out);
    out << "\n";
  }

  // member: total_y_zmp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_y_zmp: ";
    rosidl_generator_traits::value_to_yaml(msg.total_y_zmp, out);
    out << "\n";
  }

  // member: left_foot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_foot: ";
    rosidl_generator_traits::value_to_yaml(msg.left_foot, out);
    out << "\n";
  }

  // member: right_foot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_foot: ";
    rosidl_generator_traits::value_to_yaml(msg.right_foot, out);
    out << "\n";
  }

  // member: both_feet
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "both_feet: ";
    rosidl_generator_traits::value_to_yaml(msg.both_feet, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ZmpMsg & msg, bool use_flow_style = false)
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
  const humanoid_interfaces::msg::ZmpMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::ZmpMsg & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::ZmpMsg>()
{
  return "humanoid_interfaces::msg::ZmpMsg";
}

template<>
inline const char * name<humanoid_interfaces::msg::ZmpMsg>()
{
  return "humanoid_interfaces/msg/ZmpMsg";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::ZmpMsg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::ZmpMsg>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<humanoid_interfaces::msg::ZmpMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ZMP_MSG__TRAITS_HPP_
