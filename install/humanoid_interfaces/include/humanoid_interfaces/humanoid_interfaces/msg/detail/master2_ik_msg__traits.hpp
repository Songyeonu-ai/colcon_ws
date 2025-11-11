// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/Master2IkMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MASTER2_IK_MSG__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MASTER2_IK_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/master2_ik_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Master2IkMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: x_length
  {
    out << "x_length: ";
    rosidl_generator_traits::value_to_yaml(msg.x_length, out);
    out << ", ";
  }

  // member: y_length
  {
    out << "y_length: ";
    rosidl_generator_traits::value_to_yaml(msg.y_length, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: flag
  {
    out << "flag: ";
    rosidl_generator_traits::value_to_yaml(msg.flag, out);
    out << ", ";
  }

  // member: one_x_length
  {
    out << "one_x_length: ";
    rosidl_generator_traits::value_to_yaml(msg.one_x_length, out);
    out << ", ";
  }

  // member: one_y_length
  {
    out << "one_y_length: ";
    rosidl_generator_traits::value_to_yaml(msg.one_y_length, out);
    out << ", ";
  }

  // member: one_yaw
  {
    out << "one_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.one_yaw, out);
    out << ", ";
  }

  // member: one_step_flag
  {
    out << "one_step_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.one_step_flag, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Master2IkMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_length: ";
    rosidl_generator_traits::value_to_yaml(msg.x_length, out);
    out << "\n";
  }

  // member: y_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_length: ";
    rosidl_generator_traits::value_to_yaml(msg.y_length, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flag: ";
    rosidl_generator_traits::value_to_yaml(msg.flag, out);
    out << "\n";
  }

  // member: one_x_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "one_x_length: ";
    rosidl_generator_traits::value_to_yaml(msg.one_x_length, out);
    out << "\n";
  }

  // member: one_y_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "one_y_length: ";
    rosidl_generator_traits::value_to_yaml(msg.one_y_length, out);
    out << "\n";
  }

  // member: one_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "one_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.one_yaw, out);
    out << "\n";
  }

  // member: one_step_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "one_step_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.one_step_flag, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Master2IkMsg & msg, bool use_flow_style = false)
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
  const humanoid_interfaces::msg::Master2IkMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::Master2IkMsg & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::Master2IkMsg>()
{
  return "humanoid_interfaces::msg::Master2IkMsg";
}

template<>
inline const char * name<humanoid_interfaces::msg::Master2IkMsg>()
{
  return "humanoid_interfaces/msg/Master2IkMsg";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::Master2IkMsg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::Master2IkMsg>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<humanoid_interfaces::msg::Master2IkMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MASTER2_IK_MSG__TRAITS_HPP_
