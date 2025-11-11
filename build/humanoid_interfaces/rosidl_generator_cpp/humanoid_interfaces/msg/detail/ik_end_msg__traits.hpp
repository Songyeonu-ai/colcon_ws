// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/IkEndMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_END_MSG__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_END_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/ik_end_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const IkEndMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: ikend
  {
    out << "ikend: ";
    rosidl_generator_traits::value_to_yaml(msg.ikend, out);
    out << ", ";
  }

  // member: onestep_end
  {
    out << "onestep_end: ";
    rosidl_generator_traits::value_to_yaml(msg.onestep_end, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IkEndMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ikend
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ikend: ";
    rosidl_generator_traits::value_to_yaml(msg.ikend, out);
    out << "\n";
  }

  // member: onestep_end
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "onestep_end: ";
    rosidl_generator_traits::value_to_yaml(msg.onestep_end, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IkEndMsg & msg, bool use_flow_style = false)
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
  const humanoid_interfaces::msg::IkEndMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::IkEndMsg & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::IkEndMsg>()
{
  return "humanoid_interfaces::msg::IkEndMsg";
}

template<>
inline const char * name<humanoid_interfaces::msg::IkEndMsg>()
{
  return "humanoid_interfaces/msg/IkEndMsg";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::IkEndMsg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::IkEndMsg>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<humanoid_interfaces::msg::IkEndMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_END_MSG__TRAITS_HPP_
