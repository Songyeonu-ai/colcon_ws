// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/CPWalk.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__CP_WALK__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__CP_WALK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/cp_walk__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const CPWalk & msg,
  std::ostream & out)
{
  out << "{";
  // member: cpend
  {
    out << "cpend: ";
    rosidl_generator_traits::value_to_yaml(msg.cpend, out);
    out << ", ";
  }

  // member: cpflag
  {
    out << "cpflag: ";
    rosidl_generator_traits::value_to_yaml(msg.cpflag, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    rosidl_generator_traits::value_to_yaml(msg.goal, out);
    out << ", ";
  }

  // member: start
  {
    out << "start: ";
    rosidl_generator_traits::value_to_yaml(msg.start, out);
    out << ", ";
  }

  // member: goal_dist
  {
    out << "goal_dist: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_dist, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CPWalk & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cpend
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cpend: ";
    rosidl_generator_traits::value_to_yaml(msg.cpend, out);
    out << "\n";
  }

  // member: cpflag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cpflag: ";
    rosidl_generator_traits::value_to_yaml(msg.cpflag, out);
    out << "\n";
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal: ";
    rosidl_generator_traits::value_to_yaml(msg.goal, out);
    out << "\n";
  }

  // member: start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start: ";
    rosidl_generator_traits::value_to_yaml(msg.start, out);
    out << "\n";
  }

  // member: goal_dist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_dist: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_dist, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CPWalk & msg, bool use_flow_style = false)
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
  const humanoid_interfaces::msg::CPWalk & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::CPWalk & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::CPWalk>()
{
  return "humanoid_interfaces::msg::CPWalk";
}

template<>
inline const char * name<humanoid_interfaces::msg::CPWalk>()
{
  return "humanoid_interfaces/msg/CPWalk";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::CPWalk>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::CPWalk>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<humanoid_interfaces::msg::CPWalk>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__CP_WALK__TRAITS_HPP_
