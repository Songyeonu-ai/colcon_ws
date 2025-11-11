// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/Master2localization25.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MASTER2LOCALIZATION25__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MASTER2LOCALIZATION25__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/master2localization25__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Master2localization25 & msg,
  std::ostream & out)
{
  out << "{";
  // member: goalx
  {
    out << "goalx: ";
    rosidl_generator_traits::value_to_yaml(msg.goalx, out);
    out << ", ";
  }

  // member: goaly
  {
    out << "goaly: ";
    rosidl_generator_traits::value_to_yaml(msg.goaly, out);
    out << ", ";
  }

  // member: targetx
  {
    out << "targetx: ";
    rosidl_generator_traits::value_to_yaml(msg.targetx, out);
    out << ", ";
  }

  // member: targety
  {
    out << "targety: ";
    rosidl_generator_traits::value_to_yaml(msg.targety, out);
    out << ", ";
  }

  // member: robotallowanceerror
  {
    out << "robotallowanceerror: ";
    rosidl_generator_traits::value_to_yaml(msg.robotallowanceerror, out);
    out << ", ";
  }

  // member: robotcaliradius
  {
    out << "robotcaliradius: ";
    rosidl_generator_traits::value_to_yaml(msg.robotcaliradius, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Master2localization25 & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goalx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goalx: ";
    rosidl_generator_traits::value_to_yaml(msg.goalx, out);
    out << "\n";
  }

  // member: goaly
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goaly: ";
    rosidl_generator_traits::value_to_yaml(msg.goaly, out);
    out << "\n";
  }

  // member: targetx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "targetx: ";
    rosidl_generator_traits::value_to_yaml(msg.targetx, out);
    out << "\n";
  }

  // member: targety
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "targety: ";
    rosidl_generator_traits::value_to_yaml(msg.targety, out);
    out << "\n";
  }

  // member: robotallowanceerror
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robotallowanceerror: ";
    rosidl_generator_traits::value_to_yaml(msg.robotallowanceerror, out);
    out << "\n";
  }

  // member: robotcaliradius
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robotcaliradius: ";
    rosidl_generator_traits::value_to_yaml(msg.robotcaliradius, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Master2localization25 & msg, bool use_flow_style = false)
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
  const humanoid_interfaces::msg::Master2localization25 & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::Master2localization25 & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::Master2localization25>()
{
  return "humanoid_interfaces::msg::Master2localization25";
}

template<>
inline const char * name<humanoid_interfaces::msg::Master2localization25>()
{
  return "humanoid_interfaces/msg/Master2localization25";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::Master2localization25>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::Master2localization25>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<humanoid_interfaces::msg::Master2localization25>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MASTER2LOCALIZATION25__TRAITS_HPP_
