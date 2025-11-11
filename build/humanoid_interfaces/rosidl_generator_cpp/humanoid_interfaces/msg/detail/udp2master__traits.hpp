// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/Udp2master.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__UDP2MASTER__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__UDP2MASTER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/udp2master__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Udp2master & msg,
  std::ostream & out)
{
  out << "{";
  // member: robotnum
  {
    out << "robotnum: ";
    rosidl_generator_traits::value_to_yaml(msg.robotnum, out);
    out << ", ";
  }

  // member: robotcase
  {
    out << "robotcase: ";
    rosidl_generator_traits::value_to_yaml(msg.robotcase, out);
    out << ", ";
  }

  // member: localx
  {
    out << "localx: ";
    rosidl_generator_traits::value_to_yaml(msg.localx, out);
    out << ", ";
  }

  // member: localy
  {
    out << "localy: ";
    rosidl_generator_traits::value_to_yaml(msg.localy, out);
    out << ", ";
  }

  // member: localyaw
  {
    out << "localyaw: ";
    rosidl_generator_traits::value_to_yaml(msg.localyaw, out);
    out << ", ";
  }

  // member: ballx
  {
    out << "ballx: ";
    rosidl_generator_traits::value_to_yaml(msg.ballx, out);
    out << ", ";
  }

  // member: bally
  {
    out << "bally: ";
    rosidl_generator_traits::value_to_yaml(msg.bally, out);
    out << ", ";
  }

  // member: balldist
  {
    out << "balldist: ";
    rosidl_generator_traits::value_to_yaml(msg.balldist, out);
    out << ", ";
  }

  // member: balltheta
  {
    out << "balltheta: ";
    rosidl_generator_traits::value_to_yaml(msg.balltheta, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Udp2master & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robotnum
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robotnum: ";
    rosidl_generator_traits::value_to_yaml(msg.robotnum, out);
    out << "\n";
  }

  // member: robotcase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robotcase: ";
    rosidl_generator_traits::value_to_yaml(msg.robotcase, out);
    out << "\n";
  }

  // member: localx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "localx: ";
    rosidl_generator_traits::value_to_yaml(msg.localx, out);
    out << "\n";
  }

  // member: localy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "localy: ";
    rosidl_generator_traits::value_to_yaml(msg.localy, out);
    out << "\n";
  }

  // member: localyaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "localyaw: ";
    rosidl_generator_traits::value_to_yaml(msg.localyaw, out);
    out << "\n";
  }

  // member: ballx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ballx: ";
    rosidl_generator_traits::value_to_yaml(msg.ballx, out);
    out << "\n";
  }

  // member: bally
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bally: ";
    rosidl_generator_traits::value_to_yaml(msg.bally, out);
    out << "\n";
  }

  // member: balldist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balldist: ";
    rosidl_generator_traits::value_to_yaml(msg.balldist, out);
    out << "\n";
  }

  // member: balltheta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "balltheta: ";
    rosidl_generator_traits::value_to_yaml(msg.balltheta, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Udp2master & msg, bool use_flow_style = false)
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
  const humanoid_interfaces::msg::Udp2master & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::Udp2master & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::Udp2master>()
{
  return "humanoid_interfaces::msg::Udp2master";
}

template<>
inline const char * name<humanoid_interfaces::msg::Udp2master>()
{
  return "humanoid_interfaces/msg/Udp2master";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::Udp2master>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::Udp2master>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<humanoid_interfaces::msg::Udp2master>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__UDP2MASTER__TRAITS_HPP_
