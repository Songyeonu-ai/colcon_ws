// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/Master2udp.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MASTER2UDP__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MASTER2UDP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/master2udp__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Master2udp & msg,
  std::ostream & out)
{
  out << "{";
  // member: robotnum
  {
    out << "robotnum: ";
    rosidl_generator_traits::value_to_yaml(msg.robotnum, out);
    out << ", ";
  }

  // member: robotstate
  {
    out << "robotstate: ";
    rosidl_generator_traits::value_to_yaml(msg.robotstate, out);
    out << ", ";
  }

  // member: robotcoorx
  {
    out << "robotcoorx: ";
    rosidl_generator_traits::value_to_yaml(msg.robotcoorx, out);
    out << ", ";
  }

  // member: robotcoory
  {
    out << "robotcoory: ";
    rosidl_generator_traits::value_to_yaml(msg.robotcoory, out);
    out << ", ";
  }

  // member: robotimuyaw
  {
    out << "robotimuyaw: ";
    rosidl_generator_traits::value_to_yaml(msg.robotimuyaw, out);
    out << ", ";
  }

  // member: balldist
  {
    out << "balldist: ";
    rosidl_generator_traits::value_to_yaml(msg.balldist, out);
    out << ", ";
  }

  // member: ballcoorx
  {
    out << "ballcoorx: ";
    rosidl_generator_traits::value_to_yaml(msg.ballcoorx, out);
    out << ", ";
  }

  // member: ballcoory
  {
    out << "ballcoory: ";
    rosidl_generator_traits::value_to_yaml(msg.ballcoory, out);
    out << ", ";
  }

  // member: myteam
  {
    out << "myteam: ";
    rosidl_generator_traits::value_to_yaml(msg.myteam, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Master2udp & msg,
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

  // member: robotstate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robotstate: ";
    rosidl_generator_traits::value_to_yaml(msg.robotstate, out);
    out << "\n";
  }

  // member: robotcoorx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robotcoorx: ";
    rosidl_generator_traits::value_to_yaml(msg.robotcoorx, out);
    out << "\n";
  }

  // member: robotcoory
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robotcoory: ";
    rosidl_generator_traits::value_to_yaml(msg.robotcoory, out);
    out << "\n";
  }

  // member: robotimuyaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robotimuyaw: ";
    rosidl_generator_traits::value_to_yaml(msg.robotimuyaw, out);
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

  // member: ballcoorx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ballcoorx: ";
    rosidl_generator_traits::value_to_yaml(msg.ballcoorx, out);
    out << "\n";
  }

  // member: ballcoory
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ballcoory: ";
    rosidl_generator_traits::value_to_yaml(msg.ballcoory, out);
    out << "\n";
  }

  // member: myteam
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "myteam: ";
    rosidl_generator_traits::value_to_yaml(msg.myteam, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Master2udp & msg, bool use_flow_style = false)
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
  const humanoid_interfaces::msg::Master2udp & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::Master2udp & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::Master2udp>()
{
  return "humanoid_interfaces::msg::Master2udp";
}

template<>
inline const char * name<humanoid_interfaces::msg::Master2udp>()
{
  return "humanoid_interfaces/msg/Master2udp";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::Master2udp>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::Master2udp>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<humanoid_interfaces::msg::Master2udp>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MASTER2UDP__TRAITS_HPP_
