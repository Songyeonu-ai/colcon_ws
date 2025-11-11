// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/Robocupcontroller.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPCONTROLLER__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPCONTROLLER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/robocupcontroller__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Robocupcontroller & msg,
  std::ostream & out)
{
  out << "{";
  // member: robotnum
  {
    out << "robotnum: ";
    rosidl_generator_traits::value_to_yaml(msg.robotnum, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: readytime
  {
    out << "readytime: ";
    rosidl_generator_traits::value_to_yaml(msg.readytime, out);
    out << ", ";
  }

  // member: penalty
  {
    out << "penalty: ";
    rosidl_generator_traits::value_to_yaml(msg.penalty, out);
    out << ", ";
  }

  // member: firsthalf
  {
    out << "firsthalf: ";
    rosidl_generator_traits::value_to_yaml(msg.firsthalf, out);
    out << ", ";
  }

  // member: firstside
  {
    out << "firstside: ";
    rosidl_generator_traits::value_to_yaml(msg.firstside, out);
    out << ", ";
  }

  // member: kickoffteam
  {
    out << "kickoffteam: ";
    rosidl_generator_traits::value_to_yaml(msg.kickoffteam, out);
    out << ", ";
  }

  // member: secondstate
  {
    out << "secondstate: ";
    rosidl_generator_traits::value_to_yaml(msg.secondstate, out);
    out << ", ";
  }

  // member: secondinfo
  {
    if (msg.secondinfo.size() == 0) {
      out << "secondinfo: []";
    } else {
      out << "secondinfo: [";
      size_t pending_items = msg.secondinfo.size();
      for (auto item : msg.secondinfo) {
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
  const Robocupcontroller & msg,
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

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: readytime
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "readytime: ";
    rosidl_generator_traits::value_to_yaml(msg.readytime, out);
    out << "\n";
  }

  // member: penalty
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "penalty: ";
    rosidl_generator_traits::value_to_yaml(msg.penalty, out);
    out << "\n";
  }

  // member: firsthalf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "firsthalf: ";
    rosidl_generator_traits::value_to_yaml(msg.firsthalf, out);
    out << "\n";
  }

  // member: firstside
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "firstside: ";
    rosidl_generator_traits::value_to_yaml(msg.firstside, out);
    out << "\n";
  }

  // member: kickoffteam
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kickoffteam: ";
    rosidl_generator_traits::value_to_yaml(msg.kickoffteam, out);
    out << "\n";
  }

  // member: secondstate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "secondstate: ";
    rosidl_generator_traits::value_to_yaml(msg.secondstate, out);
    out << "\n";
  }

  // member: secondinfo
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.secondinfo.size() == 0) {
      out << "secondinfo: []\n";
    } else {
      out << "secondinfo:\n";
      for (auto item : msg.secondinfo) {
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

inline std::string to_yaml(const Robocupcontroller & msg, bool use_flow_style = false)
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
  const humanoid_interfaces::msg::Robocupcontroller & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::Robocupcontroller & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::Robocupcontroller>()
{
  return "humanoid_interfaces::msg::Robocupcontroller";
}

template<>
inline const char * name<humanoid_interfaces::msg::Robocupcontroller>()
{
  return "humanoid_interfaces/msg/Robocupcontroller";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::Robocupcontroller>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::Robocupcontroller>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<humanoid_interfaces::msg::Robocupcontroller>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPCONTROLLER__TRAITS_HPP_
