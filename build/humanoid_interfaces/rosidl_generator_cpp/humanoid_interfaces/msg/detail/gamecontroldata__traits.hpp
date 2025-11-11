// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/Gamecontroldata.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLDATA__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLDATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/gamecontroldata__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Gamecontroldata & msg,
  std::ostream & out)
{
  out << "{";
  // member: robotnum
  {
    out << "robotnum: ";
    rosidl_generator_traits::value_to_yaml(msg.robotnum, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << ", ";
  }

  // member: myteam
  {
    out << "myteam: ";
    rosidl_generator_traits::value_to_yaml(msg.myteam, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: myside
  {
    out << "myside: ";
    rosidl_generator_traits::value_to_yaml(msg.myside, out);
    out << ", ";
  }

  // member: iskickoff
  {
    out << "iskickoff: ";
    rosidl_generator_traits::value_to_yaml(msg.iskickoff, out);
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
  const Gamecontroldata & msg,
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

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
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

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: myside
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "myside: ";
    rosidl_generator_traits::value_to_yaml(msg.myside, out);
    out << "\n";
  }

  // member: iskickoff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "iskickoff: ";
    rosidl_generator_traits::value_to_yaml(msg.iskickoff, out);
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

inline std::string to_yaml(const Gamecontroldata & msg, bool use_flow_style = false)
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
  const humanoid_interfaces::msg::Gamecontroldata & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::Gamecontroldata & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::Gamecontroldata>()
{
  return "humanoid_interfaces::msg::Gamecontroldata";
}

template<>
inline const char * name<humanoid_interfaces::msg::Gamecontroldata>()
{
  return "humanoid_interfaces/msg/Gamecontroldata";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::Gamecontroldata>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::Gamecontroldata>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<humanoid_interfaces::msg::Gamecontroldata>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLDATA__TRAITS_HPP_
