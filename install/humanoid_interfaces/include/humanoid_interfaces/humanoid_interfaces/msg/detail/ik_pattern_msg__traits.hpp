// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/IkPatternMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_PATTERN_MSG__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_PATTERN_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/ik_pattern_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const IkPatternMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: xlpattern
  {
    out << "xlpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.xlpattern, out);
    out << ", ";
  }

  // member: ylpattern
  {
    out << "ylpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.ylpattern, out);
    out << ", ";
  }

  // member: zlpattern
  {
    out << "zlpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.zlpattern, out);
    out << ", ";
  }

  // member: slpattern
  {
    out << "slpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.slpattern, out);
    out << ", ";
  }

  // member: tlpattern
  {
    out << "tlpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.tlpattern, out);
    out << ", ";
  }

  // member: xrpattern
  {
    out << "xrpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.xrpattern, out);
    out << ", ";
  }

  // member: yrpattern
  {
    out << "yrpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.yrpattern, out);
    out << ", ";
  }

  // member: zrpattern
  {
    out << "zrpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.zrpattern, out);
    out << ", ";
  }

  // member: srpattern
  {
    out << "srpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.srpattern, out);
    out << ", ";
  }

  // member: trpattern
  {
    out << "trpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.trpattern, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IkPatternMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: xlpattern
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "xlpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.xlpattern, out);
    out << "\n";
  }

  // member: ylpattern
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ylpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.ylpattern, out);
    out << "\n";
  }

  // member: zlpattern
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "zlpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.zlpattern, out);
    out << "\n";
  }

  // member: slpattern
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "slpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.slpattern, out);
    out << "\n";
  }

  // member: tlpattern
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tlpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.tlpattern, out);
    out << "\n";
  }

  // member: xrpattern
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "xrpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.xrpattern, out);
    out << "\n";
  }

  // member: yrpattern
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yrpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.yrpattern, out);
    out << "\n";
  }

  // member: zrpattern
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "zrpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.zrpattern, out);
    out << "\n";
  }

  // member: srpattern
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "srpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.srpattern, out);
    out << "\n";
  }

  // member: trpattern
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trpattern: ";
    rosidl_generator_traits::value_to_yaml(msg.trpattern, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IkPatternMsg & msg, bool use_flow_style = false)
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
  const humanoid_interfaces::msg::IkPatternMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::IkPatternMsg & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::IkPatternMsg>()
{
  return "humanoid_interfaces::msg::IkPatternMsg";
}

template<>
inline const char * name<humanoid_interfaces::msg::IkPatternMsg>()
{
  return "humanoid_interfaces/msg/IkPatternMsg";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::IkPatternMsg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::IkPatternMsg>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<humanoid_interfaces::msg::IkPatternMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_PATTERN_MSG__TRAITS_HPP_
