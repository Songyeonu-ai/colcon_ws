// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/IkLTCMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_LTC_MSG__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_LTC_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/ik_ltc_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const IkLTCMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: entire_time
  {
    out << "entire_time: ";
    rosidl_generator_traits::value_to_yaml(msg.entire_time, out);
    out << ", ";
  }

  // member: swing_gain_l
  {
    out << "swing_gain_l: ";
    rosidl_generator_traits::value_to_yaml(msg.swing_gain_l, out);
    out << ", ";
  }

  // member: swing_gain_r
  {
    out << "swing_gain_r: ";
    rosidl_generator_traits::value_to_yaml(msg.swing_gain_r, out);
    out << ", ";
  }

  // member: warning
  {
    out << "warning: ";
    rosidl_generator_traits::value_to_yaml(msg.warning, out);
    out << ", ";
  }

  // member: safe
  {
    out << "safe: ";
    rosidl_generator_traits::value_to_yaml(msg.safe, out);
    out << ", ";
  }

  // member: landing_time_l
  {
    out << "landing_time_l: ";
    rosidl_generator_traits::value_to_yaml(msg.landing_time_l, out);
    out << ", ";
  }

  // member: landing_time_r
  {
    out << "landing_time_r: ";
    rosidl_generator_traits::value_to_yaml(msg.landing_time_r, out);
    out << ", ";
  }

  // member: landing_error_l
  {
    out << "landing_error_l: ";
    rosidl_generator_traits::value_to_yaml(msg.landing_error_l, out);
    out << ", ";
  }

  // member: landing_error_r
  {
    out << "landing_error_r: ";
    rosidl_generator_traits::value_to_yaml(msg.landing_error_r, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IkLTCMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: entire_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "entire_time: ";
    rosidl_generator_traits::value_to_yaml(msg.entire_time, out);
    out << "\n";
  }

  // member: swing_gain_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "swing_gain_l: ";
    rosidl_generator_traits::value_to_yaml(msg.swing_gain_l, out);
    out << "\n";
  }

  // member: swing_gain_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "swing_gain_r: ";
    rosidl_generator_traits::value_to_yaml(msg.swing_gain_r, out);
    out << "\n";
  }

  // member: warning
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "warning: ";
    rosidl_generator_traits::value_to_yaml(msg.warning, out);
    out << "\n";
  }

  // member: safe
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safe: ";
    rosidl_generator_traits::value_to_yaml(msg.safe, out);
    out << "\n";
  }

  // member: landing_time_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "landing_time_l: ";
    rosidl_generator_traits::value_to_yaml(msg.landing_time_l, out);
    out << "\n";
  }

  // member: landing_time_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "landing_time_r: ";
    rosidl_generator_traits::value_to_yaml(msg.landing_time_r, out);
    out << "\n";
  }

  // member: landing_error_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "landing_error_l: ";
    rosidl_generator_traits::value_to_yaml(msg.landing_error_l, out);
    out << "\n";
  }

  // member: landing_error_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "landing_error_r: ";
    rosidl_generator_traits::value_to_yaml(msg.landing_error_r, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IkLTCMsg & msg, bool use_flow_style = false)
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
  const humanoid_interfaces::msg::IkLTCMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::IkLTCMsg & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::IkLTCMsg>()
{
  return "humanoid_interfaces::msg::IkLTCMsg";
}

template<>
inline const char * name<humanoid_interfaces::msg::IkLTCMsg>()
{
  return "humanoid_interfaces/msg/IkLTCMsg";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::IkLTCMsg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::IkLTCMsg>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<humanoid_interfaces::msg::IkLTCMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_LTC_MSG__TRAITS_HPP_
