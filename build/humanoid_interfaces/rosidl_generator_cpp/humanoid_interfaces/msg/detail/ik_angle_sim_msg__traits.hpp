// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/IkAngleSimMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_ANGLE_SIM_MSG__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_ANGLE_SIM_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/ik_angle_sim_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const IkAngleSimMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: ang_l_0
  {
    out << "ang_l_0: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_0, out);
    out << ", ";
  }

  // member: ang_l_1
  {
    out << "ang_l_1: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_1, out);
    out << ", ";
  }

  // member: ang_l_2
  {
    out << "ang_l_2: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_2, out);
    out << ", ";
  }

  // member: ang_l_3
  {
    out << "ang_l_3: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_3, out);
    out << ", ";
  }

  // member: ang_l_4
  {
    out << "ang_l_4: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_4, out);
    out << ", ";
  }

  // member: ang_l_5
  {
    out << "ang_l_5: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_5, out);
    out << ", ";
  }

  // member: ang_l_6
  {
    out << "ang_l_6: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_6, out);
    out << ", ";
  }

  // member: ang_l_7
  {
    out << "ang_l_7: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_7, out);
    out << ", ";
  }

  // member: ang_l_8
  {
    out << "ang_l_8: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_8, out);
    out << ", ";
  }

  // member: ang_r_0
  {
    out << "ang_r_0: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_0, out);
    out << ", ";
  }

  // member: ang_r_1
  {
    out << "ang_r_1: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_1, out);
    out << ", ";
  }

  // member: ang_r_2
  {
    out << "ang_r_2: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_2, out);
    out << ", ";
  }

  // member: ang_r_3
  {
    out << "ang_r_3: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_3, out);
    out << ", ";
  }

  // member: ang_r_4
  {
    out << "ang_r_4: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_4, out);
    out << ", ";
  }

  // member: ang_r_5
  {
    out << "ang_r_5: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_5, out);
    out << ", ";
  }

  // member: ang_r_6
  {
    out << "ang_r_6: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_6, out);
    out << ", ";
  }

  // member: ang_r_7
  {
    out << "ang_r_7: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_7, out);
    out << ", ";
  }

  // member: ang_r_8
  {
    out << "ang_r_8: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_8, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IkAngleSimMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ang_l_0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_l_0: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_0, out);
    out << "\n";
  }

  // member: ang_l_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_l_1: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_1, out);
    out << "\n";
  }

  // member: ang_l_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_l_2: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_2, out);
    out << "\n";
  }

  // member: ang_l_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_l_3: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_3, out);
    out << "\n";
  }

  // member: ang_l_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_l_4: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_4, out);
    out << "\n";
  }

  // member: ang_l_5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_l_5: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_5, out);
    out << "\n";
  }

  // member: ang_l_6
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_l_6: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_6, out);
    out << "\n";
  }

  // member: ang_l_7
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_l_7: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_7, out);
    out << "\n";
  }

  // member: ang_l_8
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_l_8: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_l_8, out);
    out << "\n";
  }

  // member: ang_r_0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_r_0: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_0, out);
    out << "\n";
  }

  // member: ang_r_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_r_1: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_1, out);
    out << "\n";
  }

  // member: ang_r_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_r_2: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_2, out);
    out << "\n";
  }

  // member: ang_r_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_r_3: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_3, out);
    out << "\n";
  }

  // member: ang_r_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_r_4: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_4, out);
    out << "\n";
  }

  // member: ang_r_5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_r_5: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_5, out);
    out << "\n";
  }

  // member: ang_r_6
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_r_6: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_6, out);
    out << "\n";
  }

  // member: ang_r_7
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_r_7: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_7, out);
    out << "\n";
  }

  // member: ang_r_8
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ang_r_8: ";
    rosidl_generator_traits::value_to_yaml(msg.ang_r_8, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IkAngleSimMsg & msg, bool use_flow_style = false)
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
  const humanoid_interfaces::msg::IkAngleSimMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::IkAngleSimMsg & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::IkAngleSimMsg>()
{
  return "humanoid_interfaces::msg::IkAngleSimMsg";
}

template<>
inline const char * name<humanoid_interfaces::msg::IkAngleSimMsg>()
{
  return "humanoid_interfaces/msg/IkAngleSimMsg";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::IkAngleSimMsg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::IkAngleSimMsg>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<humanoid_interfaces::msg::IkAngleSimMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_ANGLE_SIM_MSG__TRAITS_HPP_
