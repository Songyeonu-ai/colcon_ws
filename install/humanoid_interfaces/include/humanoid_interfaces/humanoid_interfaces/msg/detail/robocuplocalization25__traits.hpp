// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/Robocuplocalization25.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPLOCALIZATION25__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPLOCALIZATION25__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/robocuplocalization25__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Robocuplocalization25 & msg,
  std::ostream & out)
{
  out << "{";
  // member: ball_x
  {
    out << "ball_x: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_x, out);
    out << ", ";
  }

  // member: ball_y
  {
    out << "ball_y: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_y, out);
    out << ", ";
  }

  // member: ball_speed_x
  {
    out << "ball_speed_x: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_speed_x, out);
    out << ", ";
  }

  // member: ball_speed_y
  {
    out << "ball_speed_y: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_speed_y, out);
    out << ", ";
  }

  // member: robot_x
  {
    out << "robot_x: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_x, out);
    out << ", ";
  }

  // member: robot_y
  {
    out << "robot_y: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_y, out);
    out << ", ";
  }

  // member: obstacle_x
  {
    out << "obstacle_x: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle_x, out);
    out << ", ";
  }

  // member: obstacle_y
  {
    out << "obstacle_y: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle_y, out);
    out << ", ";
  }

  // member: obstacle0_x
  {
    out << "obstacle0_x: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle0_x, out);
    out << ", ";
  }

  // member: obstacle0_y
  {
    out << "obstacle0_y: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle0_y, out);
    out << ", ";
  }

  // member: obstacle1_x
  {
    out << "obstacle1_x: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle1_x, out);
    out << ", ";
  }

  // member: obstacle1_y
  {
    out << "obstacle1_y: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle1_y, out);
    out << ", ";
  }

  // member: obstacle2_x
  {
    out << "obstacle2_x: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle2_x, out);
    out << ", ";
  }

  // member: obstacle2_y
  {
    out << "obstacle2_y: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle2_y, out);
    out << ", ";
  }

  // member: obstacle3_x
  {
    out << "obstacle3_x: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle3_x, out);
    out << ", ";
  }

  // member: obstacle3_y
  {
    out << "obstacle3_y: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle3_y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Robocuplocalization25 & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ball_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ball_x: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_x, out);
    out << "\n";
  }

  // member: ball_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ball_y: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_y, out);
    out << "\n";
  }

  // member: ball_speed_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ball_speed_x: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_speed_x, out);
    out << "\n";
  }

  // member: ball_speed_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ball_speed_y: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_speed_y, out);
    out << "\n";
  }

  // member: robot_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_x: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_x, out);
    out << "\n";
  }

  // member: robot_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_y: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_y, out);
    out << "\n";
  }

  // member: obstacle_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obstacle_x: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle_x, out);
    out << "\n";
  }

  // member: obstacle_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obstacle_y: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle_y, out);
    out << "\n";
  }

  // member: obstacle0_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obstacle0_x: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle0_x, out);
    out << "\n";
  }

  // member: obstacle0_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obstacle0_y: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle0_y, out);
    out << "\n";
  }

  // member: obstacle1_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obstacle1_x: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle1_x, out);
    out << "\n";
  }

  // member: obstacle1_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obstacle1_y: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle1_y, out);
    out << "\n";
  }

  // member: obstacle2_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obstacle2_x: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle2_x, out);
    out << "\n";
  }

  // member: obstacle2_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obstacle2_y: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle2_y, out);
    out << "\n";
  }

  // member: obstacle3_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obstacle3_x: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle3_x, out);
    out << "\n";
  }

  // member: obstacle3_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obstacle3_y: ";
    rosidl_generator_traits::value_to_yaml(msg.obstacle3_y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Robocuplocalization25 & msg, bool use_flow_style = false)
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
  const humanoid_interfaces::msg::Robocuplocalization25 & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::Robocuplocalization25 & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::Robocuplocalization25>()
{
  return "humanoid_interfaces::msg::Robocuplocalization25";
}

template<>
inline const char * name<humanoid_interfaces::msg::Robocuplocalization25>()
{
  return "humanoid_interfaces/msg/Robocuplocalization25";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::Robocuplocalization25>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::Robocuplocalization25>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<humanoid_interfaces::msg::Robocuplocalization25>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPLOCALIZATION25__TRAITS_HPP_
