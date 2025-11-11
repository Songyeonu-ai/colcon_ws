// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/Robocupvision25.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/robocupvision25__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Robocupvision25 & msg,
  std::ostream & out)
{
  out << "{";
  // member: ball_cam_x
  {
    out << "ball_cam_x: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_cam_x, out);
    out << ", ";
  }

  // member: ball_cam_y
  {
    out << "ball_cam_y: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_cam_y, out);
    out << ", ";
  }

  // member: ball_2d_x
  {
    out << "ball_2d_x: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_2d_x, out);
    out << ", ";
  }

  // member: ball_2d_y
  {
    out << "ball_2d_y: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_2d_y, out);
    out << ", ";
  }

  // member: ball_d
  {
    out << "ball_d: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_d, out);
    out << ", ";
  }

  // member: pan
  {
    out << "pan: ";
    rosidl_generator_traits::value_to_yaml(msg.pan, out);
    out << ", ";
  }

  // member: tilt
  {
    out << "tilt: ";
    rosidl_generator_traits::value_to_yaml(msg.tilt, out);
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

  // member: robot_vec_x
  {
    if (msg.robot_vec_x.size() == 0) {
      out << "robot_vec_x: []";
    } else {
      out << "robot_vec_x: [";
      size_t pending_items = msg.robot_vec_x.size();
      for (auto item : msg.robot_vec_x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: robot_vec_y
  {
    if (msg.robot_vec_y.size() == 0) {
      out << "robot_vec_y: []";
    } else {
      out << "robot_vec_y: [";
      size_t pending_items = msg.robot_vec_y.size();
      for (auto item : msg.robot_vec_y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ball_speed_level
  {
    out << "ball_speed_level: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_speed_level, out);
    out << ", ";
  }

  // member: scan_mode
  {
    out << "scan_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.scan_mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Robocupvision25 & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ball_cam_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ball_cam_x: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_cam_x, out);
    out << "\n";
  }

  // member: ball_cam_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ball_cam_y: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_cam_y, out);
    out << "\n";
  }

  // member: ball_2d_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ball_2d_x: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_2d_x, out);
    out << "\n";
  }

  // member: ball_2d_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ball_2d_y: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_2d_y, out);
    out << "\n";
  }

  // member: ball_d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ball_d: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_d, out);
    out << "\n";
  }

  // member: pan
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pan: ";
    rosidl_generator_traits::value_to_yaml(msg.pan, out);
    out << "\n";
  }

  // member: tilt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tilt: ";
    rosidl_generator_traits::value_to_yaml(msg.tilt, out);
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

  // member: robot_vec_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.robot_vec_x.size() == 0) {
      out << "robot_vec_x: []\n";
    } else {
      out << "robot_vec_x:\n";
      for (auto item : msg.robot_vec_x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: robot_vec_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.robot_vec_y.size() == 0) {
      out << "robot_vec_y: []\n";
    } else {
      out << "robot_vec_y:\n";
      for (auto item : msg.robot_vec_y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ball_speed_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ball_speed_level: ";
    rosidl_generator_traits::value_to_yaml(msg.ball_speed_level, out);
    out << "\n";
  }

  // member: scan_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scan_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.scan_mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Robocupvision25 & msg, bool use_flow_style = false)
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
  const humanoid_interfaces::msg::Robocupvision25 & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::Robocupvision25 & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::Robocupvision25>()
{
  return "humanoid_interfaces::msg::Robocupvision25";
}

template<>
inline const char * name<humanoid_interfaces::msg::Robocupvision25>()
{
  return "humanoid_interfaces/msg/Robocupvision25";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::Robocupvision25>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::Robocupvision25>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<humanoid_interfaces::msg::Robocupvision25>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25__TRAITS_HPP_
