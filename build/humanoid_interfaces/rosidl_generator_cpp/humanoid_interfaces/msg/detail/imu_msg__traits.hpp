// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/ImuMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IMU_MSG__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IMU_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/imu_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ImuMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: q_x
  {
    out << "q_x: ";
    rosidl_generator_traits::value_to_yaml(msg.q_x, out);
    out << ", ";
  }

  // member: q_y
  {
    out << "q_y: ";
    rosidl_generator_traits::value_to_yaml(msg.q_y, out);
    out << ", ";
  }

  // member: q_z
  {
    out << "q_z: ";
    rosidl_generator_traits::value_to_yaml(msg.q_z, out);
    out << ", ";
  }

  // member: q_w
  {
    out << "q_w: ";
    rosidl_generator_traits::value_to_yaml(msg.q_w, out);
    out << ", ";
  }

  // member: x_gyro
  {
    out << "x_gyro: ";
    rosidl_generator_traits::value_to_yaml(msg.x_gyro, out);
    out << ", ";
  }

  // member: y_gyro
  {
    out << "y_gyro: ";
    rosidl_generator_traits::value_to_yaml(msg.y_gyro, out);
    out << ", ";
  }

  // member: z_gyro
  {
    out << "z_gyro: ";
    rosidl_generator_traits::value_to_yaml(msg.z_gyro, out);
    out << ", ";
  }

  // member: x_acc
  {
    out << "x_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.x_acc, out);
    out << ", ";
  }

  // member: y_acc
  {
    out << "y_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.y_acc, out);
    out << ", ";
  }

  // member: z_acc
  {
    out << "z_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.z_acc, out);
    out << ", ";
  }

  // member: x_vel
  {
    out << "x_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.x_vel, out);
    out << ", ";
  }

  // member: y_vel
  {
    out << "y_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.y_vel, out);
    out << ", ";
  }

  // member: z_vel
  {
    out << "z_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.z_vel, out);
    out << ", ";
  }

  // member: x_pos
  {
    out << "x_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.x_pos, out);
    out << ", ";
  }

  // member: y_pos
  {
    out << "y_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.y_pos, out);
    out << ", ";
  }

  // member: z_pos
  {
    out << "z_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.z_pos, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ImuMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: q_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q_x: ";
    rosidl_generator_traits::value_to_yaml(msg.q_x, out);
    out << "\n";
  }

  // member: q_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q_y: ";
    rosidl_generator_traits::value_to_yaml(msg.q_y, out);
    out << "\n";
  }

  // member: q_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q_z: ";
    rosidl_generator_traits::value_to_yaml(msg.q_z, out);
    out << "\n";
  }

  // member: q_w
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q_w: ";
    rosidl_generator_traits::value_to_yaml(msg.q_w, out);
    out << "\n";
  }

  // member: x_gyro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_gyro: ";
    rosidl_generator_traits::value_to_yaml(msg.x_gyro, out);
    out << "\n";
  }

  // member: y_gyro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_gyro: ";
    rosidl_generator_traits::value_to_yaml(msg.y_gyro, out);
    out << "\n";
  }

  // member: z_gyro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_gyro: ";
    rosidl_generator_traits::value_to_yaml(msg.z_gyro, out);
    out << "\n";
  }

  // member: x_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.x_acc, out);
    out << "\n";
  }

  // member: y_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.y_acc, out);
    out << "\n";
  }

  // member: z_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.z_acc, out);
    out << "\n";
  }

  // member: x_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.x_vel, out);
    out << "\n";
  }

  // member: y_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.y_vel, out);
    out << "\n";
  }

  // member: z_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.z_vel, out);
    out << "\n";
  }

  // member: x_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.x_pos, out);
    out << "\n";
  }

  // member: y_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.y_pos, out);
    out << "\n";
  }

  // member: z_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.z_pos, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ImuMsg & msg, bool use_flow_style = false)
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
  const humanoid_interfaces::msg::ImuMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::ImuMsg & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::ImuMsg>()
{
  return "humanoid_interfaces::msg::ImuMsg";
}

template<>
inline const char * name<humanoid_interfaces::msg::ImuMsg>()
{
  return "humanoid_interfaces/msg/ImuMsg";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::ImuMsg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::ImuMsg>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<humanoid_interfaces::msg::ImuMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IMU_MSG__TRAITS_HPP_
