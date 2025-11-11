// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dynamixel_rdk_msgs:msg/DynamixelStatusMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_STATUS_MSGS__TRAITS_HPP_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_STATUS_MSGS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dynamixel_rdk_msgs/msg/detail/dynamixel_status_msgs__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace dynamixel_rdk_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DynamixelStatusMsgs & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: torque_enabled
  {
    out << "torque_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.torque_enabled, out);
    out << ", ";
  }

  // member: error_status
  {
    out << "error_status: ";
    rosidl_generator_traits::value_to_yaml(msg.error_status, out);
    out << ", ";
  }

  // member: present_position
  {
    out << "present_position: ";
    rosidl_generator_traits::value_to_yaml(msg.present_position, out);
    out << ", ";
  }

  // member: present_velocity
  {
    out << "present_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.present_velocity, out);
    out << ", ";
  }

  // member: present_acceleration
  {
    out << "present_acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.present_acceleration, out);
    out << ", ";
  }

  // member: present_current
  {
    out << "present_current: ";
    rosidl_generator_traits::value_to_yaml(msg.present_current, out);
    out << ", ";
  }

  // member: present_voltage
  {
    out << "present_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.present_voltage, out);
    out << ", ";
  }

  // member: present_temperature
  {
    out << "present_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.present_temperature, out);
    out << ", ";
  }

  // member: min_max_position
  {
    if (msg.min_max_position.size() == 0) {
      out << "min_max_position: []";
    } else {
      out << "min_max_position: [";
      size_t pending_items = msg.min_max_position.size();
      for (auto item : msg.min_max_position) {
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
  const DynamixelStatusMsgs & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: torque_enabled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torque_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.torque_enabled, out);
    out << "\n";
  }

  // member: error_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_status: ";
    rosidl_generator_traits::value_to_yaml(msg.error_status, out);
    out << "\n";
  }

  // member: present_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "present_position: ";
    rosidl_generator_traits::value_to_yaml(msg.present_position, out);
    out << "\n";
  }

  // member: present_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "present_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.present_velocity, out);
    out << "\n";
  }

  // member: present_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "present_acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.present_acceleration, out);
    out << "\n";
  }

  // member: present_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "present_current: ";
    rosidl_generator_traits::value_to_yaml(msg.present_current, out);
    out << "\n";
  }

  // member: present_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "present_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.present_voltage, out);
    out << "\n";
  }

  // member: present_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "present_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.present_temperature, out);
    out << "\n";
  }

  // member: min_max_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.min_max_position.size() == 0) {
      out << "min_max_position: []\n";
    } else {
      out << "min_max_position:\n";
      for (auto item : msg.min_max_position) {
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

inline std::string to_yaml(const DynamixelStatusMsgs & msg, bool use_flow_style = false)
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

}  // namespace dynamixel_rdk_msgs

namespace rosidl_generator_traits
{

[[deprecated("use dynamixel_rdk_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dynamixel_rdk_msgs::msg::DynamixelStatusMsgs & msg,
  std::ostream & out, size_t indentation = 0)
{
  dynamixel_rdk_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dynamixel_rdk_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dynamixel_rdk_msgs::msg::DynamixelStatusMsgs & msg)
{
  return dynamixel_rdk_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs>()
{
  return "dynamixel_rdk_msgs::msg::DynamixelStatusMsgs";
}

template<>
inline const char * name<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs>()
{
  return "dynamixel_rdk_msgs/msg/DynamixelStatusMsgs";
}

template<>
struct has_fixed_size<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_STATUS_MSGS__TRAITS_HPP_
