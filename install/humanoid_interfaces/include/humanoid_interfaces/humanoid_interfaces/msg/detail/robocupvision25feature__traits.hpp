// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from humanoid_interfaces:msg/Robocupvision25feature.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25FEATURE__TRAITS_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25FEATURE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "humanoid_interfaces/msg/detail/robocupvision25feature__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace humanoid_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Robocupvision25feature & msg,
  std::ostream & out)
{
  out << "{";
  // member: confidence
  {
    if (msg.confidence.size() == 0) {
      out << "confidence: []";
    } else {
      out << "confidence: [";
      size_t pending_items = msg.confidence.size();
      for (auto item : msg.confidence) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: distance
  {
    if (msg.distance.size() == 0) {
      out << "distance: []";
    } else {
      out << "distance: [";
      size_t pending_items = msg.distance.size();
      for (auto item : msg.distance) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: point_vec_x
  {
    if (msg.point_vec_x.size() == 0) {
      out << "point_vec_x: []";
    } else {
      out << "point_vec_x: [";
      size_t pending_items = msg.point_vec_x.size();
      for (auto item : msg.point_vec_x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: point_vec_y
  {
    if (msg.point_vec_y.size() == 0) {
      out << "point_vec_y: []";
    } else {
      out << "point_vec_y: [";
      size_t pending_items = msg.point_vec_y.size();
      for (auto item : msg.point_vec_y) {
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
  const Robocupvision25feature & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.confidence.size() == 0) {
      out << "confidence: []\n";
    } else {
      out << "confidence:\n";
      for (auto item : msg.confidence) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.distance.size() == 0) {
      out << "distance: []\n";
    } else {
      out << "distance:\n";
      for (auto item : msg.distance) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: point_vec_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.point_vec_x.size() == 0) {
      out << "point_vec_x: []\n";
    } else {
      out << "point_vec_x:\n";
      for (auto item : msg.point_vec_x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: point_vec_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.point_vec_y.size() == 0) {
      out << "point_vec_y: []\n";
    } else {
      out << "point_vec_y:\n";
      for (auto item : msg.point_vec_y) {
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

inline std::string to_yaml(const Robocupvision25feature & msg, bool use_flow_style = false)
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
  const humanoid_interfaces::msg::Robocupvision25feature & msg,
  std::ostream & out, size_t indentation = 0)
{
  humanoid_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use humanoid_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const humanoid_interfaces::msg::Robocupvision25feature & msg)
{
  return humanoid_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<humanoid_interfaces::msg::Robocupvision25feature>()
{
  return "humanoid_interfaces::msg::Robocupvision25feature";
}

template<>
inline const char * name<humanoid_interfaces::msg::Robocupvision25feature>()
{
  return "humanoid_interfaces/msg/Robocupvision25feature";
}

template<>
struct has_fixed_size<humanoid_interfaces::msg::Robocupvision25feature>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<humanoid_interfaces::msg::Robocupvision25feature>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<humanoid_interfaces::msg::Robocupvision25feature>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25FEATURE__TRAITS_HPP_
