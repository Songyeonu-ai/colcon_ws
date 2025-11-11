// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dynamixel_rdk_msgs:msg/DynamixelMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_MSGS__STRUCT_HPP_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_MSGS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dynamixel_rdk_msgs__msg__DynamixelMsgs __attribute__((deprecated))
#else
# define DEPRECATED__dynamixel_rdk_msgs__msg__DynamixelMsgs __declspec(deprecated)
#endif

namespace dynamixel_rdk_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DynamixelMsgs_
{
  using Type = DynamixelMsgs_<ContainerAllocator>;

  explicit DynamixelMsgs_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->goal_position = 0.0f;
      this->profile_acceleration = 0.0f;
      this->profile_velocity = 0.0f;
    }
  }

  explicit DynamixelMsgs_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->goal_position = 0.0f;
      this->profile_acceleration = 0.0f;
      this->profile_velocity = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _goal_position_type =
    float;
  _goal_position_type goal_position;
  using _profile_acceleration_type =
    float;
  _profile_acceleration_type profile_acceleration;
  using _profile_velocity_type =
    float;
  _profile_velocity_type profile_velocity;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__goal_position(
    const float & _arg)
  {
    this->goal_position = _arg;
    return *this;
  }
  Type & set__profile_acceleration(
    const float & _arg)
  {
    this->profile_acceleration = _arg;
    return *this;
  }
  Type & set__profile_velocity(
    const float & _arg)
  {
    this->profile_velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dynamixel_rdk_msgs::msg::DynamixelMsgs_<ContainerAllocator> *;
  using ConstRawPtr =
    const dynamixel_rdk_msgs::msg::DynamixelMsgs_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dynamixel_rdk_msgs::msg::DynamixelMsgs_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dynamixel_rdk_msgs::msg::DynamixelMsgs_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dynamixel_rdk_msgs::msg::DynamixelMsgs_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dynamixel_rdk_msgs::msg::DynamixelMsgs_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dynamixel_rdk_msgs::msg::DynamixelMsgs_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dynamixel_rdk_msgs::msg::DynamixelMsgs_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dynamixel_rdk_msgs::msg::DynamixelMsgs_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dynamixel_rdk_msgs::msg::DynamixelMsgs_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dynamixel_rdk_msgs__msg__DynamixelMsgs
    std::shared_ptr<dynamixel_rdk_msgs::msg::DynamixelMsgs_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dynamixel_rdk_msgs__msg__DynamixelMsgs
    std::shared_ptr<dynamixel_rdk_msgs::msg::DynamixelMsgs_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DynamixelMsgs_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->goal_position != other.goal_position) {
      return false;
    }
    if (this->profile_acceleration != other.profile_acceleration) {
      return false;
    }
    if (this->profile_velocity != other.profile_velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const DynamixelMsgs_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DynamixelMsgs_

// alias to use template instance with default allocator
using DynamixelMsgs =
  dynamixel_rdk_msgs::msg::DynamixelMsgs_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dynamixel_rdk_msgs

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_MSGS__STRUCT_HPP_
