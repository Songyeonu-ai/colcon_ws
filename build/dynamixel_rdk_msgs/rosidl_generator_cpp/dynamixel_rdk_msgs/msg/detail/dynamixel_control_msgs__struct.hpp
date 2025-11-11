// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dynamixel_rdk_msgs:msg/DynamixelControlMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_CONTROL_MSGS__STRUCT_HPP_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_CONTROL_MSGS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'motor_control'
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_msgs__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dynamixel_rdk_msgs__msg__DynamixelControlMsgs __attribute__((deprecated))
#else
# define DEPRECATED__dynamixel_rdk_msgs__msg__DynamixelControlMsgs __declspec(deprecated)
#endif

namespace dynamixel_rdk_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DynamixelControlMsgs_
{
  using Type = DynamixelControlMsgs_<ContainerAllocator>;

  explicit DynamixelControlMsgs_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit DynamixelControlMsgs_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _motor_control_type =
    std::vector<dynamixel_rdk_msgs::msg::DynamixelMsgs_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dynamixel_rdk_msgs::msg::DynamixelMsgs_<ContainerAllocator>>>;
  _motor_control_type motor_control;

  // setters for named parameter idiom
  Type & set__motor_control(
    const std::vector<dynamixel_rdk_msgs::msg::DynamixelMsgs_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dynamixel_rdk_msgs::msg::DynamixelMsgs_<ContainerAllocator>>> & _arg)
  {
    this->motor_control = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dynamixel_rdk_msgs::msg::DynamixelControlMsgs_<ContainerAllocator> *;
  using ConstRawPtr =
    const dynamixel_rdk_msgs::msg::DynamixelControlMsgs_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dynamixel_rdk_msgs::msg::DynamixelControlMsgs_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dynamixel_rdk_msgs::msg::DynamixelControlMsgs_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dynamixel_rdk_msgs::msg::DynamixelControlMsgs_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dynamixel_rdk_msgs::msg::DynamixelControlMsgs_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dynamixel_rdk_msgs::msg::DynamixelControlMsgs_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dynamixel_rdk_msgs::msg::DynamixelControlMsgs_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dynamixel_rdk_msgs::msg::DynamixelControlMsgs_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dynamixel_rdk_msgs::msg::DynamixelControlMsgs_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dynamixel_rdk_msgs__msg__DynamixelControlMsgs
    std::shared_ptr<dynamixel_rdk_msgs::msg::DynamixelControlMsgs_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dynamixel_rdk_msgs__msg__DynamixelControlMsgs
    std::shared_ptr<dynamixel_rdk_msgs::msg::DynamixelControlMsgs_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DynamixelControlMsgs_ & other) const
  {
    if (this->motor_control != other.motor_control) {
      return false;
    }
    return true;
  }
  bool operator!=(const DynamixelControlMsgs_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DynamixelControlMsgs_

// alias to use template instance with default allocator
using DynamixelControlMsgs =
  dynamixel_rdk_msgs::msg::DynamixelControlMsgs_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dynamixel_rdk_msgs

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_CONTROL_MSGS__STRUCT_HPP_
