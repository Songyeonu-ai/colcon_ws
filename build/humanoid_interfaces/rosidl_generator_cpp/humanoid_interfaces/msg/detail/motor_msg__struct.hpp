// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/MotorMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MOTOR_MSG__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MOTOR_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__MotorMsg __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__MotorMsg __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorMsg_
{
  using Type = MotorMsg_<ContainerAllocator>;

  explicit MotorMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->length = 0l;
      this->mode = 0l;
    }
  }

  explicit MotorMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->length = 0l;
      this->mode = 0l;
    }
  }

  // field types and members
  using _length_type =
    int32_t;
  _length_type length;
  using _mode_type =
    int32_t;
  _mode_type mode;
  using _id_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _id_type id;
  using _speed_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _speed_type speed;
  using _position_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _position_type position;
  using _type_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _type_type type;
  using _acceleration_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _acceleration_type acceleration;

  // setters for named parameter idiom
  Type & set__length(
    const int32_t & _arg)
  {
    this->length = _arg;
    return *this;
  }
  Type & set__mode(
    const int32_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__id(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__speed(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__position(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__type(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__acceleration(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::MotorMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::MotorMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::MotorMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::MotorMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::MotorMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::MotorMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::MotorMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::MotorMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::MotorMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::MotorMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__MotorMsg
    std::shared_ptr<humanoid_interfaces::msg::MotorMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__MotorMsg
    std::shared_ptr<humanoid_interfaces::msg::MotorMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorMsg_ & other) const
  {
    if (this->length != other.length) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorMsg_

// alias to use template instance with default allocator
using MotorMsg =
  humanoid_interfaces::msg::MotorMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MOTOR_MSG__STRUCT_HPP_
