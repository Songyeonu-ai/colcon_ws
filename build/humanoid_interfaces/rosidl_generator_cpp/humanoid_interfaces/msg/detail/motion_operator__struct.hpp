// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/MotionOperator.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MOTION_OPERATOR__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MOTION_OPERATOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__MotionOperator __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__MotionOperator __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotionOperator_
{
  using Type = MotionOperator_<ContainerAllocator>;

  explicit MotionOperator_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motion_num = 0l;
      this->motion_end = 0l;
    }
  }

  explicit MotionOperator_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motion_num = 0l;
      this->motion_end = 0l;
    }
  }

  // field types and members
  using _motion_num_type =
    int32_t;
  _motion_num_type motion_num;
  using _motion_end_type =
    int32_t;
  _motion_end_type motion_end;

  // setters for named parameter idiom
  Type & set__motion_num(
    const int32_t & _arg)
  {
    this->motion_num = _arg;
    return *this;
  }
  Type & set__motion_end(
    const int32_t & _arg)
  {
    this->motion_end = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::MotionOperator_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::MotionOperator_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::MotionOperator_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::MotionOperator_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::MotionOperator_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::MotionOperator_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::MotionOperator_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::MotionOperator_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::MotionOperator_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::MotionOperator_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__MotionOperator
    std::shared_ptr<humanoid_interfaces::msg::MotionOperator_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__MotionOperator
    std::shared_ptr<humanoid_interfaces::msg::MotionOperator_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotionOperator_ & other) const
  {
    if (this->motion_num != other.motion_num) {
      return false;
    }
    if (this->motion_end != other.motion_end) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotionOperator_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotionOperator_

// alias to use template instance with default allocator
using MotionOperator =
  humanoid_interfaces::msg::MotionOperator_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MOTION_OPERATOR__STRUCT_HPP_
