// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/CPWalk.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__CP_WALK__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__CP_WALK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__CPWalk __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__CPWalk __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CPWalk_
{
  using Type = CPWalk_<ContainerAllocator>;

  explicit CPWalk_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cpend = 0l;
      this->cpflag = 0l;
      this->goal = 0.0;
      this->start = 0.0;
      this->goal_dist = 0.0;
    }
  }

  explicit CPWalk_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cpend = 0l;
      this->cpflag = 0l;
      this->goal = 0.0;
      this->start = 0.0;
      this->goal_dist = 0.0;
    }
  }

  // field types and members
  using _cpend_type =
    int32_t;
  _cpend_type cpend;
  using _cpflag_type =
    int32_t;
  _cpflag_type cpflag;
  using _goal_type =
    double;
  _goal_type goal;
  using _start_type =
    double;
  _start_type start;
  using _goal_dist_type =
    double;
  _goal_dist_type goal_dist;

  // setters for named parameter idiom
  Type & set__cpend(
    const int32_t & _arg)
  {
    this->cpend = _arg;
    return *this;
  }
  Type & set__cpflag(
    const int32_t & _arg)
  {
    this->cpflag = _arg;
    return *this;
  }
  Type & set__goal(
    const double & _arg)
  {
    this->goal = _arg;
    return *this;
  }
  Type & set__start(
    const double & _arg)
  {
    this->start = _arg;
    return *this;
  }
  Type & set__goal_dist(
    const double & _arg)
  {
    this->goal_dist = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::CPWalk_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::CPWalk_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::CPWalk_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::CPWalk_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::CPWalk_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::CPWalk_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::CPWalk_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::CPWalk_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::CPWalk_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::CPWalk_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__CPWalk
    std::shared_ptr<humanoid_interfaces::msg::CPWalk_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__CPWalk
    std::shared_ptr<humanoid_interfaces::msg::CPWalk_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CPWalk_ & other) const
  {
    if (this->cpend != other.cpend) {
      return false;
    }
    if (this->cpflag != other.cpflag) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    if (this->start != other.start) {
      return false;
    }
    if (this->goal_dist != other.goal_dist) {
      return false;
    }
    return true;
  }
  bool operator!=(const CPWalk_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CPWalk_

// alias to use template instance with default allocator
using CPWalk =
  humanoid_interfaces::msg::CPWalk_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__CP_WALK__STRUCT_HPP_
