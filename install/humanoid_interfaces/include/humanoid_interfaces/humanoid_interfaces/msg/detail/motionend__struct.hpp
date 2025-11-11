// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/Motionend.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MOTIONEND__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MOTIONEND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__Motionend __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__Motionend __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Motionend_
{
  using Type = Motionend_<ContainerAllocator>;

  explicit Motionend_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motionend = 0l;
      this->motioning = 0l;
      this->motionstarting = 0l;
      this->motionending = 0l;
      this->motionnum = 0l;
    }
  }

  explicit Motionend_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motionend = 0l;
      this->motioning = 0l;
      this->motionstarting = 0l;
      this->motionending = 0l;
      this->motionnum = 0l;
    }
  }

  // field types and members
  using _motionend_type =
    int32_t;
  _motionend_type motionend;
  using _motioning_type =
    int32_t;
  _motioning_type motioning;
  using _motionstarting_type =
    int32_t;
  _motionstarting_type motionstarting;
  using _motionending_type =
    int32_t;
  _motionending_type motionending;
  using _motionnum_type =
    int32_t;
  _motionnum_type motionnum;

  // setters for named parameter idiom
  Type & set__motionend(
    const int32_t & _arg)
  {
    this->motionend = _arg;
    return *this;
  }
  Type & set__motioning(
    const int32_t & _arg)
  {
    this->motioning = _arg;
    return *this;
  }
  Type & set__motionstarting(
    const int32_t & _arg)
  {
    this->motionstarting = _arg;
    return *this;
  }
  Type & set__motionending(
    const int32_t & _arg)
  {
    this->motionending = _arg;
    return *this;
  }
  Type & set__motionnum(
    const int32_t & _arg)
  {
    this->motionnum = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::Motionend_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::Motionend_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Motionend_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Motionend_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Motionend_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Motionend_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Motionend_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Motionend_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Motionend_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Motionend_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__Motionend
    std::shared_ptr<humanoid_interfaces::msg::Motionend_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__Motionend
    std::shared_ptr<humanoid_interfaces::msg::Motionend_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Motionend_ & other) const
  {
    if (this->motionend != other.motionend) {
      return false;
    }
    if (this->motioning != other.motioning) {
      return false;
    }
    if (this->motionstarting != other.motionstarting) {
      return false;
    }
    if (this->motionending != other.motionending) {
      return false;
    }
    if (this->motionnum != other.motionnum) {
      return false;
    }
    return true;
  }
  bool operator!=(const Motionend_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Motionend_

// alias to use template instance with default allocator
using Motionend =
  humanoid_interfaces::msg::Motionend_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MOTIONEND__STRUCT_HPP_
