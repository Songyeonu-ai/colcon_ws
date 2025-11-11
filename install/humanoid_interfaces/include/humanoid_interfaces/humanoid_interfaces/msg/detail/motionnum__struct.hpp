// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/Motionnum.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MOTIONNUM__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MOTIONNUM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__Motionnum __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__Motionnum __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Motionnum_
{
  using Type = Motionnum_<ContainerAllocator>;

  explicit Motionnum_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motionmode = 0l;
      this->motionnum = 0l;
    }
  }

  explicit Motionnum_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motionmode = 0l;
      this->motionnum = 0l;
    }
  }

  // field types and members
  using _motionmode_type =
    int32_t;
  _motionmode_type motionmode;
  using _motionnum_type =
    int32_t;
  _motionnum_type motionnum;

  // setters for named parameter idiom
  Type & set__motionmode(
    const int32_t & _arg)
  {
    this->motionmode = _arg;
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
    humanoid_interfaces::msg::Motionnum_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::Motionnum_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Motionnum_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Motionnum_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Motionnum_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Motionnum_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Motionnum_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Motionnum_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Motionnum_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Motionnum_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__Motionnum
    std::shared_ptr<humanoid_interfaces::msg::Motionnum_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__Motionnum
    std::shared_ptr<humanoid_interfaces::msg::Motionnum_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Motionnum_ & other) const
  {
    if (this->motionmode != other.motionmode) {
      return false;
    }
    if (this->motionnum != other.motionnum) {
      return false;
    }
    return true;
  }
  bool operator!=(const Motionnum_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Motionnum_

// alias to use template instance with default allocator
using Motionnum =
  humanoid_interfaces::msg::Motionnum_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MOTIONNUM__STRUCT_HPP_
