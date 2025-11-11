// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/IkPatternMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_PATTERN_MSG__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_PATTERN_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__IkPatternMsg __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__IkPatternMsg __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct IkPatternMsg_
{
  using Type = IkPatternMsg_<ContainerAllocator>;

  explicit IkPatternMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->xlpattern = 0.0;
      this->ylpattern = 0.0;
      this->zlpattern = 0.0;
      this->slpattern = 0.0;
      this->tlpattern = 0.0;
      this->xrpattern = 0.0;
      this->yrpattern = 0.0;
      this->zrpattern = 0.0;
      this->srpattern = 0.0;
      this->trpattern = 0.0;
    }
  }

  explicit IkPatternMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->xlpattern = 0.0;
      this->ylpattern = 0.0;
      this->zlpattern = 0.0;
      this->slpattern = 0.0;
      this->tlpattern = 0.0;
      this->xrpattern = 0.0;
      this->yrpattern = 0.0;
      this->zrpattern = 0.0;
      this->srpattern = 0.0;
      this->trpattern = 0.0;
    }
  }

  // field types and members
  using _xlpattern_type =
    double;
  _xlpattern_type xlpattern;
  using _ylpattern_type =
    double;
  _ylpattern_type ylpattern;
  using _zlpattern_type =
    double;
  _zlpattern_type zlpattern;
  using _slpattern_type =
    double;
  _slpattern_type slpattern;
  using _tlpattern_type =
    double;
  _tlpattern_type tlpattern;
  using _xrpattern_type =
    double;
  _xrpattern_type xrpattern;
  using _yrpattern_type =
    double;
  _yrpattern_type yrpattern;
  using _zrpattern_type =
    double;
  _zrpattern_type zrpattern;
  using _srpattern_type =
    double;
  _srpattern_type srpattern;
  using _trpattern_type =
    double;
  _trpattern_type trpattern;

  // setters for named parameter idiom
  Type & set__xlpattern(
    const double & _arg)
  {
    this->xlpattern = _arg;
    return *this;
  }
  Type & set__ylpattern(
    const double & _arg)
  {
    this->ylpattern = _arg;
    return *this;
  }
  Type & set__zlpattern(
    const double & _arg)
  {
    this->zlpattern = _arg;
    return *this;
  }
  Type & set__slpattern(
    const double & _arg)
  {
    this->slpattern = _arg;
    return *this;
  }
  Type & set__tlpattern(
    const double & _arg)
  {
    this->tlpattern = _arg;
    return *this;
  }
  Type & set__xrpattern(
    const double & _arg)
  {
    this->xrpattern = _arg;
    return *this;
  }
  Type & set__yrpattern(
    const double & _arg)
  {
    this->yrpattern = _arg;
    return *this;
  }
  Type & set__zrpattern(
    const double & _arg)
  {
    this->zrpattern = _arg;
    return *this;
  }
  Type & set__srpattern(
    const double & _arg)
  {
    this->srpattern = _arg;
    return *this;
  }
  Type & set__trpattern(
    const double & _arg)
  {
    this->trpattern = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::IkPatternMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::IkPatternMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::IkPatternMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::IkPatternMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::IkPatternMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::IkPatternMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::IkPatternMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::IkPatternMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::IkPatternMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::IkPatternMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__IkPatternMsg
    std::shared_ptr<humanoid_interfaces::msg::IkPatternMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__IkPatternMsg
    std::shared_ptr<humanoid_interfaces::msg::IkPatternMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IkPatternMsg_ & other) const
  {
    if (this->xlpattern != other.xlpattern) {
      return false;
    }
    if (this->ylpattern != other.ylpattern) {
      return false;
    }
    if (this->zlpattern != other.zlpattern) {
      return false;
    }
    if (this->slpattern != other.slpattern) {
      return false;
    }
    if (this->tlpattern != other.tlpattern) {
      return false;
    }
    if (this->xrpattern != other.xrpattern) {
      return false;
    }
    if (this->yrpattern != other.yrpattern) {
      return false;
    }
    if (this->zrpattern != other.zrpattern) {
      return false;
    }
    if (this->srpattern != other.srpattern) {
      return false;
    }
    if (this->trpattern != other.trpattern) {
      return false;
    }
    return true;
  }
  bool operator!=(const IkPatternMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IkPatternMsg_

// alias to use template instance with default allocator
using IkPatternMsg =
  humanoid_interfaces::msg::IkPatternMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_PATTERN_MSG__STRUCT_HPP_
