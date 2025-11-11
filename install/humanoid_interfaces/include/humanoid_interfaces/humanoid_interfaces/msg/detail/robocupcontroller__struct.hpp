// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/Robocupcontroller.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPCONTROLLER__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPCONTROLLER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__Robocupcontroller __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__Robocupcontroller __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Robocupcontroller_
{
  using Type = Robocupcontroller_<ContainerAllocator>;

  explicit Robocupcontroller_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robotnum = 0ll;
      this->state = 0ll;
      this->readytime = 0ll;
      this->penalty = 0ll;
      this->firsthalf = 0ll;
      this->firstside = 0ll;
      this->kickoffteam = 0ll;
      this->secondstate = 0ll;
    }
  }

  explicit Robocupcontroller_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robotnum = 0ll;
      this->state = 0ll;
      this->readytime = 0ll;
      this->penalty = 0ll;
      this->firsthalf = 0ll;
      this->firstside = 0ll;
      this->kickoffteam = 0ll;
      this->secondstate = 0ll;
    }
  }

  // field types and members
  using _robotnum_type =
    int64_t;
  _robotnum_type robotnum;
  using _state_type =
    int64_t;
  _state_type state;
  using _readytime_type =
    int64_t;
  _readytime_type readytime;
  using _penalty_type =
    int64_t;
  _penalty_type penalty;
  using _firsthalf_type =
    int64_t;
  _firsthalf_type firsthalf;
  using _firstside_type =
    int64_t;
  _firstside_type firstside;
  using _kickoffteam_type =
    int64_t;
  _kickoffteam_type kickoffteam;
  using _secondstate_type =
    int64_t;
  _secondstate_type secondstate;
  using _secondinfo_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _secondinfo_type secondinfo;

  // setters for named parameter idiom
  Type & set__robotnum(
    const int64_t & _arg)
  {
    this->robotnum = _arg;
    return *this;
  }
  Type & set__state(
    const int64_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__readytime(
    const int64_t & _arg)
  {
    this->readytime = _arg;
    return *this;
  }
  Type & set__penalty(
    const int64_t & _arg)
  {
    this->penalty = _arg;
    return *this;
  }
  Type & set__firsthalf(
    const int64_t & _arg)
  {
    this->firsthalf = _arg;
    return *this;
  }
  Type & set__firstside(
    const int64_t & _arg)
  {
    this->firstside = _arg;
    return *this;
  }
  Type & set__kickoffteam(
    const int64_t & _arg)
  {
    this->kickoffteam = _arg;
    return *this;
  }
  Type & set__secondstate(
    const int64_t & _arg)
  {
    this->secondstate = _arg;
    return *this;
  }
  Type & set__secondinfo(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->secondinfo = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::Robocupcontroller_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::Robocupcontroller_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Robocupcontroller_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Robocupcontroller_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Robocupcontroller_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Robocupcontroller_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Robocupcontroller_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Robocupcontroller_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Robocupcontroller_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Robocupcontroller_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__Robocupcontroller
    std::shared_ptr<humanoid_interfaces::msg::Robocupcontroller_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__Robocupcontroller
    std::shared_ptr<humanoid_interfaces::msg::Robocupcontroller_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Robocupcontroller_ & other) const
  {
    if (this->robotnum != other.robotnum) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->readytime != other.readytime) {
      return false;
    }
    if (this->penalty != other.penalty) {
      return false;
    }
    if (this->firsthalf != other.firsthalf) {
      return false;
    }
    if (this->firstside != other.firstside) {
      return false;
    }
    if (this->kickoffteam != other.kickoffteam) {
      return false;
    }
    if (this->secondstate != other.secondstate) {
      return false;
    }
    if (this->secondinfo != other.secondinfo) {
      return false;
    }
    return true;
  }
  bool operator!=(const Robocupcontroller_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Robocupcontroller_

// alias to use template instance with default allocator
using Robocupcontroller =
  humanoid_interfaces::msg::Robocupcontroller_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPCONTROLLER__STRUCT_HPP_
