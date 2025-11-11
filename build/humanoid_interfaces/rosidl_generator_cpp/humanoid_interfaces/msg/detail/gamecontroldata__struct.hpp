// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/Gamecontroldata.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLDATA__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLDATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__Gamecontroldata __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__Gamecontroldata __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Gamecontroldata_
{
  using Type = Gamecontroldata_<ContainerAllocator>;

  explicit Gamecontroldata_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robotnum = 0ll;
      this->position = 0ll;
      this->myteam = 0ll;
      this->state = 0ll;
      this->myside = 0ll;
      this->iskickoff = 0ll;
      this->readytime = 0ll;
      this->penalty = 0ll;
      this->secondstate = 0ll;
    }
  }

  explicit Gamecontroldata_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robotnum = 0ll;
      this->position = 0ll;
      this->myteam = 0ll;
      this->state = 0ll;
      this->myside = 0ll;
      this->iskickoff = 0ll;
      this->readytime = 0ll;
      this->penalty = 0ll;
      this->secondstate = 0ll;
    }
  }

  // field types and members
  using _robotnum_type =
    int64_t;
  _robotnum_type robotnum;
  using _position_type =
    int64_t;
  _position_type position;
  using _myteam_type =
    int64_t;
  _myteam_type myteam;
  using _state_type =
    int64_t;
  _state_type state;
  using _myside_type =
    int64_t;
  _myside_type myside;
  using _iskickoff_type =
    int64_t;
  _iskickoff_type iskickoff;
  using _readytime_type =
    int64_t;
  _readytime_type readytime;
  using _penalty_type =
    int64_t;
  _penalty_type penalty;
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
  Type & set__position(
    const int64_t & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__myteam(
    const int64_t & _arg)
  {
    this->myteam = _arg;
    return *this;
  }
  Type & set__state(
    const int64_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__myside(
    const int64_t & _arg)
  {
    this->myside = _arg;
    return *this;
  }
  Type & set__iskickoff(
    const int64_t & _arg)
  {
    this->iskickoff = _arg;
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
    humanoid_interfaces::msg::Gamecontroldata_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::Gamecontroldata_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Gamecontroldata_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Gamecontroldata_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Gamecontroldata_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Gamecontroldata_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Gamecontroldata_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Gamecontroldata_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Gamecontroldata_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Gamecontroldata_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__Gamecontroldata
    std::shared_ptr<humanoid_interfaces::msg::Gamecontroldata_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__Gamecontroldata
    std::shared_ptr<humanoid_interfaces::msg::Gamecontroldata_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Gamecontroldata_ & other) const
  {
    if (this->robotnum != other.robotnum) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->myteam != other.myteam) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->myside != other.myside) {
      return false;
    }
    if (this->iskickoff != other.iskickoff) {
      return false;
    }
    if (this->readytime != other.readytime) {
      return false;
    }
    if (this->penalty != other.penalty) {
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
  bool operator!=(const Gamecontroldata_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Gamecontroldata_

// alias to use template instance with default allocator
using Gamecontroldata =
  humanoid_interfaces::msg::Gamecontroldata_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLDATA__STRUCT_HPP_
