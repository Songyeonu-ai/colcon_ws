// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/Udp2master.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__UDP2MASTER__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__UDP2MASTER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__Udp2master __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__Udp2master __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Udp2master_
{
  using Type = Udp2master_<ContainerAllocator>;

  explicit Udp2master_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robotnum = 0ll;
      this->robotcase = 0ll;
      this->localx = 0ll;
      this->localy = 0ll;
      this->localyaw = 0ll;
      this->ballx = 0ll;
      this->bally = 0ll;
      this->balldist = 0ll;
      this->balltheta = 0ll;
    }
  }

  explicit Udp2master_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robotnum = 0ll;
      this->robotcase = 0ll;
      this->localx = 0ll;
      this->localy = 0ll;
      this->localyaw = 0ll;
      this->ballx = 0ll;
      this->bally = 0ll;
      this->balldist = 0ll;
      this->balltheta = 0ll;
    }
  }

  // field types and members
  using _robotnum_type =
    int64_t;
  _robotnum_type robotnum;
  using _robotcase_type =
    int64_t;
  _robotcase_type robotcase;
  using _localx_type =
    int64_t;
  _localx_type localx;
  using _localy_type =
    int64_t;
  _localy_type localy;
  using _localyaw_type =
    int64_t;
  _localyaw_type localyaw;
  using _ballx_type =
    int64_t;
  _ballx_type ballx;
  using _bally_type =
    int64_t;
  _bally_type bally;
  using _balldist_type =
    int64_t;
  _balldist_type balldist;
  using _balltheta_type =
    int64_t;
  _balltheta_type balltheta;

  // setters for named parameter idiom
  Type & set__robotnum(
    const int64_t & _arg)
  {
    this->robotnum = _arg;
    return *this;
  }
  Type & set__robotcase(
    const int64_t & _arg)
  {
    this->robotcase = _arg;
    return *this;
  }
  Type & set__localx(
    const int64_t & _arg)
  {
    this->localx = _arg;
    return *this;
  }
  Type & set__localy(
    const int64_t & _arg)
  {
    this->localy = _arg;
    return *this;
  }
  Type & set__localyaw(
    const int64_t & _arg)
  {
    this->localyaw = _arg;
    return *this;
  }
  Type & set__ballx(
    const int64_t & _arg)
  {
    this->ballx = _arg;
    return *this;
  }
  Type & set__bally(
    const int64_t & _arg)
  {
    this->bally = _arg;
    return *this;
  }
  Type & set__balldist(
    const int64_t & _arg)
  {
    this->balldist = _arg;
    return *this;
  }
  Type & set__balltheta(
    const int64_t & _arg)
  {
    this->balltheta = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::Udp2master_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::Udp2master_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Udp2master_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Udp2master_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Udp2master_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Udp2master_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Udp2master_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Udp2master_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Udp2master_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Udp2master_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__Udp2master
    std::shared_ptr<humanoid_interfaces::msg::Udp2master_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__Udp2master
    std::shared_ptr<humanoid_interfaces::msg::Udp2master_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Udp2master_ & other) const
  {
    if (this->robotnum != other.robotnum) {
      return false;
    }
    if (this->robotcase != other.robotcase) {
      return false;
    }
    if (this->localx != other.localx) {
      return false;
    }
    if (this->localy != other.localy) {
      return false;
    }
    if (this->localyaw != other.localyaw) {
      return false;
    }
    if (this->ballx != other.ballx) {
      return false;
    }
    if (this->bally != other.bally) {
      return false;
    }
    if (this->balldist != other.balldist) {
      return false;
    }
    if (this->balltheta != other.balltheta) {
      return false;
    }
    return true;
  }
  bool operator!=(const Udp2master_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Udp2master_

// alias to use template instance with default allocator
using Udp2master =
  humanoid_interfaces::msg::Udp2master_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__UDP2MASTER__STRUCT_HPP_
