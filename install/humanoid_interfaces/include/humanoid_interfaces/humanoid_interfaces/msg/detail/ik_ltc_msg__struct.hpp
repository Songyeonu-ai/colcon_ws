// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/IkLTCMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_LTC_MSG__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_LTC_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__IkLTCMsg __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__IkLTCMsg __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct IkLTCMsg_
{
  using Type = IkLTCMsg_<ContainerAllocator>;

  explicit IkLTCMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->entire_time = 0.0;
      this->swing_gain_l = 0.0;
      this->swing_gain_r = 0.0;
      this->warning = 0ll;
      this->safe = 0ll;
      this->landing_time_l = 0.0;
      this->landing_time_r = 0.0;
      this->landing_error_l = 0.0;
      this->landing_error_r = 0.0;
    }
  }

  explicit IkLTCMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->entire_time = 0.0;
      this->swing_gain_l = 0.0;
      this->swing_gain_r = 0.0;
      this->warning = 0ll;
      this->safe = 0ll;
      this->landing_time_l = 0.0;
      this->landing_time_r = 0.0;
      this->landing_error_l = 0.0;
      this->landing_error_r = 0.0;
    }
  }

  // field types and members
  using _entire_time_type =
    double;
  _entire_time_type entire_time;
  using _swing_gain_l_type =
    double;
  _swing_gain_l_type swing_gain_l;
  using _swing_gain_r_type =
    double;
  _swing_gain_r_type swing_gain_r;
  using _warning_type =
    int64_t;
  _warning_type warning;
  using _safe_type =
    int64_t;
  _safe_type safe;
  using _landing_time_l_type =
    double;
  _landing_time_l_type landing_time_l;
  using _landing_time_r_type =
    double;
  _landing_time_r_type landing_time_r;
  using _landing_error_l_type =
    double;
  _landing_error_l_type landing_error_l;
  using _landing_error_r_type =
    double;
  _landing_error_r_type landing_error_r;

  // setters for named parameter idiom
  Type & set__entire_time(
    const double & _arg)
  {
    this->entire_time = _arg;
    return *this;
  }
  Type & set__swing_gain_l(
    const double & _arg)
  {
    this->swing_gain_l = _arg;
    return *this;
  }
  Type & set__swing_gain_r(
    const double & _arg)
  {
    this->swing_gain_r = _arg;
    return *this;
  }
  Type & set__warning(
    const int64_t & _arg)
  {
    this->warning = _arg;
    return *this;
  }
  Type & set__safe(
    const int64_t & _arg)
  {
    this->safe = _arg;
    return *this;
  }
  Type & set__landing_time_l(
    const double & _arg)
  {
    this->landing_time_l = _arg;
    return *this;
  }
  Type & set__landing_time_r(
    const double & _arg)
  {
    this->landing_time_r = _arg;
    return *this;
  }
  Type & set__landing_error_l(
    const double & _arg)
  {
    this->landing_error_l = _arg;
    return *this;
  }
  Type & set__landing_error_r(
    const double & _arg)
  {
    this->landing_error_r = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::IkLTCMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::IkLTCMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::IkLTCMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::IkLTCMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::IkLTCMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::IkLTCMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::IkLTCMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::IkLTCMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::IkLTCMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::IkLTCMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__IkLTCMsg
    std::shared_ptr<humanoid_interfaces::msg::IkLTCMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__IkLTCMsg
    std::shared_ptr<humanoid_interfaces::msg::IkLTCMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IkLTCMsg_ & other) const
  {
    if (this->entire_time != other.entire_time) {
      return false;
    }
    if (this->swing_gain_l != other.swing_gain_l) {
      return false;
    }
    if (this->swing_gain_r != other.swing_gain_r) {
      return false;
    }
    if (this->warning != other.warning) {
      return false;
    }
    if (this->safe != other.safe) {
      return false;
    }
    if (this->landing_time_l != other.landing_time_l) {
      return false;
    }
    if (this->landing_time_r != other.landing_time_r) {
      return false;
    }
    if (this->landing_error_l != other.landing_error_l) {
      return false;
    }
    if (this->landing_error_r != other.landing_error_r) {
      return false;
    }
    return true;
  }
  bool operator!=(const IkLTCMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IkLTCMsg_

// alias to use template instance with default allocator
using IkLTCMsg =
  humanoid_interfaces::msg::IkLTCMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_LTC_MSG__STRUCT_HPP_
