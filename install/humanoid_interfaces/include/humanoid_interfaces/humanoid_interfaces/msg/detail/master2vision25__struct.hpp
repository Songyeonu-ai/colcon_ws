// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/Master2vision25.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MASTER2VISION25__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MASTER2VISION25__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__Master2vision25 __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__Master2vision25 __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Master2vision25_
{
  using Type = Master2vision25_<ContainerAllocator>;

  explicit Master2vision25_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->scanmode = 0ll;
      this->pan = 0.0;
      this->tilt = 0.0;
    }
  }

  explicit Master2vision25_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->scanmode = 0ll;
      this->pan = 0.0;
      this->tilt = 0.0;
    }
  }

  // field types and members
  using _scanmode_type =
    int64_t;
  _scanmode_type scanmode;
  using _pan_type =
    double;
  _pan_type pan;
  using _tilt_type =
    double;
  _tilt_type tilt;

  // setters for named parameter idiom
  Type & set__scanmode(
    const int64_t & _arg)
  {
    this->scanmode = _arg;
    return *this;
  }
  Type & set__pan(
    const double & _arg)
  {
    this->pan = _arg;
    return *this;
  }
  Type & set__tilt(
    const double & _arg)
  {
    this->tilt = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::Master2vision25_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::Master2vision25_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Master2vision25_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Master2vision25_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Master2vision25_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Master2vision25_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Master2vision25_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Master2vision25_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Master2vision25_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Master2vision25_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__Master2vision25
    std::shared_ptr<humanoid_interfaces::msg::Master2vision25_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__Master2vision25
    std::shared_ptr<humanoid_interfaces::msg::Master2vision25_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Master2vision25_ & other) const
  {
    if (this->scanmode != other.scanmode) {
      return false;
    }
    if (this->pan != other.pan) {
      return false;
    }
    if (this->tilt != other.tilt) {
      return false;
    }
    return true;
  }
  bool operator!=(const Master2vision25_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Master2vision25_

// alias to use template instance with default allocator
using Master2vision25 =
  humanoid_interfaces::msg::Master2vision25_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MASTER2VISION25__STRUCT_HPP_
