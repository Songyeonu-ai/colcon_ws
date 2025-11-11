// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/Master2localization25.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MASTER2LOCALIZATION25__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MASTER2LOCALIZATION25__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__Master2localization25 __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__Master2localization25 __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Master2localization25_
{
  using Type = Master2localization25_<ContainerAllocator>;

  explicit Master2localization25_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->goalx = 0ll;
      this->goaly = 0ll;
      this->targetx = 0ll;
      this->targety = 0ll;
      this->robotallowanceerror = 0ll;
      this->robotcaliradius = 0ll;
    }
  }

  explicit Master2localization25_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->goalx = 0ll;
      this->goaly = 0ll;
      this->targetx = 0ll;
      this->targety = 0ll;
      this->robotallowanceerror = 0ll;
      this->robotcaliradius = 0ll;
    }
  }

  // field types and members
  using _goalx_type =
    int64_t;
  _goalx_type goalx;
  using _goaly_type =
    int64_t;
  _goaly_type goaly;
  using _targetx_type =
    int64_t;
  _targetx_type targetx;
  using _targety_type =
    int64_t;
  _targety_type targety;
  using _robotallowanceerror_type =
    int64_t;
  _robotallowanceerror_type robotallowanceerror;
  using _robotcaliradius_type =
    int64_t;
  _robotcaliradius_type robotcaliradius;

  // setters for named parameter idiom
  Type & set__goalx(
    const int64_t & _arg)
  {
    this->goalx = _arg;
    return *this;
  }
  Type & set__goaly(
    const int64_t & _arg)
  {
    this->goaly = _arg;
    return *this;
  }
  Type & set__targetx(
    const int64_t & _arg)
  {
    this->targetx = _arg;
    return *this;
  }
  Type & set__targety(
    const int64_t & _arg)
  {
    this->targety = _arg;
    return *this;
  }
  Type & set__robotallowanceerror(
    const int64_t & _arg)
  {
    this->robotallowanceerror = _arg;
    return *this;
  }
  Type & set__robotcaliradius(
    const int64_t & _arg)
  {
    this->robotcaliradius = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::Master2localization25_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::Master2localization25_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Master2localization25_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Master2localization25_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Master2localization25_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Master2localization25_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Master2localization25_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Master2localization25_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Master2localization25_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Master2localization25_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__Master2localization25
    std::shared_ptr<humanoid_interfaces::msg::Master2localization25_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__Master2localization25
    std::shared_ptr<humanoid_interfaces::msg::Master2localization25_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Master2localization25_ & other) const
  {
    if (this->goalx != other.goalx) {
      return false;
    }
    if (this->goaly != other.goaly) {
      return false;
    }
    if (this->targetx != other.targetx) {
      return false;
    }
    if (this->targety != other.targety) {
      return false;
    }
    if (this->robotallowanceerror != other.robotallowanceerror) {
      return false;
    }
    if (this->robotcaliradius != other.robotcaliradius) {
      return false;
    }
    return true;
  }
  bool operator!=(const Master2localization25_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Master2localization25_

// alias to use template instance with default allocator
using Master2localization25 =
  humanoid_interfaces::msg::Master2localization25_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MASTER2LOCALIZATION25__STRUCT_HPP_
