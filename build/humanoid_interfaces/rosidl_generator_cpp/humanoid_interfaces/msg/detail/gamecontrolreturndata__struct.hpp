// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/Gamecontrolreturndata.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLRETURNDATA__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLRETURNDATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__Gamecontrolreturndata __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__Gamecontrolreturndata __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Gamecontrolreturndata_
{
  using Type = Gamecontrolreturndata_<ContainerAllocator>;

  explicit Gamecontrolreturndata_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message = 0ll;
    }
  }

  explicit Gamecontrolreturndata_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message = 0ll;
    }
  }

  // field types and members
  using _message_type =
    int64_t;
  _message_type message;

  // setters for named parameter idiom
  Type & set__message(
    const int64_t & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::Gamecontrolreturndata_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::Gamecontrolreturndata_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Gamecontrolreturndata_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Gamecontrolreturndata_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Gamecontrolreturndata_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Gamecontrolreturndata_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Gamecontrolreturndata_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Gamecontrolreturndata_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Gamecontrolreturndata_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Gamecontrolreturndata_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__Gamecontrolreturndata
    std::shared_ptr<humanoid_interfaces::msg::Gamecontrolreturndata_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__Gamecontrolreturndata
    std::shared_ptr<humanoid_interfaces::msg::Gamecontrolreturndata_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Gamecontrolreturndata_ & other) const
  {
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const Gamecontrolreturndata_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Gamecontrolreturndata_

// alias to use template instance with default allocator
using Gamecontrolreturndata =
  humanoid_interfaces::msg::Gamecontrolreturndata_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLRETURNDATA__STRUCT_HPP_
