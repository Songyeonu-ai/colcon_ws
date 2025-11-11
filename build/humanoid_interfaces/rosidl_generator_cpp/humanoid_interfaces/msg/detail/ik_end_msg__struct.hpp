// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/IkEndMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_END_MSG__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_END_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__IkEndMsg __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__IkEndMsg __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct IkEndMsg_
{
  using Type = IkEndMsg_<ContainerAllocator>;

  explicit IkEndMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ikend = 0l;
      this->onestep_end = 0l;
    }
  }

  explicit IkEndMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ikend = 0l;
      this->onestep_end = 0l;
    }
  }

  // field types and members
  using _ikend_type =
    int32_t;
  _ikend_type ikend;
  using _onestep_end_type =
    int32_t;
  _onestep_end_type onestep_end;

  // setters for named parameter idiom
  Type & set__ikend(
    const int32_t & _arg)
  {
    this->ikend = _arg;
    return *this;
  }
  Type & set__onestep_end(
    const int32_t & _arg)
  {
    this->onestep_end = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::IkEndMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::IkEndMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::IkEndMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::IkEndMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::IkEndMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::IkEndMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::IkEndMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::IkEndMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::IkEndMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::IkEndMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__IkEndMsg
    std::shared_ptr<humanoid_interfaces::msg::IkEndMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__IkEndMsg
    std::shared_ptr<humanoid_interfaces::msg::IkEndMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IkEndMsg_ & other) const
  {
    if (this->ikend != other.ikend) {
      return false;
    }
    if (this->onestep_end != other.onestep_end) {
      return false;
    }
    return true;
  }
  bool operator!=(const IkEndMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IkEndMsg_

// alias to use template instance with default allocator
using IkEndMsg =
  humanoid_interfaces::msg::IkEndMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_END_MSG__STRUCT_HPP_
