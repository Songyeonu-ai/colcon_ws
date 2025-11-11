// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/IkComMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_COM_MSG__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_COM_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__IkComMsg __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__IkComMsg __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct IkComMsg_
{
  using Type = IkComMsg_<ContainerAllocator>;

  explicit IkComMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_com = 0.0;
      this->y_com = 0.0;
    }
  }

  explicit IkComMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_com = 0.0;
      this->y_com = 0.0;
    }
  }

  // field types and members
  using _x_com_type =
    double;
  _x_com_type x_com;
  using _y_com_type =
    double;
  _y_com_type y_com;

  // setters for named parameter idiom
  Type & set__x_com(
    const double & _arg)
  {
    this->x_com = _arg;
    return *this;
  }
  Type & set__y_com(
    const double & _arg)
  {
    this->y_com = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::IkComMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::IkComMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::IkComMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::IkComMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::IkComMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::IkComMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::IkComMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::IkComMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::IkComMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::IkComMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__IkComMsg
    std::shared_ptr<humanoid_interfaces::msg::IkComMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__IkComMsg
    std::shared_ptr<humanoid_interfaces::msg::IkComMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IkComMsg_ & other) const
  {
    if (this->x_com != other.x_com) {
      return false;
    }
    if (this->y_com != other.y_com) {
      return false;
    }
    return true;
  }
  bool operator!=(const IkComMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IkComMsg_

// alias to use template instance with default allocator
using IkComMsg =
  humanoid_interfaces::msg::IkComMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_COM_MSG__STRUCT_HPP_
