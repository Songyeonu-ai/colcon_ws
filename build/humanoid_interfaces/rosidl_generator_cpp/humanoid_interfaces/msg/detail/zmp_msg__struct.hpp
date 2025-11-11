// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/ZmpMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ZMP_MSG__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__ZMP_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__ZmpMsg __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__ZmpMsg __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ZmpMsg_
{
  using Type = ZmpMsg_<ContainerAllocator>;

  explicit ZmpMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_x_zmp = 0.0;
      this->left_y_zmp = 0.0;
      this->right_x_zmp = 0.0;
      this->right_y_zmp = 0.0;
      this->total_x_zmp = 0.0;
      this->total_y_zmp = 0.0;
      this->left_foot = false;
      this->right_foot = false;
      this->both_feet = false;
    }
  }

  explicit ZmpMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_x_zmp = 0.0;
      this->left_y_zmp = 0.0;
      this->right_x_zmp = 0.0;
      this->right_y_zmp = 0.0;
      this->total_x_zmp = 0.0;
      this->total_y_zmp = 0.0;
      this->left_foot = false;
      this->right_foot = false;
      this->both_feet = false;
    }
  }

  // field types and members
  using _left_x_zmp_type =
    double;
  _left_x_zmp_type left_x_zmp;
  using _left_y_zmp_type =
    double;
  _left_y_zmp_type left_y_zmp;
  using _right_x_zmp_type =
    double;
  _right_x_zmp_type right_x_zmp;
  using _right_y_zmp_type =
    double;
  _right_y_zmp_type right_y_zmp;
  using _total_x_zmp_type =
    double;
  _total_x_zmp_type total_x_zmp;
  using _total_y_zmp_type =
    double;
  _total_y_zmp_type total_y_zmp;
  using _left_foot_type =
    bool;
  _left_foot_type left_foot;
  using _right_foot_type =
    bool;
  _right_foot_type right_foot;
  using _both_feet_type =
    bool;
  _both_feet_type both_feet;

  // setters for named parameter idiom
  Type & set__left_x_zmp(
    const double & _arg)
  {
    this->left_x_zmp = _arg;
    return *this;
  }
  Type & set__left_y_zmp(
    const double & _arg)
  {
    this->left_y_zmp = _arg;
    return *this;
  }
  Type & set__right_x_zmp(
    const double & _arg)
  {
    this->right_x_zmp = _arg;
    return *this;
  }
  Type & set__right_y_zmp(
    const double & _arg)
  {
    this->right_y_zmp = _arg;
    return *this;
  }
  Type & set__total_x_zmp(
    const double & _arg)
  {
    this->total_x_zmp = _arg;
    return *this;
  }
  Type & set__total_y_zmp(
    const double & _arg)
  {
    this->total_y_zmp = _arg;
    return *this;
  }
  Type & set__left_foot(
    const bool & _arg)
  {
    this->left_foot = _arg;
    return *this;
  }
  Type & set__right_foot(
    const bool & _arg)
  {
    this->right_foot = _arg;
    return *this;
  }
  Type & set__both_feet(
    const bool & _arg)
  {
    this->both_feet = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::ZmpMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::ZmpMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::ZmpMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::ZmpMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::ZmpMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::ZmpMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::ZmpMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::ZmpMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::ZmpMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::ZmpMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__ZmpMsg
    std::shared_ptr<humanoid_interfaces::msg::ZmpMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__ZmpMsg
    std::shared_ptr<humanoid_interfaces::msg::ZmpMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ZmpMsg_ & other) const
  {
    if (this->left_x_zmp != other.left_x_zmp) {
      return false;
    }
    if (this->left_y_zmp != other.left_y_zmp) {
      return false;
    }
    if (this->right_x_zmp != other.right_x_zmp) {
      return false;
    }
    if (this->right_y_zmp != other.right_y_zmp) {
      return false;
    }
    if (this->total_x_zmp != other.total_x_zmp) {
      return false;
    }
    if (this->total_y_zmp != other.total_y_zmp) {
      return false;
    }
    if (this->left_foot != other.left_foot) {
      return false;
    }
    if (this->right_foot != other.right_foot) {
      return false;
    }
    if (this->both_feet != other.both_feet) {
      return false;
    }
    return true;
  }
  bool operator!=(const ZmpMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ZmpMsg_

// alias to use template instance with default allocator
using ZmpMsg =
  humanoid_interfaces::msg::ZmpMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ZMP_MSG__STRUCT_HPP_
