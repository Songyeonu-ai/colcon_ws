// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/Master2IkMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MASTER2_IK_MSG__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MASTER2_IK_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__Master2IkMsg __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__Master2IkMsg __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Master2IkMsg_
{
  using Type = Master2IkMsg_<ContainerAllocator>;

  explicit Master2IkMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_length = 0.0;
      this->y_length = 0.0;
      this->yaw = 0.0;
      this->flag = 0.0;
      this->one_x_length = 0l;
      this->one_y_length = 0l;
      this->one_yaw = 0l;
      this->one_step_flag = 0l;
    }
  }

  explicit Master2IkMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_length = 0.0;
      this->y_length = 0.0;
      this->yaw = 0.0;
      this->flag = 0.0;
      this->one_x_length = 0l;
      this->one_y_length = 0l;
      this->one_yaw = 0l;
      this->one_step_flag = 0l;
    }
  }

  // field types and members
  using _x_length_type =
    double;
  _x_length_type x_length;
  using _y_length_type =
    double;
  _y_length_type y_length;
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _flag_type =
    double;
  _flag_type flag;
  using _one_x_length_type =
    int32_t;
  _one_x_length_type one_x_length;
  using _one_y_length_type =
    int32_t;
  _one_y_length_type one_y_length;
  using _one_yaw_type =
    int32_t;
  _one_yaw_type one_yaw;
  using _one_step_flag_type =
    int32_t;
  _one_step_flag_type one_step_flag;

  // setters for named parameter idiom
  Type & set__x_length(
    const double & _arg)
  {
    this->x_length = _arg;
    return *this;
  }
  Type & set__y_length(
    const double & _arg)
  {
    this->y_length = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__flag(
    const double & _arg)
  {
    this->flag = _arg;
    return *this;
  }
  Type & set__one_x_length(
    const int32_t & _arg)
  {
    this->one_x_length = _arg;
    return *this;
  }
  Type & set__one_y_length(
    const int32_t & _arg)
  {
    this->one_y_length = _arg;
    return *this;
  }
  Type & set__one_yaw(
    const int32_t & _arg)
  {
    this->one_yaw = _arg;
    return *this;
  }
  Type & set__one_step_flag(
    const int32_t & _arg)
  {
    this->one_step_flag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::Master2IkMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::Master2IkMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Master2IkMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Master2IkMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Master2IkMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Master2IkMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Master2IkMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Master2IkMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Master2IkMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Master2IkMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__Master2IkMsg
    std::shared_ptr<humanoid_interfaces::msg::Master2IkMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__Master2IkMsg
    std::shared_ptr<humanoid_interfaces::msg::Master2IkMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Master2IkMsg_ & other) const
  {
    if (this->x_length != other.x_length) {
      return false;
    }
    if (this->y_length != other.y_length) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->flag != other.flag) {
      return false;
    }
    if (this->one_x_length != other.one_x_length) {
      return false;
    }
    if (this->one_y_length != other.one_y_length) {
      return false;
    }
    if (this->one_yaw != other.one_yaw) {
      return false;
    }
    if (this->one_step_flag != other.one_step_flag) {
      return false;
    }
    return true;
  }
  bool operator!=(const Master2IkMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Master2IkMsg_

// alias to use template instance with default allocator
using Master2IkMsg =
  humanoid_interfaces::msg::Master2IkMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MASTER2_IK_MSG__STRUCT_HPP_
