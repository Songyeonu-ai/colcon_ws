// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/IkAngleSimMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_ANGLE_SIM_MSG__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_ANGLE_SIM_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__IkAngleSimMsg __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__IkAngleSimMsg __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct IkAngleSimMsg_
{
  using Type = IkAngleSimMsg_<ContainerAllocator>;

  explicit IkAngleSimMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ang_l_0 = 0.0;
      this->ang_l_1 = 0.0;
      this->ang_l_2 = 0.0;
      this->ang_l_3 = 0.0;
      this->ang_l_4 = 0.0;
      this->ang_l_5 = 0.0;
      this->ang_l_6 = 0.0;
      this->ang_l_7 = 0.0;
      this->ang_l_8 = 0.0;
      this->ang_r_0 = 0.0;
      this->ang_r_1 = 0.0;
      this->ang_r_2 = 0.0;
      this->ang_r_3 = 0.0;
      this->ang_r_4 = 0.0;
      this->ang_r_5 = 0.0;
      this->ang_r_6 = 0.0;
      this->ang_r_7 = 0.0;
      this->ang_r_8 = 0.0;
    }
  }

  explicit IkAngleSimMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ang_l_0 = 0.0;
      this->ang_l_1 = 0.0;
      this->ang_l_2 = 0.0;
      this->ang_l_3 = 0.0;
      this->ang_l_4 = 0.0;
      this->ang_l_5 = 0.0;
      this->ang_l_6 = 0.0;
      this->ang_l_7 = 0.0;
      this->ang_l_8 = 0.0;
      this->ang_r_0 = 0.0;
      this->ang_r_1 = 0.0;
      this->ang_r_2 = 0.0;
      this->ang_r_3 = 0.0;
      this->ang_r_4 = 0.0;
      this->ang_r_5 = 0.0;
      this->ang_r_6 = 0.0;
      this->ang_r_7 = 0.0;
      this->ang_r_8 = 0.0;
    }
  }

  // field types and members
  using _ang_l_0_type =
    double;
  _ang_l_0_type ang_l_0;
  using _ang_l_1_type =
    double;
  _ang_l_1_type ang_l_1;
  using _ang_l_2_type =
    double;
  _ang_l_2_type ang_l_2;
  using _ang_l_3_type =
    double;
  _ang_l_3_type ang_l_3;
  using _ang_l_4_type =
    double;
  _ang_l_4_type ang_l_4;
  using _ang_l_5_type =
    double;
  _ang_l_5_type ang_l_5;
  using _ang_l_6_type =
    double;
  _ang_l_6_type ang_l_6;
  using _ang_l_7_type =
    double;
  _ang_l_7_type ang_l_7;
  using _ang_l_8_type =
    double;
  _ang_l_8_type ang_l_8;
  using _ang_r_0_type =
    double;
  _ang_r_0_type ang_r_0;
  using _ang_r_1_type =
    double;
  _ang_r_1_type ang_r_1;
  using _ang_r_2_type =
    double;
  _ang_r_2_type ang_r_2;
  using _ang_r_3_type =
    double;
  _ang_r_3_type ang_r_3;
  using _ang_r_4_type =
    double;
  _ang_r_4_type ang_r_4;
  using _ang_r_5_type =
    double;
  _ang_r_5_type ang_r_5;
  using _ang_r_6_type =
    double;
  _ang_r_6_type ang_r_6;
  using _ang_r_7_type =
    double;
  _ang_r_7_type ang_r_7;
  using _ang_r_8_type =
    double;
  _ang_r_8_type ang_r_8;

  // setters for named parameter idiom
  Type & set__ang_l_0(
    const double & _arg)
  {
    this->ang_l_0 = _arg;
    return *this;
  }
  Type & set__ang_l_1(
    const double & _arg)
  {
    this->ang_l_1 = _arg;
    return *this;
  }
  Type & set__ang_l_2(
    const double & _arg)
  {
    this->ang_l_2 = _arg;
    return *this;
  }
  Type & set__ang_l_3(
    const double & _arg)
  {
    this->ang_l_3 = _arg;
    return *this;
  }
  Type & set__ang_l_4(
    const double & _arg)
  {
    this->ang_l_4 = _arg;
    return *this;
  }
  Type & set__ang_l_5(
    const double & _arg)
  {
    this->ang_l_5 = _arg;
    return *this;
  }
  Type & set__ang_l_6(
    const double & _arg)
  {
    this->ang_l_6 = _arg;
    return *this;
  }
  Type & set__ang_l_7(
    const double & _arg)
  {
    this->ang_l_7 = _arg;
    return *this;
  }
  Type & set__ang_l_8(
    const double & _arg)
  {
    this->ang_l_8 = _arg;
    return *this;
  }
  Type & set__ang_r_0(
    const double & _arg)
  {
    this->ang_r_0 = _arg;
    return *this;
  }
  Type & set__ang_r_1(
    const double & _arg)
  {
    this->ang_r_1 = _arg;
    return *this;
  }
  Type & set__ang_r_2(
    const double & _arg)
  {
    this->ang_r_2 = _arg;
    return *this;
  }
  Type & set__ang_r_3(
    const double & _arg)
  {
    this->ang_r_3 = _arg;
    return *this;
  }
  Type & set__ang_r_4(
    const double & _arg)
  {
    this->ang_r_4 = _arg;
    return *this;
  }
  Type & set__ang_r_5(
    const double & _arg)
  {
    this->ang_r_5 = _arg;
    return *this;
  }
  Type & set__ang_r_6(
    const double & _arg)
  {
    this->ang_r_6 = _arg;
    return *this;
  }
  Type & set__ang_r_7(
    const double & _arg)
  {
    this->ang_r_7 = _arg;
    return *this;
  }
  Type & set__ang_r_8(
    const double & _arg)
  {
    this->ang_r_8 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::IkAngleSimMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::IkAngleSimMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::IkAngleSimMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::IkAngleSimMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::IkAngleSimMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::IkAngleSimMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::IkAngleSimMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::IkAngleSimMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::IkAngleSimMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::IkAngleSimMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__IkAngleSimMsg
    std::shared_ptr<humanoid_interfaces::msg::IkAngleSimMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__IkAngleSimMsg
    std::shared_ptr<humanoid_interfaces::msg::IkAngleSimMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IkAngleSimMsg_ & other) const
  {
    if (this->ang_l_0 != other.ang_l_0) {
      return false;
    }
    if (this->ang_l_1 != other.ang_l_1) {
      return false;
    }
    if (this->ang_l_2 != other.ang_l_2) {
      return false;
    }
    if (this->ang_l_3 != other.ang_l_3) {
      return false;
    }
    if (this->ang_l_4 != other.ang_l_4) {
      return false;
    }
    if (this->ang_l_5 != other.ang_l_5) {
      return false;
    }
    if (this->ang_l_6 != other.ang_l_6) {
      return false;
    }
    if (this->ang_l_7 != other.ang_l_7) {
      return false;
    }
    if (this->ang_l_8 != other.ang_l_8) {
      return false;
    }
    if (this->ang_r_0 != other.ang_r_0) {
      return false;
    }
    if (this->ang_r_1 != other.ang_r_1) {
      return false;
    }
    if (this->ang_r_2 != other.ang_r_2) {
      return false;
    }
    if (this->ang_r_3 != other.ang_r_3) {
      return false;
    }
    if (this->ang_r_4 != other.ang_r_4) {
      return false;
    }
    if (this->ang_r_5 != other.ang_r_5) {
      return false;
    }
    if (this->ang_r_6 != other.ang_r_6) {
      return false;
    }
    if (this->ang_r_7 != other.ang_r_7) {
      return false;
    }
    if (this->ang_r_8 != other.ang_r_8) {
      return false;
    }
    return true;
  }
  bool operator!=(const IkAngleSimMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IkAngleSimMsg_

// alias to use template instance with default allocator
using IkAngleSimMsg =
  humanoid_interfaces::msg::IkAngleSimMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_ANGLE_SIM_MSG__STRUCT_HPP_
