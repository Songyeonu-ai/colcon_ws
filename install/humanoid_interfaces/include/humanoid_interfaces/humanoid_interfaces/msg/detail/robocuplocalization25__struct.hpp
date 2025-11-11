// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/Robocuplocalization25.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPLOCALIZATION25__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPLOCALIZATION25__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__Robocuplocalization25 __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__Robocuplocalization25 __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Robocuplocalization25_
{
  using Type = Robocuplocalization25_<ContainerAllocator>;

  explicit Robocuplocalization25_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ball_x = 0.0;
      this->ball_y = 0.0;
      this->ball_speed_x = 0.0;
      this->ball_speed_y = 0.0;
      this->robot_x = 0.0;
      this->robot_y = 0.0;
      this->obstacle_x = 0.0;
      this->obstacle_y = 0.0;
      this->obstacle0_x = 0.0;
      this->obstacle0_y = 0.0;
      this->obstacle1_x = 0.0;
      this->obstacle1_y = 0.0;
      this->obstacle2_x = 0.0;
      this->obstacle2_y = 0.0;
      this->obstacle3_x = 0.0;
      this->obstacle3_y = 0.0;
    }
  }

  explicit Robocuplocalization25_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ball_x = 0.0;
      this->ball_y = 0.0;
      this->ball_speed_x = 0.0;
      this->ball_speed_y = 0.0;
      this->robot_x = 0.0;
      this->robot_y = 0.0;
      this->obstacle_x = 0.0;
      this->obstacle_y = 0.0;
      this->obstacle0_x = 0.0;
      this->obstacle0_y = 0.0;
      this->obstacle1_x = 0.0;
      this->obstacle1_y = 0.0;
      this->obstacle2_x = 0.0;
      this->obstacle2_y = 0.0;
      this->obstacle3_x = 0.0;
      this->obstacle3_y = 0.0;
    }
  }

  // field types and members
  using _ball_x_type =
    double;
  _ball_x_type ball_x;
  using _ball_y_type =
    double;
  _ball_y_type ball_y;
  using _ball_speed_x_type =
    double;
  _ball_speed_x_type ball_speed_x;
  using _ball_speed_y_type =
    double;
  _ball_speed_y_type ball_speed_y;
  using _robot_x_type =
    double;
  _robot_x_type robot_x;
  using _robot_y_type =
    double;
  _robot_y_type robot_y;
  using _obstacle_x_type =
    double;
  _obstacle_x_type obstacle_x;
  using _obstacle_y_type =
    double;
  _obstacle_y_type obstacle_y;
  using _obstacle0_x_type =
    double;
  _obstacle0_x_type obstacle0_x;
  using _obstacle0_y_type =
    double;
  _obstacle0_y_type obstacle0_y;
  using _obstacle1_x_type =
    double;
  _obstacle1_x_type obstacle1_x;
  using _obstacle1_y_type =
    double;
  _obstacle1_y_type obstacle1_y;
  using _obstacle2_x_type =
    double;
  _obstacle2_x_type obstacle2_x;
  using _obstacle2_y_type =
    double;
  _obstacle2_y_type obstacle2_y;
  using _obstacle3_x_type =
    double;
  _obstacle3_x_type obstacle3_x;
  using _obstacle3_y_type =
    double;
  _obstacle3_y_type obstacle3_y;

  // setters for named parameter idiom
  Type & set__ball_x(
    const double & _arg)
  {
    this->ball_x = _arg;
    return *this;
  }
  Type & set__ball_y(
    const double & _arg)
  {
    this->ball_y = _arg;
    return *this;
  }
  Type & set__ball_speed_x(
    const double & _arg)
  {
    this->ball_speed_x = _arg;
    return *this;
  }
  Type & set__ball_speed_y(
    const double & _arg)
  {
    this->ball_speed_y = _arg;
    return *this;
  }
  Type & set__robot_x(
    const double & _arg)
  {
    this->robot_x = _arg;
    return *this;
  }
  Type & set__robot_y(
    const double & _arg)
  {
    this->robot_y = _arg;
    return *this;
  }
  Type & set__obstacle_x(
    const double & _arg)
  {
    this->obstacle_x = _arg;
    return *this;
  }
  Type & set__obstacle_y(
    const double & _arg)
  {
    this->obstacle_y = _arg;
    return *this;
  }
  Type & set__obstacle0_x(
    const double & _arg)
  {
    this->obstacle0_x = _arg;
    return *this;
  }
  Type & set__obstacle0_y(
    const double & _arg)
  {
    this->obstacle0_y = _arg;
    return *this;
  }
  Type & set__obstacle1_x(
    const double & _arg)
  {
    this->obstacle1_x = _arg;
    return *this;
  }
  Type & set__obstacle1_y(
    const double & _arg)
  {
    this->obstacle1_y = _arg;
    return *this;
  }
  Type & set__obstacle2_x(
    const double & _arg)
  {
    this->obstacle2_x = _arg;
    return *this;
  }
  Type & set__obstacle2_y(
    const double & _arg)
  {
    this->obstacle2_y = _arg;
    return *this;
  }
  Type & set__obstacle3_x(
    const double & _arg)
  {
    this->obstacle3_x = _arg;
    return *this;
  }
  Type & set__obstacle3_y(
    const double & _arg)
  {
    this->obstacle3_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::Robocuplocalization25_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::Robocuplocalization25_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Robocuplocalization25_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Robocuplocalization25_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Robocuplocalization25_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Robocuplocalization25_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Robocuplocalization25_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Robocuplocalization25_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Robocuplocalization25_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Robocuplocalization25_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__Robocuplocalization25
    std::shared_ptr<humanoid_interfaces::msg::Robocuplocalization25_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__Robocuplocalization25
    std::shared_ptr<humanoid_interfaces::msg::Robocuplocalization25_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Robocuplocalization25_ & other) const
  {
    if (this->ball_x != other.ball_x) {
      return false;
    }
    if (this->ball_y != other.ball_y) {
      return false;
    }
    if (this->ball_speed_x != other.ball_speed_x) {
      return false;
    }
    if (this->ball_speed_y != other.ball_speed_y) {
      return false;
    }
    if (this->robot_x != other.robot_x) {
      return false;
    }
    if (this->robot_y != other.robot_y) {
      return false;
    }
    if (this->obstacle_x != other.obstacle_x) {
      return false;
    }
    if (this->obstacle_y != other.obstacle_y) {
      return false;
    }
    if (this->obstacle0_x != other.obstacle0_x) {
      return false;
    }
    if (this->obstacle0_y != other.obstacle0_y) {
      return false;
    }
    if (this->obstacle1_x != other.obstacle1_x) {
      return false;
    }
    if (this->obstacle1_y != other.obstacle1_y) {
      return false;
    }
    if (this->obstacle2_x != other.obstacle2_x) {
      return false;
    }
    if (this->obstacle2_y != other.obstacle2_y) {
      return false;
    }
    if (this->obstacle3_x != other.obstacle3_x) {
      return false;
    }
    if (this->obstacle3_y != other.obstacle3_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const Robocuplocalization25_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Robocuplocalization25_

// alias to use template instance with default allocator
using Robocuplocalization25 =
  humanoid_interfaces::msg::Robocuplocalization25_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPLOCALIZATION25__STRUCT_HPP_
