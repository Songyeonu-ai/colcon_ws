// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/Robocupvision25.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__Robocupvision25 __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__Robocupvision25 __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Robocupvision25_
{
  using Type = Robocupvision25_<ContainerAllocator>;

  explicit Robocupvision25_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ball_cam_x = 0ll;
      this->ball_cam_y = 0ll;
      this->ball_2d_x = 0.0;
      this->ball_2d_y = 0.0;
      this->ball_d = 0.0;
      this->pan = 0.0;
      this->tilt = 0.0;
      this->ball_speed_x = 0.0;
      this->ball_speed_y = 0.0;
      this->ball_speed_level = 0ll;
      this->scan_mode = 0ll;
    }
  }

  explicit Robocupvision25_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ball_cam_x = 0ll;
      this->ball_cam_y = 0ll;
      this->ball_2d_x = 0.0;
      this->ball_2d_y = 0.0;
      this->ball_d = 0.0;
      this->pan = 0.0;
      this->tilt = 0.0;
      this->ball_speed_x = 0.0;
      this->ball_speed_y = 0.0;
      this->ball_speed_level = 0ll;
      this->scan_mode = 0ll;
    }
  }

  // field types and members
  using _ball_cam_x_type =
    int64_t;
  _ball_cam_x_type ball_cam_x;
  using _ball_cam_y_type =
    int64_t;
  _ball_cam_y_type ball_cam_y;
  using _ball_2d_x_type =
    double;
  _ball_2d_x_type ball_2d_x;
  using _ball_2d_y_type =
    double;
  _ball_2d_y_type ball_2d_y;
  using _ball_d_type =
    double;
  _ball_d_type ball_d;
  using _pan_type =
    double;
  _pan_type pan;
  using _tilt_type =
    double;
  _tilt_type tilt;
  using _ball_speed_x_type =
    double;
  _ball_speed_x_type ball_speed_x;
  using _ball_speed_y_type =
    double;
  _ball_speed_y_type ball_speed_y;
  using _robot_vec_x_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _robot_vec_x_type robot_vec_x;
  using _robot_vec_y_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _robot_vec_y_type robot_vec_y;
  using _ball_speed_level_type =
    int64_t;
  _ball_speed_level_type ball_speed_level;
  using _scan_mode_type =
    int64_t;
  _scan_mode_type scan_mode;

  // setters for named parameter idiom
  Type & set__ball_cam_x(
    const int64_t & _arg)
  {
    this->ball_cam_x = _arg;
    return *this;
  }
  Type & set__ball_cam_y(
    const int64_t & _arg)
  {
    this->ball_cam_y = _arg;
    return *this;
  }
  Type & set__ball_2d_x(
    const double & _arg)
  {
    this->ball_2d_x = _arg;
    return *this;
  }
  Type & set__ball_2d_y(
    const double & _arg)
  {
    this->ball_2d_y = _arg;
    return *this;
  }
  Type & set__ball_d(
    const double & _arg)
  {
    this->ball_d = _arg;
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
  Type & set__robot_vec_x(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->robot_vec_x = _arg;
    return *this;
  }
  Type & set__robot_vec_y(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->robot_vec_y = _arg;
    return *this;
  }
  Type & set__ball_speed_level(
    const int64_t & _arg)
  {
    this->ball_speed_level = _arg;
    return *this;
  }
  Type & set__scan_mode(
    const int64_t & _arg)
  {
    this->scan_mode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::Robocupvision25_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::Robocupvision25_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Robocupvision25_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Robocupvision25_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Robocupvision25_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Robocupvision25_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Robocupvision25_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Robocupvision25_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Robocupvision25_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Robocupvision25_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__Robocupvision25
    std::shared_ptr<humanoid_interfaces::msg::Robocupvision25_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__Robocupvision25
    std::shared_ptr<humanoid_interfaces::msg::Robocupvision25_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Robocupvision25_ & other) const
  {
    if (this->ball_cam_x != other.ball_cam_x) {
      return false;
    }
    if (this->ball_cam_y != other.ball_cam_y) {
      return false;
    }
    if (this->ball_2d_x != other.ball_2d_x) {
      return false;
    }
    if (this->ball_2d_y != other.ball_2d_y) {
      return false;
    }
    if (this->ball_d != other.ball_d) {
      return false;
    }
    if (this->pan != other.pan) {
      return false;
    }
    if (this->tilt != other.tilt) {
      return false;
    }
    if (this->ball_speed_x != other.ball_speed_x) {
      return false;
    }
    if (this->ball_speed_y != other.ball_speed_y) {
      return false;
    }
    if (this->robot_vec_x != other.robot_vec_x) {
      return false;
    }
    if (this->robot_vec_y != other.robot_vec_y) {
      return false;
    }
    if (this->ball_speed_level != other.ball_speed_level) {
      return false;
    }
    if (this->scan_mode != other.scan_mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const Robocupvision25_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Robocupvision25_

// alias to use template instance with default allocator
using Robocupvision25 =
  humanoid_interfaces::msg::Robocupvision25_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25__STRUCT_HPP_
