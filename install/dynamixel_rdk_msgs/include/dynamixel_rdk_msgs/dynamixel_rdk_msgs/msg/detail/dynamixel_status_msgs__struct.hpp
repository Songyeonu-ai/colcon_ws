// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dynamixel_rdk_msgs:msg/DynamixelStatusMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_STATUS_MSGS__STRUCT_HPP_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_STATUS_MSGS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dynamixel_rdk_msgs__msg__DynamixelStatusMsgs __attribute__((deprecated))
#else
# define DEPRECATED__dynamixel_rdk_msgs__msg__DynamixelStatusMsgs __declspec(deprecated)
#endif

namespace dynamixel_rdk_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DynamixelStatusMsgs_
{
  using Type = DynamixelStatusMsgs_<ContainerAllocator>;

  explicit DynamixelStatusMsgs_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->torque_enabled = false;
      this->error_status = 0;
      this->present_position = 0.0;
      this->present_velocity = 0.0;
      this->present_acceleration = 0.0;
      this->present_current = 0.0;
      this->present_voltage = 0.0;
      this->present_temperature = 0.0;
      std::fill<typename std::array<double, 2>::iterator, double>(this->min_max_position.begin(), this->min_max_position.end(), 0.0);
    }
  }

  explicit DynamixelStatusMsgs_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    min_max_position(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->torque_enabled = false;
      this->error_status = 0;
      this->present_position = 0.0;
      this->present_velocity = 0.0;
      this->present_acceleration = 0.0;
      this->present_current = 0.0;
      this->present_voltage = 0.0;
      this->present_temperature = 0.0;
      std::fill<typename std::array<double, 2>::iterator, double>(this->min_max_position.begin(), this->min_max_position.end(), 0.0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _id_type =
    int32_t;
  _id_type id;
  using _torque_enabled_type =
    bool;
  _torque_enabled_type torque_enabled;
  using _error_status_type =
    uint8_t;
  _error_status_type error_status;
  using _present_position_type =
    double;
  _present_position_type present_position;
  using _present_velocity_type =
    double;
  _present_velocity_type present_velocity;
  using _present_acceleration_type =
    double;
  _present_acceleration_type present_acceleration;
  using _present_current_type =
    double;
  _present_current_type present_current;
  using _present_voltage_type =
    double;
  _present_voltage_type present_voltage;
  using _present_temperature_type =
    double;
  _present_temperature_type present_temperature;
  using _min_max_position_type =
    std::array<double, 2>;
  _min_max_position_type min_max_position;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__torque_enabled(
    const bool & _arg)
  {
    this->torque_enabled = _arg;
    return *this;
  }
  Type & set__error_status(
    const uint8_t & _arg)
  {
    this->error_status = _arg;
    return *this;
  }
  Type & set__present_position(
    const double & _arg)
  {
    this->present_position = _arg;
    return *this;
  }
  Type & set__present_velocity(
    const double & _arg)
  {
    this->present_velocity = _arg;
    return *this;
  }
  Type & set__present_acceleration(
    const double & _arg)
  {
    this->present_acceleration = _arg;
    return *this;
  }
  Type & set__present_current(
    const double & _arg)
  {
    this->present_current = _arg;
    return *this;
  }
  Type & set__present_voltage(
    const double & _arg)
  {
    this->present_voltage = _arg;
    return *this;
  }
  Type & set__present_temperature(
    const double & _arg)
  {
    this->present_temperature = _arg;
    return *this;
  }
  Type & set__min_max_position(
    const std::array<double, 2> & _arg)
  {
    this->min_max_position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dynamixel_rdk_msgs::msg::DynamixelStatusMsgs_<ContainerAllocator> *;
  using ConstRawPtr =
    const dynamixel_rdk_msgs::msg::DynamixelStatusMsgs_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dynamixel_rdk_msgs::msg::DynamixelStatusMsgs_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dynamixel_rdk_msgs::msg::DynamixelStatusMsgs_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dynamixel_rdk_msgs__msg__DynamixelStatusMsgs
    std::shared_ptr<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dynamixel_rdk_msgs__msg__DynamixelStatusMsgs
    std::shared_ptr<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DynamixelStatusMsgs_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->torque_enabled != other.torque_enabled) {
      return false;
    }
    if (this->error_status != other.error_status) {
      return false;
    }
    if (this->present_position != other.present_position) {
      return false;
    }
    if (this->present_velocity != other.present_velocity) {
      return false;
    }
    if (this->present_acceleration != other.present_acceleration) {
      return false;
    }
    if (this->present_current != other.present_current) {
      return false;
    }
    if (this->present_voltage != other.present_voltage) {
      return false;
    }
    if (this->present_temperature != other.present_temperature) {
      return false;
    }
    if (this->min_max_position != other.min_max_position) {
      return false;
    }
    return true;
  }
  bool operator!=(const DynamixelStatusMsgs_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DynamixelStatusMsgs_

// alias to use template instance with default allocator
using DynamixelStatusMsgs =
  dynamixel_rdk_msgs::msg::DynamixelStatusMsgs_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dynamixel_rdk_msgs

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_STATUS_MSGS__STRUCT_HPP_
