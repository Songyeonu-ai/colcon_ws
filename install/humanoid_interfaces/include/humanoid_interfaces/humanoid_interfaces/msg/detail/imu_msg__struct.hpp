// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/ImuMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IMU_MSG__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__IMU_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__ImuMsg __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__ImuMsg __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ImuMsg_
{
  using Type = ImuMsg_<ContainerAllocator>;

  explicit ImuMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pitch = 0.0f;
      this->roll = 0.0f;
      this->yaw = 0.0f;
      this->q_x = 0.0f;
      this->q_y = 0.0f;
      this->q_z = 0.0f;
      this->q_w = 0.0f;
      this->x_gyro = 0.0f;
      this->y_gyro = 0.0f;
      this->z_gyro = 0.0f;
      this->x_acc = 0.0f;
      this->y_acc = 0.0f;
      this->z_acc = 0.0f;
      this->x_vel = 0.0f;
      this->y_vel = 0.0f;
      this->z_vel = 0.0f;
      this->x_pos = 0.0f;
      this->y_pos = 0.0f;
      this->z_pos = 0.0f;
    }
  }

  explicit ImuMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pitch = 0.0f;
      this->roll = 0.0f;
      this->yaw = 0.0f;
      this->q_x = 0.0f;
      this->q_y = 0.0f;
      this->q_z = 0.0f;
      this->q_w = 0.0f;
      this->x_gyro = 0.0f;
      this->y_gyro = 0.0f;
      this->z_gyro = 0.0f;
      this->x_acc = 0.0f;
      this->y_acc = 0.0f;
      this->z_acc = 0.0f;
      this->x_vel = 0.0f;
      this->y_vel = 0.0f;
      this->z_vel = 0.0f;
      this->x_pos = 0.0f;
      this->y_pos = 0.0f;
      this->z_pos = 0.0f;
    }
  }

  // field types and members
  using _pitch_type =
    float;
  _pitch_type pitch;
  using _roll_type =
    float;
  _roll_type roll;
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _q_x_type =
    float;
  _q_x_type q_x;
  using _q_y_type =
    float;
  _q_y_type q_y;
  using _q_z_type =
    float;
  _q_z_type q_z;
  using _q_w_type =
    float;
  _q_w_type q_w;
  using _x_gyro_type =
    float;
  _x_gyro_type x_gyro;
  using _y_gyro_type =
    float;
  _y_gyro_type y_gyro;
  using _z_gyro_type =
    float;
  _z_gyro_type z_gyro;
  using _x_acc_type =
    float;
  _x_acc_type x_acc;
  using _y_acc_type =
    float;
  _y_acc_type y_acc;
  using _z_acc_type =
    float;
  _z_acc_type z_acc;
  using _x_vel_type =
    float;
  _x_vel_type x_vel;
  using _y_vel_type =
    float;
  _y_vel_type y_vel;
  using _z_vel_type =
    float;
  _z_vel_type z_vel;
  using _x_pos_type =
    float;
  _x_pos_type x_pos;
  using _y_pos_type =
    float;
  _y_pos_type y_pos;
  using _z_pos_type =
    float;
  _z_pos_type z_pos;

  // setters for named parameter idiom
  Type & set__pitch(
    const float & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__roll(
    const float & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__q_x(
    const float & _arg)
  {
    this->q_x = _arg;
    return *this;
  }
  Type & set__q_y(
    const float & _arg)
  {
    this->q_y = _arg;
    return *this;
  }
  Type & set__q_z(
    const float & _arg)
  {
    this->q_z = _arg;
    return *this;
  }
  Type & set__q_w(
    const float & _arg)
  {
    this->q_w = _arg;
    return *this;
  }
  Type & set__x_gyro(
    const float & _arg)
  {
    this->x_gyro = _arg;
    return *this;
  }
  Type & set__y_gyro(
    const float & _arg)
  {
    this->y_gyro = _arg;
    return *this;
  }
  Type & set__z_gyro(
    const float & _arg)
  {
    this->z_gyro = _arg;
    return *this;
  }
  Type & set__x_acc(
    const float & _arg)
  {
    this->x_acc = _arg;
    return *this;
  }
  Type & set__y_acc(
    const float & _arg)
  {
    this->y_acc = _arg;
    return *this;
  }
  Type & set__z_acc(
    const float & _arg)
  {
    this->z_acc = _arg;
    return *this;
  }
  Type & set__x_vel(
    const float & _arg)
  {
    this->x_vel = _arg;
    return *this;
  }
  Type & set__y_vel(
    const float & _arg)
  {
    this->y_vel = _arg;
    return *this;
  }
  Type & set__z_vel(
    const float & _arg)
  {
    this->z_vel = _arg;
    return *this;
  }
  Type & set__x_pos(
    const float & _arg)
  {
    this->x_pos = _arg;
    return *this;
  }
  Type & set__y_pos(
    const float & _arg)
  {
    this->y_pos = _arg;
    return *this;
  }
  Type & set__z_pos(
    const float & _arg)
  {
    this->z_pos = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::ImuMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::ImuMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::ImuMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::ImuMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::ImuMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::ImuMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::ImuMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::ImuMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::ImuMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::ImuMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__ImuMsg
    std::shared_ptr<humanoid_interfaces::msg::ImuMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__ImuMsg
    std::shared_ptr<humanoid_interfaces::msg::ImuMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ImuMsg_ & other) const
  {
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->q_x != other.q_x) {
      return false;
    }
    if (this->q_y != other.q_y) {
      return false;
    }
    if (this->q_z != other.q_z) {
      return false;
    }
    if (this->q_w != other.q_w) {
      return false;
    }
    if (this->x_gyro != other.x_gyro) {
      return false;
    }
    if (this->y_gyro != other.y_gyro) {
      return false;
    }
    if (this->z_gyro != other.z_gyro) {
      return false;
    }
    if (this->x_acc != other.x_acc) {
      return false;
    }
    if (this->y_acc != other.y_acc) {
      return false;
    }
    if (this->z_acc != other.z_acc) {
      return false;
    }
    if (this->x_vel != other.x_vel) {
      return false;
    }
    if (this->y_vel != other.y_vel) {
      return false;
    }
    if (this->z_vel != other.z_vel) {
      return false;
    }
    if (this->x_pos != other.x_pos) {
      return false;
    }
    if (this->y_pos != other.y_pos) {
      return false;
    }
    if (this->z_pos != other.z_pos) {
      return false;
    }
    return true;
  }
  bool operator!=(const ImuMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ImuMsg_

// alias to use template instance with default allocator
using ImuMsg =
  humanoid_interfaces::msg::ImuMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IMU_MSG__STRUCT_HPP_
