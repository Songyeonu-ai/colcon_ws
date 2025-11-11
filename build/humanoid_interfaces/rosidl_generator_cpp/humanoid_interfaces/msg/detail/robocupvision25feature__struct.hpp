// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/Robocupvision25feature.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25FEATURE__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25FEATURE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__Robocupvision25feature __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__Robocupvision25feature __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Robocupvision25feature_
{
  using Type = Robocupvision25feature_<ContainerAllocator>;

  explicit Robocupvision25feature_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Robocupvision25feature_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _confidence_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _confidence_type confidence;
  using _distance_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _distance_type distance;
  using _point_vec_x_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _point_vec_x_type point_vec_x;
  using _point_vec_y_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _point_vec_y_type point_vec_y;

  // setters for named parameter idiom
  Type & set__confidence(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__distance(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__point_vec_x(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->point_vec_x = _arg;
    return *this;
  }
  Type & set__point_vec_y(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->point_vec_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::Robocupvision25feature_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::Robocupvision25feature_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Robocupvision25feature_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Robocupvision25feature_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Robocupvision25feature_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Robocupvision25feature_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Robocupvision25feature_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Robocupvision25feature_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Robocupvision25feature_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Robocupvision25feature_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__Robocupvision25feature
    std::shared_ptr<humanoid_interfaces::msg::Robocupvision25feature_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__Robocupvision25feature
    std::shared_ptr<humanoid_interfaces::msg::Robocupvision25feature_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Robocupvision25feature_ & other) const
  {
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->point_vec_x != other.point_vec_x) {
      return false;
    }
    if (this->point_vec_y != other.point_vec_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const Robocupvision25feature_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Robocupvision25feature_

// alias to use template instance with default allocator
using Robocupvision25feature =
  humanoid_interfaces::msg::Robocupvision25feature_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25FEATURE__STRUCT_HPP_
