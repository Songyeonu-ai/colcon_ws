// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/LCMsgs.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__LC_MSGS__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__LC_MSGS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__LCMsgs __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__LCMsgs __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LCMsgs_
{
  using Type = LCMsgs_<ContainerAllocator>;

  explicit LCMsgs_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit LCMsgs_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _r_lc_data_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _r_lc_data_type r_lc_data;
  using _l_lc_data_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _l_lc_data_type l_lc_data;

  // setters for named parameter idiom
  Type & set__r_lc_data(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->r_lc_data = _arg;
    return *this;
  }
  Type & set__l_lc_data(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->l_lc_data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::LCMsgs_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::LCMsgs_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::LCMsgs_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::LCMsgs_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::LCMsgs_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::LCMsgs_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::LCMsgs_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::LCMsgs_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::LCMsgs_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::LCMsgs_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__LCMsgs
    std::shared_ptr<humanoid_interfaces::msg::LCMsgs_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__LCMsgs
    std::shared_ptr<humanoid_interfaces::msg::LCMsgs_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LCMsgs_ & other) const
  {
    if (this->r_lc_data != other.r_lc_data) {
      return false;
    }
    if (this->l_lc_data != other.l_lc_data) {
      return false;
    }
    return true;
  }
  bool operator!=(const LCMsgs_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LCMsgs_

// alias to use template instance with default allocator
using LCMsgs =
  humanoid_interfaces::msg::LCMsgs_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__LC_MSGS__STRUCT_HPP_
