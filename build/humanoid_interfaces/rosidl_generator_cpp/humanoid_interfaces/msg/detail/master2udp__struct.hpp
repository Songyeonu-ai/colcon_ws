// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from humanoid_interfaces:msg/Master2udp.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MASTER2UDP__STRUCT_HPP_
#define HUMANOID_INTERFACES__MSG__DETAIL__MASTER2UDP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__humanoid_interfaces__msg__Master2udp __attribute__((deprecated))
#else
# define DEPRECATED__humanoid_interfaces__msg__Master2udp __declspec(deprecated)
#endif

namespace humanoid_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Master2udp_
{
  using Type = Master2udp_<ContainerAllocator>;

  explicit Master2udp_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robotnum = 0ll;
      this->robotstate = 0ll;
      this->robotcoorx = 0ll;
      this->robotcoory = 0ll;
      this->robotimuyaw = 0ll;
      this->balldist = 0ll;
      this->ballcoorx = 0ll;
      this->ballcoory = 0ll;
      this->myteam = 0ll;
    }
  }

  explicit Master2udp_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robotnum = 0ll;
      this->robotstate = 0ll;
      this->robotcoorx = 0ll;
      this->robotcoory = 0ll;
      this->robotimuyaw = 0ll;
      this->balldist = 0ll;
      this->ballcoorx = 0ll;
      this->ballcoory = 0ll;
      this->myteam = 0ll;
    }
  }

  // field types and members
  using _robotnum_type =
    int64_t;
  _robotnum_type robotnum;
  using _robotstate_type =
    int64_t;
  _robotstate_type robotstate;
  using _robotcoorx_type =
    int64_t;
  _robotcoorx_type robotcoorx;
  using _robotcoory_type =
    int64_t;
  _robotcoory_type robotcoory;
  using _robotimuyaw_type =
    int64_t;
  _robotimuyaw_type robotimuyaw;
  using _balldist_type =
    int64_t;
  _balldist_type balldist;
  using _ballcoorx_type =
    int64_t;
  _ballcoorx_type ballcoorx;
  using _ballcoory_type =
    int64_t;
  _ballcoory_type ballcoory;
  using _myteam_type =
    int64_t;
  _myteam_type myteam;

  // setters for named parameter idiom
  Type & set__robotnum(
    const int64_t & _arg)
  {
    this->robotnum = _arg;
    return *this;
  }
  Type & set__robotstate(
    const int64_t & _arg)
  {
    this->robotstate = _arg;
    return *this;
  }
  Type & set__robotcoorx(
    const int64_t & _arg)
  {
    this->robotcoorx = _arg;
    return *this;
  }
  Type & set__robotcoory(
    const int64_t & _arg)
  {
    this->robotcoory = _arg;
    return *this;
  }
  Type & set__robotimuyaw(
    const int64_t & _arg)
  {
    this->robotimuyaw = _arg;
    return *this;
  }
  Type & set__balldist(
    const int64_t & _arg)
  {
    this->balldist = _arg;
    return *this;
  }
  Type & set__ballcoorx(
    const int64_t & _arg)
  {
    this->ballcoorx = _arg;
    return *this;
  }
  Type & set__ballcoory(
    const int64_t & _arg)
  {
    this->ballcoory = _arg;
    return *this;
  }
  Type & set__myteam(
    const int64_t & _arg)
  {
    this->myteam = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    humanoid_interfaces::msg::Master2udp_<ContainerAllocator> *;
  using ConstRawPtr =
    const humanoid_interfaces::msg::Master2udp_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Master2udp_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<humanoid_interfaces::msg::Master2udp_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Master2udp_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Master2udp_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      humanoid_interfaces::msg::Master2udp_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<humanoid_interfaces::msg::Master2udp_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Master2udp_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<humanoid_interfaces::msg::Master2udp_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__humanoid_interfaces__msg__Master2udp
    std::shared_ptr<humanoid_interfaces::msg::Master2udp_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__humanoid_interfaces__msg__Master2udp
    std::shared_ptr<humanoid_interfaces::msg::Master2udp_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Master2udp_ & other) const
  {
    if (this->robotnum != other.robotnum) {
      return false;
    }
    if (this->robotstate != other.robotstate) {
      return false;
    }
    if (this->robotcoorx != other.robotcoorx) {
      return false;
    }
    if (this->robotcoory != other.robotcoory) {
      return false;
    }
    if (this->robotimuyaw != other.robotimuyaw) {
      return false;
    }
    if (this->balldist != other.balldist) {
      return false;
    }
    if (this->ballcoorx != other.ballcoorx) {
      return false;
    }
    if (this->ballcoory != other.ballcoory) {
      return false;
    }
    if (this->myteam != other.myteam) {
      return false;
    }
    return true;
  }
  bool operator!=(const Master2udp_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Master2udp_

// alias to use template instance with default allocator
using Master2udp =
  humanoid_interfaces::msg::Master2udp_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace humanoid_interfaces

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MASTER2UDP__STRUCT_HPP_
