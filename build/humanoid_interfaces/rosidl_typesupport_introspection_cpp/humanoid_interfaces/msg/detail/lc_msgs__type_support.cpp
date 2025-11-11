// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from humanoid_interfaces:msg/LCMsgs.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "humanoid_interfaces/msg/detail/lc_msgs__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace humanoid_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void LCMsgs_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) humanoid_interfaces::msg::LCMsgs(_init);
}

void LCMsgs_fini_function(void * message_memory)
{
  auto typed_message = static_cast<humanoid_interfaces::msg::LCMsgs *>(message_memory);
  typed_message->~LCMsgs();
}

size_t size_function__LCMsgs__r_lc_data(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__LCMsgs__r_lc_data(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__LCMsgs__r_lc_data(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__LCMsgs__r_lc_data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__LCMsgs__r_lc_data(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__LCMsgs__r_lc_data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__LCMsgs__r_lc_data(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__LCMsgs__r_lc_data(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__LCMsgs__l_lc_data(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__LCMsgs__l_lc_data(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__LCMsgs__l_lc_data(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__LCMsgs__l_lc_data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__LCMsgs__l_lc_data(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__LCMsgs__l_lc_data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__LCMsgs__l_lc_data(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__LCMsgs__l_lc_data(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember LCMsgs_message_member_array[2] = {
  {
    "r_lc_data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces::msg::LCMsgs, r_lc_data),  // bytes offset in struct
    nullptr,  // default value
    size_function__LCMsgs__r_lc_data,  // size() function pointer
    get_const_function__LCMsgs__r_lc_data,  // get_const(index) function pointer
    get_function__LCMsgs__r_lc_data,  // get(index) function pointer
    fetch_function__LCMsgs__r_lc_data,  // fetch(index, &value) function pointer
    assign_function__LCMsgs__r_lc_data,  // assign(index, value) function pointer
    resize_function__LCMsgs__r_lc_data  // resize(index) function pointer
  },
  {
    "l_lc_data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces::msg::LCMsgs, l_lc_data),  // bytes offset in struct
    nullptr,  // default value
    size_function__LCMsgs__l_lc_data,  // size() function pointer
    get_const_function__LCMsgs__l_lc_data,  // get_const(index) function pointer
    get_function__LCMsgs__l_lc_data,  // get(index) function pointer
    fetch_function__LCMsgs__l_lc_data,  // fetch(index, &value) function pointer
    assign_function__LCMsgs__l_lc_data,  // assign(index, value) function pointer
    resize_function__LCMsgs__l_lc_data  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers LCMsgs_message_members = {
  "humanoid_interfaces::msg",  // message namespace
  "LCMsgs",  // message name
  2,  // number of fields
  sizeof(humanoid_interfaces::msg::LCMsgs),
  LCMsgs_message_member_array,  // message members
  LCMsgs_init_function,  // function to initialize message memory (memory has to be allocated)
  LCMsgs_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t LCMsgs_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &LCMsgs_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace humanoid_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<humanoid_interfaces::msg::LCMsgs>()
{
  return &::humanoid_interfaces::msg::rosidl_typesupport_introspection_cpp::LCMsgs_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, humanoid_interfaces, msg, LCMsgs)() {
  return &::humanoid_interfaces::msg::rosidl_typesupport_introspection_cpp::LCMsgs_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
