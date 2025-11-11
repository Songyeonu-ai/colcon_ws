// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dynamixel_rdk_msgs:msg/DynamixelBulkReadMsgs.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_bulk_read_msgs__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dynamixel_rdk_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void DynamixelBulkReadMsgs_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs(_init);
}

void DynamixelBulkReadMsgs_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs *>(message_memory);
  typed_message->~DynamixelBulkReadMsgs();
}

size_t size_function__DynamixelBulkReadMsgs__status_msgs(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DynamixelBulkReadMsgs__status_msgs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs> *>(untyped_member);
  return &member[index];
}

void * get_function__DynamixelBulkReadMsgs__status_msgs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs> *>(untyped_member);
  return &member[index];
}

void fetch_function__DynamixelBulkReadMsgs__status_msgs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const dynamixel_rdk_msgs::msg::DynamixelStatusMsgs *>(
    get_const_function__DynamixelBulkReadMsgs__status_msgs(untyped_member, index));
  auto & value = *reinterpret_cast<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs *>(untyped_value);
  value = item;
}

void assign_function__DynamixelBulkReadMsgs__status_msgs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs *>(
    get_function__DynamixelBulkReadMsgs__status_msgs(untyped_member, index));
  const auto & value = *reinterpret_cast<const dynamixel_rdk_msgs::msg::DynamixelStatusMsgs *>(untyped_value);
  item = value;
}

void resize_function__DynamixelBulkReadMsgs__status_msgs(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DynamixelBulkReadMsgs_message_member_array[1] = {
  {
    "status_msgs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dynamixel_rdk_msgs::msg::DynamixelStatusMsgs>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs, status_msgs),  // bytes offset in struct
    nullptr,  // default value
    size_function__DynamixelBulkReadMsgs__status_msgs,  // size() function pointer
    get_const_function__DynamixelBulkReadMsgs__status_msgs,  // get_const(index) function pointer
    get_function__DynamixelBulkReadMsgs__status_msgs,  // get(index) function pointer
    fetch_function__DynamixelBulkReadMsgs__status_msgs,  // fetch(index, &value) function pointer
    assign_function__DynamixelBulkReadMsgs__status_msgs,  // assign(index, value) function pointer
    resize_function__DynamixelBulkReadMsgs__status_msgs  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DynamixelBulkReadMsgs_message_members = {
  "dynamixel_rdk_msgs::msg",  // message namespace
  "DynamixelBulkReadMsgs",  // message name
  1,  // number of fields
  sizeof(dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs),
  DynamixelBulkReadMsgs_message_member_array,  // message members
  DynamixelBulkReadMsgs_init_function,  // function to initialize message memory (memory has to be allocated)
  DynamixelBulkReadMsgs_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DynamixelBulkReadMsgs_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DynamixelBulkReadMsgs_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace dynamixel_rdk_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs>()
{
  return &::dynamixel_rdk_msgs::msg::rosidl_typesupport_introspection_cpp::DynamixelBulkReadMsgs_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dynamixel_rdk_msgs, msg, DynamixelBulkReadMsgs)() {
  return &::dynamixel_rdk_msgs::msg::rosidl_typesupport_introspection_cpp::DynamixelBulkReadMsgs_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
