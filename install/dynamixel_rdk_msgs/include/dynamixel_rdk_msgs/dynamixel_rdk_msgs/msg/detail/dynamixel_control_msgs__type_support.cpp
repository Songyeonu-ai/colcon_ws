// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dynamixel_rdk_msgs:msg/DynamixelControlMsgs.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_control_msgs__struct.hpp"
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

void DynamixelControlMsgs_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dynamixel_rdk_msgs::msg::DynamixelControlMsgs(_init);
}

void DynamixelControlMsgs_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dynamixel_rdk_msgs::msg::DynamixelControlMsgs *>(message_memory);
  typed_message->~DynamixelControlMsgs();
}

size_t size_function__DynamixelControlMsgs__motor_control(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dynamixel_rdk_msgs::msg::DynamixelMsgs> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DynamixelControlMsgs__motor_control(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dynamixel_rdk_msgs::msg::DynamixelMsgs> *>(untyped_member);
  return &member[index];
}

void * get_function__DynamixelControlMsgs__motor_control(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dynamixel_rdk_msgs::msg::DynamixelMsgs> *>(untyped_member);
  return &member[index];
}

void fetch_function__DynamixelControlMsgs__motor_control(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const dynamixel_rdk_msgs::msg::DynamixelMsgs *>(
    get_const_function__DynamixelControlMsgs__motor_control(untyped_member, index));
  auto & value = *reinterpret_cast<dynamixel_rdk_msgs::msg::DynamixelMsgs *>(untyped_value);
  value = item;
}

void assign_function__DynamixelControlMsgs__motor_control(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<dynamixel_rdk_msgs::msg::DynamixelMsgs *>(
    get_function__DynamixelControlMsgs__motor_control(untyped_member, index));
  const auto & value = *reinterpret_cast<const dynamixel_rdk_msgs::msg::DynamixelMsgs *>(untyped_value);
  item = value;
}

void resize_function__DynamixelControlMsgs__motor_control(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dynamixel_rdk_msgs::msg::DynamixelMsgs> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DynamixelControlMsgs_message_member_array[1] = {
  {
    "motor_control",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dynamixel_rdk_msgs::msg::DynamixelMsgs>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs::msg::DynamixelControlMsgs, motor_control),  // bytes offset in struct
    nullptr,  // default value
    size_function__DynamixelControlMsgs__motor_control,  // size() function pointer
    get_const_function__DynamixelControlMsgs__motor_control,  // get_const(index) function pointer
    get_function__DynamixelControlMsgs__motor_control,  // get(index) function pointer
    fetch_function__DynamixelControlMsgs__motor_control,  // fetch(index, &value) function pointer
    assign_function__DynamixelControlMsgs__motor_control,  // assign(index, value) function pointer
    resize_function__DynamixelControlMsgs__motor_control  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DynamixelControlMsgs_message_members = {
  "dynamixel_rdk_msgs::msg",  // message namespace
  "DynamixelControlMsgs",  // message name
  1,  // number of fields
  sizeof(dynamixel_rdk_msgs::msg::DynamixelControlMsgs),
  DynamixelControlMsgs_message_member_array,  // message members
  DynamixelControlMsgs_init_function,  // function to initialize message memory (memory has to be allocated)
  DynamixelControlMsgs_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DynamixelControlMsgs_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DynamixelControlMsgs_message_members,
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
get_message_type_support_handle<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>()
{
  return &::dynamixel_rdk_msgs::msg::rosidl_typesupport_introspection_cpp::DynamixelControlMsgs_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dynamixel_rdk_msgs, msg, DynamixelControlMsgs)() {
  return &::dynamixel_rdk_msgs::msg::rosidl_typesupport_introspection_cpp::DynamixelControlMsgs_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
