// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dynamixel_rdk_msgs:msg/DynamixelControlMsgs.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_control_msgs__rosidl_typesupport_introspection_c.h"
#include "dynamixel_rdk_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_control_msgs__functions.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_control_msgs__struct.h"


// Include directives for member types
// Member `motor_control`
#include "dynamixel_rdk_msgs/msg/dynamixel_msgs.h"
// Member `motor_control`
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_msgs__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__DynamixelControlMsgs_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dynamixel_rdk_msgs__msg__DynamixelControlMsgs__init(message_memory);
}

void dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__DynamixelControlMsgs_fini_function(void * message_memory)
{
  dynamixel_rdk_msgs__msg__DynamixelControlMsgs__fini(message_memory);
}

size_t dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__size_function__DynamixelControlMsgs__motor_control(
  const void * untyped_member)
{
  const dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * member =
    (const dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence *)(untyped_member);
  return member->size;
}

const void * dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__get_const_function__DynamixelControlMsgs__motor_control(
  const void * untyped_member, size_t index)
{
  const dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * member =
    (const dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__get_function__DynamixelControlMsgs__motor_control(
  void * untyped_member, size_t index)
{
  dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * member =
    (dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence *)(untyped_member);
  return &member->data[index];
}

void dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__fetch_function__DynamixelControlMsgs__motor_control(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const dynamixel_rdk_msgs__msg__DynamixelMsgs * item =
    ((const dynamixel_rdk_msgs__msg__DynamixelMsgs *)
    dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__get_const_function__DynamixelControlMsgs__motor_control(untyped_member, index));
  dynamixel_rdk_msgs__msg__DynamixelMsgs * value =
    (dynamixel_rdk_msgs__msg__DynamixelMsgs *)(untyped_value);
  *value = *item;
}

void dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__assign_function__DynamixelControlMsgs__motor_control(
  void * untyped_member, size_t index, const void * untyped_value)
{
  dynamixel_rdk_msgs__msg__DynamixelMsgs * item =
    ((dynamixel_rdk_msgs__msg__DynamixelMsgs *)
    dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__get_function__DynamixelControlMsgs__motor_control(untyped_member, index));
  const dynamixel_rdk_msgs__msg__DynamixelMsgs * value =
    (const dynamixel_rdk_msgs__msg__DynamixelMsgs *)(untyped_value);
  *item = *value;
}

bool dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__resize_function__DynamixelControlMsgs__motor_control(
  void * untyped_member, size_t size)
{
  dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * member =
    (dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence *)(untyped_member);
  dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__fini(member);
  return dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__DynamixelControlMsgs_message_member_array[1] = {
  {
    "motor_control",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs__msg__DynamixelControlMsgs, motor_control),  // bytes offset in struct
    NULL,  // default value
    dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__size_function__DynamixelControlMsgs__motor_control,  // size() function pointer
    dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__get_const_function__DynamixelControlMsgs__motor_control,  // get_const(index) function pointer
    dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__get_function__DynamixelControlMsgs__motor_control,  // get(index) function pointer
    dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__fetch_function__DynamixelControlMsgs__motor_control,  // fetch(index, &value) function pointer
    dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__assign_function__DynamixelControlMsgs__motor_control,  // assign(index, value) function pointer
    dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__resize_function__DynamixelControlMsgs__motor_control  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__DynamixelControlMsgs_message_members = {
  "dynamixel_rdk_msgs__msg",  // message namespace
  "DynamixelControlMsgs",  // message name
  1,  // number of fields
  sizeof(dynamixel_rdk_msgs__msg__DynamixelControlMsgs),
  dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__DynamixelControlMsgs_message_member_array,  // message members
  dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__DynamixelControlMsgs_init_function,  // function to initialize message memory (memory has to be allocated)
  dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__DynamixelControlMsgs_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__DynamixelControlMsgs_message_type_support_handle = {
  0,
  &dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__DynamixelControlMsgs_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dynamixel_rdk_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_rdk_msgs, msg, DynamixelControlMsgs)() {
  dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__DynamixelControlMsgs_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_rdk_msgs, msg, DynamixelMsgs)();
  if (!dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__DynamixelControlMsgs_message_type_support_handle.typesupport_identifier) {
    dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__DynamixelControlMsgs_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dynamixel_rdk_msgs__msg__DynamixelControlMsgs__rosidl_typesupport_introspection_c__DynamixelControlMsgs_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
