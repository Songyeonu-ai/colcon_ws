// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from humanoid_interfaces:msg/MotorMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "humanoid_interfaces/msg/detail/motor_msg__rosidl_typesupport_introspection_c.h"
#include "humanoid_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "humanoid_interfaces/msg/detail/motor_msg__functions.h"
#include "humanoid_interfaces/msg/detail/motor_msg__struct.h"


// Include directives for member types
// Member `id`
// Member `speed`
// Member `position`
// Member `type`
// Member `acceleration`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__MotorMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  humanoid_interfaces__msg__MotorMsg__init(message_memory);
}

void humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__MotorMsg_fini_function(void * message_memory)
{
  humanoid_interfaces__msg__MotorMsg__fini(message_memory);
}

size_t humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__size_function__MotorMsg__id(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_const_function__MotorMsg__id(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_function__MotorMsg__id(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__fetch_function__MotorMsg__id(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_const_function__MotorMsg__id(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__assign_function__MotorMsg__id(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_function__MotorMsg__id(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__resize_function__MotorMsg__id(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__size_function__MotorMsg__speed(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_const_function__MotorMsg__speed(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_function__MotorMsg__speed(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__fetch_function__MotorMsg__speed(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_const_function__MotorMsg__speed(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__assign_function__MotorMsg__speed(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_function__MotorMsg__speed(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__resize_function__MotorMsg__speed(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__size_function__MotorMsg__position(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_const_function__MotorMsg__position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_function__MotorMsg__position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__fetch_function__MotorMsg__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_const_function__MotorMsg__position(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__assign_function__MotorMsg__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_function__MotorMsg__position(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__resize_function__MotorMsg__position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__size_function__MotorMsg__type(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_const_function__MotorMsg__type(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_function__MotorMsg__type(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__fetch_function__MotorMsg__type(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_const_function__MotorMsg__type(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__assign_function__MotorMsg__type(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_function__MotorMsg__type(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__resize_function__MotorMsg__type(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__size_function__MotorMsg__acceleration(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_const_function__MotorMsg__acceleration(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_function__MotorMsg__acceleration(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__fetch_function__MotorMsg__acceleration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_const_function__MotorMsg__acceleration(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__assign_function__MotorMsg__acceleration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_function__MotorMsg__acceleration(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__resize_function__MotorMsg__acceleration(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__MotorMsg_message_member_array[7] = {
  {
    "length",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__MotorMsg, length),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__MotorMsg, mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__MotorMsg, id),  // bytes offset in struct
    NULL,  // default value
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__size_function__MotorMsg__id,  // size() function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_const_function__MotorMsg__id,  // get_const(index) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_function__MotorMsg__id,  // get(index) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__fetch_function__MotorMsg__id,  // fetch(index, &value) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__assign_function__MotorMsg__id,  // assign(index, value) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__resize_function__MotorMsg__id  // resize(index) function pointer
  },
  {
    "speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__MotorMsg, speed),  // bytes offset in struct
    NULL,  // default value
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__size_function__MotorMsg__speed,  // size() function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_const_function__MotorMsg__speed,  // get_const(index) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_function__MotorMsg__speed,  // get(index) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__fetch_function__MotorMsg__speed,  // fetch(index, &value) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__assign_function__MotorMsg__speed,  // assign(index, value) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__resize_function__MotorMsg__speed  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__MotorMsg, position),  // bytes offset in struct
    NULL,  // default value
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__size_function__MotorMsg__position,  // size() function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_const_function__MotorMsg__position,  // get_const(index) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_function__MotorMsg__position,  // get(index) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__fetch_function__MotorMsg__position,  // fetch(index, &value) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__assign_function__MotorMsg__position,  // assign(index, value) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__resize_function__MotorMsg__position  // resize(index) function pointer
  },
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__MotorMsg, type),  // bytes offset in struct
    NULL,  // default value
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__size_function__MotorMsg__type,  // size() function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_const_function__MotorMsg__type,  // get_const(index) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_function__MotorMsg__type,  // get(index) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__fetch_function__MotorMsg__type,  // fetch(index, &value) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__assign_function__MotorMsg__type,  // assign(index, value) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__resize_function__MotorMsg__type  // resize(index) function pointer
  },
  {
    "acceleration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__MotorMsg, acceleration),  // bytes offset in struct
    NULL,  // default value
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__size_function__MotorMsg__acceleration,  // size() function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_const_function__MotorMsg__acceleration,  // get_const(index) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__get_function__MotorMsg__acceleration,  // get(index) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__fetch_function__MotorMsg__acceleration,  // fetch(index, &value) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__assign_function__MotorMsg__acceleration,  // assign(index, value) function pointer
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__resize_function__MotorMsg__acceleration  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__MotorMsg_message_members = {
  "humanoid_interfaces__msg",  // message namespace
  "MotorMsg",  // message name
  7,  // number of fields
  sizeof(humanoid_interfaces__msg__MotorMsg),
  humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__MotorMsg_message_member_array,  // message members
  humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__MotorMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__MotorMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__MotorMsg_message_type_support_handle = {
  0,
  &humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__MotorMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_humanoid_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, humanoid_interfaces, msg, MotorMsg)() {
  if (!humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__MotorMsg_message_type_support_handle.typesupport_identifier) {
    humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__MotorMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &humanoid_interfaces__msg__MotorMsg__rosidl_typesupport_introspection_c__MotorMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
