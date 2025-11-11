// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from humanoid_interfaces:msg/Gamecontroldata.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "humanoid_interfaces/msg/detail/gamecontroldata__rosidl_typesupport_introspection_c.h"
#include "humanoid_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "humanoid_interfaces/msg/detail/gamecontroldata__functions.h"
#include "humanoid_interfaces/msg/detail/gamecontroldata__struct.h"


// Include directives for member types
// Member `secondinfo`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__Gamecontroldata_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  humanoid_interfaces__msg__Gamecontroldata__init(message_memory);
}

void humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__Gamecontroldata_fini_function(void * message_memory)
{
  humanoid_interfaces__msg__Gamecontroldata__fini(message_memory);
}

size_t humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__size_function__Gamecontroldata__secondinfo(
  const void * untyped_member)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return member->size;
}

const void * humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__get_const_function__Gamecontroldata__secondinfo(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void * humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__get_function__Gamecontroldata__secondinfo(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__fetch_function__Gamecontroldata__secondinfo(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int64_t * item =
    ((const int64_t *)
    humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__get_const_function__Gamecontroldata__secondinfo(untyped_member, index));
  int64_t * value =
    (int64_t *)(untyped_value);
  *value = *item;
}

void humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__assign_function__Gamecontroldata__secondinfo(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int64_t * item =
    ((int64_t *)
    humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__get_function__Gamecontroldata__secondinfo(untyped_member, index));
  const int64_t * value =
    (const int64_t *)(untyped_value);
  *item = *value;
}

bool humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__resize_function__Gamecontroldata__secondinfo(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  rosidl_runtime_c__int64__Sequence__fini(member);
  return rosidl_runtime_c__int64__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__Gamecontroldata_message_member_array[10] = {
  {
    "robotnum",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Gamecontroldata, robotnum),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Gamecontroldata, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "myteam",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Gamecontroldata, myteam),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Gamecontroldata, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "myside",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Gamecontroldata, myside),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "iskickoff",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Gamecontroldata, iskickoff),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "readytime",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Gamecontroldata, readytime),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "penalty",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Gamecontroldata, penalty),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "secondstate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Gamecontroldata, secondstate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "secondinfo",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Gamecontroldata, secondinfo),  // bytes offset in struct
    NULL,  // default value
    humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__size_function__Gamecontroldata__secondinfo,  // size() function pointer
    humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__get_const_function__Gamecontroldata__secondinfo,  // get_const(index) function pointer
    humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__get_function__Gamecontroldata__secondinfo,  // get(index) function pointer
    humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__fetch_function__Gamecontroldata__secondinfo,  // fetch(index, &value) function pointer
    humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__assign_function__Gamecontroldata__secondinfo,  // assign(index, value) function pointer
    humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__resize_function__Gamecontroldata__secondinfo  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__Gamecontroldata_message_members = {
  "humanoid_interfaces__msg",  // message namespace
  "Gamecontroldata",  // message name
  10,  // number of fields
  sizeof(humanoid_interfaces__msg__Gamecontroldata),
  humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__Gamecontroldata_message_member_array,  // message members
  humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__Gamecontroldata_init_function,  // function to initialize message memory (memory has to be allocated)
  humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__Gamecontroldata_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__Gamecontroldata_message_type_support_handle = {
  0,
  &humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__Gamecontroldata_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_humanoid_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, humanoid_interfaces, msg, Gamecontroldata)() {
  if (!humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__Gamecontroldata_message_type_support_handle.typesupport_identifier) {
    humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__Gamecontroldata_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &humanoid_interfaces__msg__Gamecontroldata__rosidl_typesupport_introspection_c__Gamecontroldata_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
