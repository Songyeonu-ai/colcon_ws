// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from humanoid_interfaces:msg/Pidtuning.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "humanoid_interfaces/msg/detail/pidtuning__rosidl_typesupport_introspection_c.h"
#include "humanoid_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "humanoid_interfaces/msg/detail/pidtuning__functions.h"
#include "humanoid_interfaces/msg/detail/pidtuning__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void humanoid_interfaces__msg__Pidtuning__rosidl_typesupport_introspection_c__Pidtuning_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  humanoid_interfaces__msg__Pidtuning__init(message_memory);
}

void humanoid_interfaces__msg__Pidtuning__rosidl_typesupport_introspection_c__Pidtuning_fini_function(void * message_memory)
{
  humanoid_interfaces__msg__Pidtuning__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember humanoid_interfaces__msg__Pidtuning__rosidl_typesupport_introspection_c__Pidtuning_message_member_array[3] = {
  {
    "kp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Pidtuning, kp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ki",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Pidtuning, ki),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "kd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Pidtuning, kd),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers humanoid_interfaces__msg__Pidtuning__rosidl_typesupport_introspection_c__Pidtuning_message_members = {
  "humanoid_interfaces__msg",  // message namespace
  "Pidtuning",  // message name
  3,  // number of fields
  sizeof(humanoid_interfaces__msg__Pidtuning),
  humanoid_interfaces__msg__Pidtuning__rosidl_typesupport_introspection_c__Pidtuning_message_member_array,  // message members
  humanoid_interfaces__msg__Pidtuning__rosidl_typesupport_introspection_c__Pidtuning_init_function,  // function to initialize message memory (memory has to be allocated)
  humanoid_interfaces__msg__Pidtuning__rosidl_typesupport_introspection_c__Pidtuning_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t humanoid_interfaces__msg__Pidtuning__rosidl_typesupport_introspection_c__Pidtuning_message_type_support_handle = {
  0,
  &humanoid_interfaces__msg__Pidtuning__rosidl_typesupport_introspection_c__Pidtuning_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_humanoid_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, humanoid_interfaces, msg, Pidtuning)() {
  if (!humanoid_interfaces__msg__Pidtuning__rosidl_typesupport_introspection_c__Pidtuning_message_type_support_handle.typesupport_identifier) {
    humanoid_interfaces__msg__Pidtuning__rosidl_typesupport_introspection_c__Pidtuning_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &humanoid_interfaces__msg__Pidtuning__rosidl_typesupport_introspection_c__Pidtuning_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
