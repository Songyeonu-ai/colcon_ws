// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from humanoid_interfaces:msg/MotionOperator.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "humanoid_interfaces/msg/detail/motion_operator__rosidl_typesupport_introspection_c.h"
#include "humanoid_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "humanoid_interfaces/msg/detail/motion_operator__functions.h"
#include "humanoid_interfaces/msg/detail/motion_operator__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void humanoid_interfaces__msg__MotionOperator__rosidl_typesupport_introspection_c__MotionOperator_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  humanoid_interfaces__msg__MotionOperator__init(message_memory);
}

void humanoid_interfaces__msg__MotionOperator__rosidl_typesupport_introspection_c__MotionOperator_fini_function(void * message_memory)
{
  humanoid_interfaces__msg__MotionOperator__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember humanoid_interfaces__msg__MotionOperator__rosidl_typesupport_introspection_c__MotionOperator_message_member_array[2] = {
  {
    "motion_num",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__MotionOperator, motion_num),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motion_end",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__MotionOperator, motion_end),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers humanoid_interfaces__msg__MotionOperator__rosidl_typesupport_introspection_c__MotionOperator_message_members = {
  "humanoid_interfaces__msg",  // message namespace
  "MotionOperator",  // message name
  2,  // number of fields
  sizeof(humanoid_interfaces__msg__MotionOperator),
  humanoid_interfaces__msg__MotionOperator__rosidl_typesupport_introspection_c__MotionOperator_message_member_array,  // message members
  humanoid_interfaces__msg__MotionOperator__rosidl_typesupport_introspection_c__MotionOperator_init_function,  // function to initialize message memory (memory has to be allocated)
  humanoid_interfaces__msg__MotionOperator__rosidl_typesupport_introspection_c__MotionOperator_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t humanoid_interfaces__msg__MotionOperator__rosidl_typesupport_introspection_c__MotionOperator_message_type_support_handle = {
  0,
  &humanoid_interfaces__msg__MotionOperator__rosidl_typesupport_introspection_c__MotionOperator_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_humanoid_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, humanoid_interfaces, msg, MotionOperator)() {
  if (!humanoid_interfaces__msg__MotionOperator__rosidl_typesupport_introspection_c__MotionOperator_message_type_support_handle.typesupport_identifier) {
    humanoid_interfaces__msg__MotionOperator__rosidl_typesupport_introspection_c__MotionOperator_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &humanoid_interfaces__msg__MotionOperator__rosidl_typesupport_introspection_c__MotionOperator_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
