// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dynamixel_rdk_msgs:msg/DynamixelStatusMsgs.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_status_msgs__rosidl_typesupport_introspection_c.h"
#include "dynamixel_rdk_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_status_msgs__functions.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_status_msgs__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__DynamixelStatusMsgs_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__init(message_memory);
}

void dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__DynamixelStatusMsgs_fini_function(void * message_memory)
{
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__fini(message_memory);
}

size_t dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__size_function__DynamixelStatusMsgs__min_max_position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__get_const_function__DynamixelStatusMsgs__min_max_position(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__get_function__DynamixelStatusMsgs__min_max_position(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__fetch_function__DynamixelStatusMsgs__min_max_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__get_const_function__DynamixelStatusMsgs__min_max_position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__assign_function__DynamixelStatusMsgs__min_max_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__get_function__DynamixelStatusMsgs__min_max_position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__DynamixelStatusMsgs_message_member_array[11] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs, header),  // bytes offset in struct
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
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "torque_enabled",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs, torque_enabled),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs, error_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "present_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs, present_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "present_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs, present_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "present_acceleration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs, present_acceleration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "present_current",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs, present_current),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "present_voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs, present_voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "present_temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs, present_temperature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "min_max_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs, min_max_position),  // bytes offset in struct
    NULL,  // default value
    dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__size_function__DynamixelStatusMsgs__min_max_position,  // size() function pointer
    dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__get_const_function__DynamixelStatusMsgs__min_max_position,  // get_const(index) function pointer
    dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__get_function__DynamixelStatusMsgs__min_max_position,  // get(index) function pointer
    dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__fetch_function__DynamixelStatusMsgs__min_max_position,  // fetch(index, &value) function pointer
    dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__assign_function__DynamixelStatusMsgs__min_max_position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__DynamixelStatusMsgs_message_members = {
  "dynamixel_rdk_msgs__msg",  // message namespace
  "DynamixelStatusMsgs",  // message name
  11,  // number of fields
  sizeof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs),
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__DynamixelStatusMsgs_message_member_array,  // message members
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__DynamixelStatusMsgs_init_function,  // function to initialize message memory (memory has to be allocated)
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__DynamixelStatusMsgs_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__DynamixelStatusMsgs_message_type_support_handle = {
  0,
  &dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__DynamixelStatusMsgs_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dynamixel_rdk_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_rdk_msgs, msg, DynamixelStatusMsgs)() {
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__DynamixelStatusMsgs_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__DynamixelStatusMsgs_message_type_support_handle.typesupport_identifier) {
    dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__DynamixelStatusMsgs_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__rosidl_typesupport_introspection_c__DynamixelStatusMsgs_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
