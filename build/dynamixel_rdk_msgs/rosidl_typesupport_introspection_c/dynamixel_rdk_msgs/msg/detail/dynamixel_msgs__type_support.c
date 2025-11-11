// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dynamixel_rdk_msgs:msg/DynamixelMsgs.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_msgs__rosidl_typesupport_introspection_c.h"
#include "dynamixel_rdk_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_msgs__functions.h"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_msgs__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dynamixel_rdk_msgs__msg__DynamixelMsgs__rosidl_typesupport_introspection_c__DynamixelMsgs_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dynamixel_rdk_msgs__msg__DynamixelMsgs__init(message_memory);
}

void dynamixel_rdk_msgs__msg__DynamixelMsgs__rosidl_typesupport_introspection_c__DynamixelMsgs_fini_function(void * message_memory)
{
  dynamixel_rdk_msgs__msg__DynamixelMsgs__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dynamixel_rdk_msgs__msg__DynamixelMsgs__rosidl_typesupport_introspection_c__DynamixelMsgs_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs__msg__DynamixelMsgs, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs__msg__DynamixelMsgs, goal_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "profile_acceleration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs__msg__DynamixelMsgs, profile_acceleration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "profile_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_rdk_msgs__msg__DynamixelMsgs, profile_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dynamixel_rdk_msgs__msg__DynamixelMsgs__rosidl_typesupport_introspection_c__DynamixelMsgs_message_members = {
  "dynamixel_rdk_msgs__msg",  // message namespace
  "DynamixelMsgs",  // message name
  4,  // number of fields
  sizeof(dynamixel_rdk_msgs__msg__DynamixelMsgs),
  dynamixel_rdk_msgs__msg__DynamixelMsgs__rosidl_typesupport_introspection_c__DynamixelMsgs_message_member_array,  // message members
  dynamixel_rdk_msgs__msg__DynamixelMsgs__rosidl_typesupport_introspection_c__DynamixelMsgs_init_function,  // function to initialize message memory (memory has to be allocated)
  dynamixel_rdk_msgs__msg__DynamixelMsgs__rosidl_typesupport_introspection_c__DynamixelMsgs_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dynamixel_rdk_msgs__msg__DynamixelMsgs__rosidl_typesupport_introspection_c__DynamixelMsgs_message_type_support_handle = {
  0,
  &dynamixel_rdk_msgs__msg__DynamixelMsgs__rosidl_typesupport_introspection_c__DynamixelMsgs_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dynamixel_rdk_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_rdk_msgs, msg, DynamixelMsgs)() {
  dynamixel_rdk_msgs__msg__DynamixelMsgs__rosidl_typesupport_introspection_c__DynamixelMsgs_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!dynamixel_rdk_msgs__msg__DynamixelMsgs__rosidl_typesupport_introspection_c__DynamixelMsgs_message_type_support_handle.typesupport_identifier) {
    dynamixel_rdk_msgs__msg__DynamixelMsgs__rosidl_typesupport_introspection_c__DynamixelMsgs_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dynamixel_rdk_msgs__msg__DynamixelMsgs__rosidl_typesupport_introspection_c__DynamixelMsgs_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
