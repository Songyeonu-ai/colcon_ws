// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from humanoid_interfaces:msg/Robocupvision25.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "humanoid_interfaces/msg/detail/robocupvision25__rosidl_typesupport_introspection_c.h"
#include "humanoid_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "humanoid_interfaces/msg/detail/robocupvision25__functions.h"
#include "humanoid_interfaces/msg/detail/robocupvision25__struct.h"


// Include directives for member types
// Member `robot_vec_x`
// Member `robot_vec_y`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__Robocupvision25_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  humanoid_interfaces__msg__Robocupvision25__init(message_memory);
}

void humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__Robocupvision25_fini_function(void * message_memory)
{
  humanoid_interfaces__msg__Robocupvision25__fini(message_memory);
}

size_t humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__size_function__Robocupvision25__robot_vec_x(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__get_const_function__Robocupvision25__robot_vec_x(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__get_function__Robocupvision25__robot_vec_x(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__fetch_function__Robocupvision25__robot_vec_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__get_const_function__Robocupvision25__robot_vec_x(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__assign_function__Robocupvision25__robot_vec_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__get_function__Robocupvision25__robot_vec_x(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__resize_function__Robocupvision25__robot_vec_x(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__size_function__Robocupvision25__robot_vec_y(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__get_const_function__Robocupvision25__robot_vec_y(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__get_function__Robocupvision25__robot_vec_y(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__fetch_function__Robocupvision25__robot_vec_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__get_const_function__Robocupvision25__robot_vec_y(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__assign_function__Robocupvision25__robot_vec_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__get_function__Robocupvision25__robot_vec_y(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__resize_function__Robocupvision25__robot_vec_y(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__Robocupvision25_message_member_array[13] = {
  {
    "ball_cam_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Robocupvision25, ball_cam_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ball_cam_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Robocupvision25, ball_cam_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ball_2d_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Robocupvision25, ball_2d_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ball_2d_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Robocupvision25, ball_2d_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ball_d",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Robocupvision25, ball_d),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pan",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Robocupvision25, pan),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tilt",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Robocupvision25, tilt),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ball_speed_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Robocupvision25, ball_speed_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ball_speed_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Robocupvision25, ball_speed_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_vec_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Robocupvision25, robot_vec_x),  // bytes offset in struct
    NULL,  // default value
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__size_function__Robocupvision25__robot_vec_x,  // size() function pointer
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__get_const_function__Robocupvision25__robot_vec_x,  // get_const(index) function pointer
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__get_function__Robocupvision25__robot_vec_x,  // get(index) function pointer
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__fetch_function__Robocupvision25__robot_vec_x,  // fetch(index, &value) function pointer
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__assign_function__Robocupvision25__robot_vec_x,  // assign(index, value) function pointer
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__resize_function__Robocupvision25__robot_vec_x  // resize(index) function pointer
  },
  {
    "robot_vec_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Robocupvision25, robot_vec_y),  // bytes offset in struct
    NULL,  // default value
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__size_function__Robocupvision25__robot_vec_y,  // size() function pointer
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__get_const_function__Robocupvision25__robot_vec_y,  // get_const(index) function pointer
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__get_function__Robocupvision25__robot_vec_y,  // get(index) function pointer
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__fetch_function__Robocupvision25__robot_vec_y,  // fetch(index, &value) function pointer
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__assign_function__Robocupvision25__robot_vec_y,  // assign(index, value) function pointer
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__resize_function__Robocupvision25__robot_vec_y  // resize(index) function pointer
  },
  {
    "ball_speed_level",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Robocupvision25, ball_speed_level),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "scan_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces__msg__Robocupvision25, scan_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__Robocupvision25_message_members = {
  "humanoid_interfaces__msg",  // message namespace
  "Robocupvision25",  // message name
  13,  // number of fields
  sizeof(humanoid_interfaces__msg__Robocupvision25),
  humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__Robocupvision25_message_member_array,  // message members
  humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__Robocupvision25_init_function,  // function to initialize message memory (memory has to be allocated)
  humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__Robocupvision25_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__Robocupvision25_message_type_support_handle = {
  0,
  &humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__Robocupvision25_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_humanoid_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, humanoid_interfaces, msg, Robocupvision25)() {
  if (!humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__Robocupvision25_message_type_support_handle.typesupport_identifier) {
    humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__Robocupvision25_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &humanoid_interfaces__msg__Robocupvision25__rosidl_typesupport_introspection_c__Robocupvision25_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
