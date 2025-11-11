// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/Master2IkMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MASTER2_IK_MSG__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__MASTER2_IK_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Master2IkMsg in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__Master2IkMsg
{
  double x_length;
  double y_length;
  double yaw;
  double flag;
  int32_t one_x_length;
  int32_t one_y_length;
  int32_t one_yaw;
  int32_t one_step_flag;
} humanoid_interfaces__msg__Master2IkMsg;

// Struct for a sequence of humanoid_interfaces__msg__Master2IkMsg.
typedef struct humanoid_interfaces__msg__Master2IkMsg__Sequence
{
  humanoid_interfaces__msg__Master2IkMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__Master2IkMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MASTER2_IK_MSG__STRUCT_H_
