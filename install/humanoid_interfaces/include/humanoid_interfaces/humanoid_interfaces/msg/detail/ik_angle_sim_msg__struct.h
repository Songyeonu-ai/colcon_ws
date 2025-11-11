// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/IkAngleSimMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_ANGLE_SIM_MSG__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_ANGLE_SIM_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/IkAngleSimMsg in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__IkAngleSimMsg
{
  double ang_l_0;
  double ang_l_1;
  double ang_l_2;
  double ang_l_3;
  double ang_l_4;
  double ang_l_5;
  double ang_l_6;
  double ang_l_7;
  double ang_l_8;
  double ang_r_0;
  double ang_r_1;
  double ang_r_2;
  double ang_r_3;
  double ang_r_4;
  double ang_r_5;
  double ang_r_6;
  double ang_r_7;
  double ang_r_8;
} humanoid_interfaces__msg__IkAngleSimMsg;

// Struct for a sequence of humanoid_interfaces__msg__IkAngleSimMsg.
typedef struct humanoid_interfaces__msg__IkAngleSimMsg__Sequence
{
  humanoid_interfaces__msg__IkAngleSimMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__IkAngleSimMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_ANGLE_SIM_MSG__STRUCT_H_
