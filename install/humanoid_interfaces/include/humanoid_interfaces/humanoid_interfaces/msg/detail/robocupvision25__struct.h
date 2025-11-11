// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/Robocupvision25.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_vec_x'
// Member 'robot_vec_y'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Robocupvision25 in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__Robocupvision25
{
  int64_t ball_cam_x;
  int64_t ball_cam_y;
  double ball_2d_x;
  double ball_2d_y;
  double ball_d;
  double pan;
  double tilt;
  double ball_speed_x;
  double ball_speed_y;
  rosidl_runtime_c__double__Sequence robot_vec_x;
  rosidl_runtime_c__double__Sequence robot_vec_y;
  int64_t ball_speed_level;
  int64_t scan_mode;
} humanoid_interfaces__msg__Robocupvision25;

// Struct for a sequence of humanoid_interfaces__msg__Robocupvision25.
typedef struct humanoid_interfaces__msg__Robocupvision25__Sequence
{
  humanoid_interfaces__msg__Robocupvision25 * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__Robocupvision25__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25__STRUCT_H_
