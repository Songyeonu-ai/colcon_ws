// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/Robocuplocalization25.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPLOCALIZATION25__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPLOCALIZATION25__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Robocuplocalization25 in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__Robocuplocalization25
{
  double ball_x;
  double ball_y;
  double ball_speed_x;
  double ball_speed_y;
  double robot_x;
  double robot_y;
  double obstacle_x;
  double obstacle_y;
  double obstacle0_x;
  double obstacle0_y;
  double obstacle1_x;
  double obstacle1_y;
  double obstacle2_x;
  double obstacle2_y;
  double obstacle3_x;
  double obstacle3_y;
} humanoid_interfaces__msg__Robocuplocalization25;

// Struct for a sequence of humanoid_interfaces__msg__Robocuplocalization25.
typedef struct humanoid_interfaces__msg__Robocuplocalization25__Sequence
{
  humanoid_interfaces__msg__Robocuplocalization25 * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__Robocuplocalization25__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPLOCALIZATION25__STRUCT_H_
