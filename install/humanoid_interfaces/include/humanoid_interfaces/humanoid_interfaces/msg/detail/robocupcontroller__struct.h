// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/Robocupcontroller.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPCONTROLLER__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPCONTROLLER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'secondinfo'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Robocupcontroller in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__Robocupcontroller
{
  int64_t robotnum;
  int64_t state;
  int64_t readytime;
  int64_t penalty;
  int64_t firsthalf;
  int64_t firstside;
  int64_t kickoffteam;
  int64_t secondstate;
  rosidl_runtime_c__int64__Sequence secondinfo;
} humanoid_interfaces__msg__Robocupcontroller;

// Struct for a sequence of humanoid_interfaces__msg__Robocupcontroller.
typedef struct humanoid_interfaces__msg__Robocupcontroller__Sequence
{
  humanoid_interfaces__msg__Robocupcontroller * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__Robocupcontroller__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPCONTROLLER__STRUCT_H_
