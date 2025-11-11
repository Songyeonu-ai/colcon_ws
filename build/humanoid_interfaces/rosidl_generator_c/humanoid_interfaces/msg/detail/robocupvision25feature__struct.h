// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/Robocupvision25feature.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25FEATURE__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25FEATURE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'confidence'
// Member 'distance'
// Member 'point_vec_x'
// Member 'point_vec_y'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Robocupvision25feature in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__Robocupvision25feature
{
  rosidl_runtime_c__double__Sequence confidence;
  rosidl_runtime_c__double__Sequence distance;
  rosidl_runtime_c__double__Sequence point_vec_x;
  rosidl_runtime_c__double__Sequence point_vec_y;
} humanoid_interfaces__msg__Robocupvision25feature;

// Struct for a sequence of humanoid_interfaces__msg__Robocupvision25feature.
typedef struct humanoid_interfaces__msg__Robocupvision25feature__Sequence
{
  humanoid_interfaces__msg__Robocupvision25feature * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__Robocupvision25feature__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ROBOCUPVISION25FEATURE__STRUCT_H_
