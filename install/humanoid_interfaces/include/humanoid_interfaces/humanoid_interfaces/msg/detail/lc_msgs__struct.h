// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/LCMsgs.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__LC_MSGS__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__LC_MSGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'r_lc_data'
// Member 'l_lc_data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/LCMsgs in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__LCMsgs
{
  rosidl_runtime_c__int32__Sequence r_lc_data;
  rosidl_runtime_c__int32__Sequence l_lc_data;
} humanoid_interfaces__msg__LCMsgs;

// Struct for a sequence of humanoid_interfaces__msg__LCMsgs.
typedef struct humanoid_interfaces__msg__LCMsgs__Sequence
{
  humanoid_interfaces__msg__LCMsgs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__LCMsgs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__LC_MSGS__STRUCT_H_
