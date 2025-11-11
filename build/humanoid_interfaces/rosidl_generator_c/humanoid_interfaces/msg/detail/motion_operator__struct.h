// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/MotionOperator.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MOTION_OPERATOR__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__MOTION_OPERATOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotionOperator in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__MotionOperator
{
  int32_t motion_num;
  int32_t motion_end;
} humanoid_interfaces__msg__MotionOperator;

// Struct for a sequence of humanoid_interfaces__msg__MotionOperator.
typedef struct humanoid_interfaces__msg__MotionOperator__Sequence
{
  humanoid_interfaces__msg__MotionOperator * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__MotionOperator__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MOTION_OPERATOR__STRUCT_H_
