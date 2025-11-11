// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/Motionend.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MOTIONEND__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__MOTIONEND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Motionend in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__Motionend
{
  int32_t motionend;
  int32_t motioning;
  int32_t motionstarting;
  int32_t motionending;
  int32_t motionnum;
} humanoid_interfaces__msg__Motionend;

// Struct for a sequence of humanoid_interfaces__msg__Motionend.
typedef struct humanoid_interfaces__msg__Motionend__Sequence
{
  humanoid_interfaces__msg__Motionend * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__Motionend__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MOTIONEND__STRUCT_H_
