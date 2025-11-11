// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/Motionnum.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MOTIONNUM__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__MOTIONNUM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Motionnum in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__Motionnum
{
  int32_t motionmode;
  int32_t motionnum;
} humanoid_interfaces__msg__Motionnum;

// Struct for a sequence of humanoid_interfaces__msg__Motionnum.
typedef struct humanoid_interfaces__msg__Motionnum__Sequence
{
  humanoid_interfaces__msg__Motionnum * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__Motionnum__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MOTIONNUM__STRUCT_H_
