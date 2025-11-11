// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/IkEndMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_END_MSG__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_END_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/IkEndMsg in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__IkEndMsg
{
  int32_t ikend;
  int32_t onestep_end;
} humanoid_interfaces__msg__IkEndMsg;

// Struct for a sequence of humanoid_interfaces__msg__IkEndMsg.
typedef struct humanoid_interfaces__msg__IkEndMsg__Sequence
{
  humanoid_interfaces__msg__IkEndMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__IkEndMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_END_MSG__STRUCT_H_
