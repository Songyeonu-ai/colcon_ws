// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/IkPatternMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_PATTERN_MSG__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_PATTERN_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/IkPatternMsg in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__IkPatternMsg
{
  double xlpattern;
  double ylpattern;
  double zlpattern;
  double slpattern;
  double tlpattern;
  double xrpattern;
  double yrpattern;
  double zrpattern;
  double srpattern;
  double trpattern;
} humanoid_interfaces__msg__IkPatternMsg;

// Struct for a sequence of humanoid_interfaces__msg__IkPatternMsg.
typedef struct humanoid_interfaces__msg__IkPatternMsg__Sequence
{
  humanoid_interfaces__msg__IkPatternMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__IkPatternMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_PATTERN_MSG__STRUCT_H_
