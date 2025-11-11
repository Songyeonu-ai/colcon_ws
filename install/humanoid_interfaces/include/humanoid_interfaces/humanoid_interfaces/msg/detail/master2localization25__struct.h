// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/Master2localization25.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MASTER2LOCALIZATION25__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__MASTER2LOCALIZATION25__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Master2localization25 in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__Master2localization25
{
  int64_t goalx;
  int64_t goaly;
  int64_t targetx;
  int64_t targety;
  int64_t robotallowanceerror;
  int64_t robotcaliradius;
} humanoid_interfaces__msg__Master2localization25;

// Struct for a sequence of humanoid_interfaces__msg__Master2localization25.
typedef struct humanoid_interfaces__msg__Master2localization25__Sequence
{
  humanoid_interfaces__msg__Master2localization25 * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__Master2localization25__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MASTER2LOCALIZATION25__STRUCT_H_
