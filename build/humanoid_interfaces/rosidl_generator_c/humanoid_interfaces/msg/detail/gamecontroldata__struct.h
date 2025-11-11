// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/Gamecontroldata.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLDATA__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLDATA__STRUCT_H_

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

/// Struct defined in msg/Gamecontroldata in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__Gamecontroldata
{
  int64_t robotnum;
  int64_t position;
  int64_t myteam;
  int64_t state;
  int64_t myside;
  int64_t iskickoff;
  int64_t readytime;
  int64_t penalty;
  int64_t secondstate;
  rosidl_runtime_c__int64__Sequence secondinfo;
} humanoid_interfaces__msg__Gamecontroldata;

// Struct for a sequence of humanoid_interfaces__msg__Gamecontroldata.
typedef struct humanoid_interfaces__msg__Gamecontroldata__Sequence
{
  humanoid_interfaces__msg__Gamecontroldata * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__Gamecontroldata__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__GAMECONTROLDATA__STRUCT_H_
