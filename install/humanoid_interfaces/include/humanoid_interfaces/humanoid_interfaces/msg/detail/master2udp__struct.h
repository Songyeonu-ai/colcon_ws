// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/Master2udp.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MASTER2UDP__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__MASTER2UDP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Master2udp in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__Master2udp
{
  int64_t robotnum;
  int64_t robotstate;
  int64_t robotcoorx;
  int64_t robotcoory;
  int64_t robotimuyaw;
  int64_t balldist;
  int64_t ballcoorx;
  int64_t ballcoory;
  int64_t myteam;
} humanoid_interfaces__msg__Master2udp;

// Struct for a sequence of humanoid_interfaces__msg__Master2udp.
typedef struct humanoid_interfaces__msg__Master2udp__Sequence
{
  humanoid_interfaces__msg__Master2udp * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__Master2udp__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MASTER2UDP__STRUCT_H_
