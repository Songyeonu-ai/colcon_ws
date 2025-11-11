// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/Udp2master.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__UDP2MASTER__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__UDP2MASTER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Udp2master in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__Udp2master
{
  int64_t robotnum;
  int64_t robotcase;
  int64_t localx;
  int64_t localy;
  int64_t localyaw;
  int64_t ballx;
  int64_t bally;
  int64_t balldist;
  int64_t balltheta;
} humanoid_interfaces__msg__Udp2master;

// Struct for a sequence of humanoid_interfaces__msg__Udp2master.
typedef struct humanoid_interfaces__msg__Udp2master__Sequence
{
  humanoid_interfaces__msg__Udp2master * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__Udp2master__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__UDP2MASTER__STRUCT_H_
