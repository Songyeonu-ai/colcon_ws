// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/IkLTCMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IK_LTC_MSG__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__IK_LTC_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/IkLTCMsg in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__IkLTCMsg
{
  double entire_time;
  double swing_gain_l;
  double swing_gain_r;
  int64_t warning;
  int64_t safe;
  double landing_time_l;
  double landing_time_r;
  double landing_error_l;
  double landing_error_r;
} humanoid_interfaces__msg__IkLTCMsg;

// Struct for a sequence of humanoid_interfaces__msg__IkLTCMsg.
typedef struct humanoid_interfaces__msg__IkLTCMsg__Sequence
{
  humanoid_interfaces__msg__IkLTCMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__IkLTCMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IK_LTC_MSG__STRUCT_H_
