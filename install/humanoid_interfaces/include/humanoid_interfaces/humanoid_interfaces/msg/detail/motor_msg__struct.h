// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/MotorMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MOTOR_MSG__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__MOTOR_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'id'
// Member 'speed'
// Member 'position'
// Member 'type'
// Member 'acceleration'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/MotorMsg in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__MotorMsg
{
  int32_t length;
  int32_t mode;
  rosidl_runtime_c__int32__Sequence id;
  rosidl_runtime_c__int32__Sequence speed;
  rosidl_runtime_c__int32__Sequence position;
  rosidl_runtime_c__int32__Sequence type;
  rosidl_runtime_c__int32__Sequence acceleration;
} humanoid_interfaces__msg__MotorMsg;

// Struct for a sequence of humanoid_interfaces__msg__MotorMsg.
typedef struct humanoid_interfaces__msg__MotorMsg__Sequence
{
  humanoid_interfaces__msg__MotorMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__MotorMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MOTOR_MSG__STRUCT_H_
