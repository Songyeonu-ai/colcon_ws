// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dynamixel_rdk_msgs:msg/DynamixelControlMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_CONTROL_MSGS__STRUCT_H_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_CONTROL_MSGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'motor_control'
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_msgs__struct.h"

/// Struct defined in msg/DynamixelControlMsgs in the package dynamixel_rdk_msgs.
/**
  * Messages
  *
  * Dynamixel Control Message
 */
typedef struct dynamixel_rdk_msgs__msg__DynamixelControlMsgs
{
  /// DynamixelMsgs
  dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence motor_control;
} dynamixel_rdk_msgs__msg__DynamixelControlMsgs;

// Struct for a sequence of dynamixel_rdk_msgs__msg__DynamixelControlMsgs.
typedef struct dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence
{
  dynamixel_rdk_msgs__msg__DynamixelControlMsgs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_CONTROL_MSGS__STRUCT_H_
