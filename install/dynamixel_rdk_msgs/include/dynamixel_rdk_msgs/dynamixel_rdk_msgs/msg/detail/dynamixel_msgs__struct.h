// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dynamixel_rdk_msgs:msg/DynamixelMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_MSGS__STRUCT_H_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_MSGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/DynamixelMsgs in the package dynamixel_rdk_msgs.
/**
  * Messages
  *
  * Dynamixel Control Message
 */
typedef struct dynamixel_rdk_msgs__msg__DynamixelMsgs
{
  std_msgs__msg__Header header;
  /// radian
  float goal_position;
  /// radian/s^2
  float profile_acceleration;
  /// radian/s
  float profile_velocity;
} dynamixel_rdk_msgs__msg__DynamixelMsgs;

// Struct for a sequence of dynamixel_rdk_msgs__msg__DynamixelMsgs.
typedef struct dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence
{
  dynamixel_rdk_msgs__msg__DynamixelMsgs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_MSGS__STRUCT_H_
