// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dynamixel_rdk_msgs:msg/DynamixelStatusMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_STATUS_MSGS__STRUCT_H_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_STATUS_MSGS__STRUCT_H_

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

/// Struct defined in msg/DynamixelStatusMsgs in the package dynamixel_rdk_msgs.
/**
  * Messages
  *
  * Dynamixel Status Message
 */
typedef struct dynamixel_rdk_msgs__msg__DynamixelStatusMsgs
{
  std_msgs__msg__Header header;
  int32_t id;
  /// 1 if torque is enabled, 0 if disabled
  bool torque_enabled;
  /// https://emanual.robotis.com/docs/kr/dxl/mx/mx-106-2/#shutdown63
  uint8_t error_status;
  /// radians
  double present_position;
  /// radians/s
  double present_velocity;
  /// radians/s^2
  double present_acceleration;
  /// mA
  double present_current;
  /// V
  double present_voltage;
  /// C
  double present_temperature;
  /// radians
  double min_max_position[2];
} dynamixel_rdk_msgs__msg__DynamixelStatusMsgs;

// Struct for a sequence of dynamixel_rdk_msgs__msg__DynamixelStatusMsgs.
typedef struct dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence
{
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_STATUS_MSGS__STRUCT_H_
