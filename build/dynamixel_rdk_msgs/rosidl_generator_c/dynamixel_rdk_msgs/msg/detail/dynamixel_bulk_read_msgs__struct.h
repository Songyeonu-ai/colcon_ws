// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dynamixel_rdk_msgs:msg/DynamixelBulkReadMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_BULK_READ_MSGS__STRUCT_H_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_BULK_READ_MSGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'status_msgs'
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_status_msgs__struct.h"

/// Struct defined in msg/DynamixelBulkReadMsgs in the package dynamixel_rdk_msgs.
/**
  * Messages
  *
  * Dynamixel Bulk Read Message
 */
typedef struct dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs
{
  /// DynamixelStatusMsgs
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence status_msgs;
} dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs;

// Struct for a sequence of dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs.
typedef struct dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence
{
  dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_BULK_READ_MSGS__STRUCT_H_
