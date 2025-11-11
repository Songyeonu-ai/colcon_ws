// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/ImuMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__IMU_MSG__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__IMU_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ImuMsg in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__ImuMsg
{
  float pitch;
  float roll;
  float yaw;
  float q_x;
  float q_y;
  float q_z;
  float q_w;
  float x_gyro;
  float y_gyro;
  float z_gyro;
  float x_acc;
  float y_acc;
  float z_acc;
  float x_vel;
  float y_vel;
  float z_vel;
  float x_pos;
  float y_pos;
  float z_pos;
} humanoid_interfaces__msg__ImuMsg;

// Struct for a sequence of humanoid_interfaces__msg__ImuMsg.
typedef struct humanoid_interfaces__msg__ImuMsg__Sequence
{
  humanoid_interfaces__msg__ImuMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__ImuMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__IMU_MSG__STRUCT_H_
