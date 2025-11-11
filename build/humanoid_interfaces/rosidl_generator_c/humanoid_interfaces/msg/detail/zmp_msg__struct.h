// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from humanoid_interfaces:msg/ZmpMsg.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__ZMP_MSG__STRUCT_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__ZMP_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ZmpMsg in the package humanoid_interfaces.
typedef struct humanoid_interfaces__msg__ZmpMsg
{
  double left_x_zmp;
  double left_y_zmp;
  double right_x_zmp;
  double right_y_zmp;
  double total_x_zmp;
  double total_y_zmp;
  bool left_foot;
  bool right_foot;
  bool both_feet;
} humanoid_interfaces__msg__ZmpMsg;

// Struct for a sequence of humanoid_interfaces__msg__ZmpMsg.
typedef struct humanoid_interfaces__msg__ZmpMsg__Sequence
{
  humanoid_interfaces__msg__ZmpMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} humanoid_interfaces__msg__ZmpMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__ZMP_MSG__STRUCT_H_
