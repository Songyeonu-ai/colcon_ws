// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dynamixel_rdk_msgs:msg/DynamixelControlMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_CONTROL_MSGS__FUNCTIONS_H_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_CONTROL_MSGS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dynamixel_rdk_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "dynamixel_rdk_msgs/msg/detail/dynamixel_control_msgs__struct.h"

/// Initialize msg/DynamixelControlMsgs message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dynamixel_rdk_msgs__msg__DynamixelControlMsgs
 * )) before or use
 * dynamixel_rdk_msgs__msg__DynamixelControlMsgs__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
bool
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__init(dynamixel_rdk_msgs__msg__DynamixelControlMsgs * msg);

/// Finalize msg/DynamixelControlMsgs message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
void
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__fini(dynamixel_rdk_msgs__msg__DynamixelControlMsgs * msg);

/// Create msg/DynamixelControlMsgs message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dynamixel_rdk_msgs__msg__DynamixelControlMsgs__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
dynamixel_rdk_msgs__msg__DynamixelControlMsgs *
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__create();

/// Destroy msg/DynamixelControlMsgs message.
/**
 * It calls
 * dynamixel_rdk_msgs__msg__DynamixelControlMsgs__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
void
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__destroy(dynamixel_rdk_msgs__msg__DynamixelControlMsgs * msg);

/// Check for msg/DynamixelControlMsgs message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
bool
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__are_equal(const dynamixel_rdk_msgs__msg__DynamixelControlMsgs * lhs, const dynamixel_rdk_msgs__msg__DynamixelControlMsgs * rhs);

/// Copy a msg/DynamixelControlMsgs message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
bool
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__copy(
  const dynamixel_rdk_msgs__msg__DynamixelControlMsgs * input,
  dynamixel_rdk_msgs__msg__DynamixelControlMsgs * output);

/// Initialize array of msg/DynamixelControlMsgs messages.
/**
 * It allocates the memory for the number of elements and calls
 * dynamixel_rdk_msgs__msg__DynamixelControlMsgs__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
bool
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence__init(dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence * array, size_t size);

/// Finalize array of msg/DynamixelControlMsgs messages.
/**
 * It calls
 * dynamixel_rdk_msgs__msg__DynamixelControlMsgs__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
void
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence__fini(dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence * array);

/// Create array of msg/DynamixelControlMsgs messages.
/**
 * It allocates the memory for the array and calls
 * dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence *
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence__create(size_t size);

/// Destroy array of msg/DynamixelControlMsgs messages.
/**
 * It calls
 * dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
void
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence__destroy(dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence * array);

/// Check for msg/DynamixelControlMsgs message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
bool
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence__are_equal(const dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence * lhs, const dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence * rhs);

/// Copy an array of msg/DynamixelControlMsgs messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
bool
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence__copy(
  const dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence * input,
  dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_CONTROL_MSGS__FUNCTIONS_H_
