// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dynamixel_rdk_msgs:msg/DynamixelMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_MSGS__FUNCTIONS_H_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_MSGS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dynamixel_rdk_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "dynamixel_rdk_msgs/msg/detail/dynamixel_msgs__struct.h"

/// Initialize msg/DynamixelMsgs message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dynamixel_rdk_msgs__msg__DynamixelMsgs
 * )) before or use
 * dynamixel_rdk_msgs__msg__DynamixelMsgs__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
bool
dynamixel_rdk_msgs__msg__DynamixelMsgs__init(dynamixel_rdk_msgs__msg__DynamixelMsgs * msg);

/// Finalize msg/DynamixelMsgs message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
void
dynamixel_rdk_msgs__msg__DynamixelMsgs__fini(dynamixel_rdk_msgs__msg__DynamixelMsgs * msg);

/// Create msg/DynamixelMsgs message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dynamixel_rdk_msgs__msg__DynamixelMsgs__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
dynamixel_rdk_msgs__msg__DynamixelMsgs *
dynamixel_rdk_msgs__msg__DynamixelMsgs__create();

/// Destroy msg/DynamixelMsgs message.
/**
 * It calls
 * dynamixel_rdk_msgs__msg__DynamixelMsgs__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
void
dynamixel_rdk_msgs__msg__DynamixelMsgs__destroy(dynamixel_rdk_msgs__msg__DynamixelMsgs * msg);

/// Check for msg/DynamixelMsgs message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
bool
dynamixel_rdk_msgs__msg__DynamixelMsgs__are_equal(const dynamixel_rdk_msgs__msg__DynamixelMsgs * lhs, const dynamixel_rdk_msgs__msg__DynamixelMsgs * rhs);

/// Copy a msg/DynamixelMsgs message.
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
dynamixel_rdk_msgs__msg__DynamixelMsgs__copy(
  const dynamixel_rdk_msgs__msg__DynamixelMsgs * input,
  dynamixel_rdk_msgs__msg__DynamixelMsgs * output);

/// Initialize array of msg/DynamixelMsgs messages.
/**
 * It allocates the memory for the number of elements and calls
 * dynamixel_rdk_msgs__msg__DynamixelMsgs__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
bool
dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__init(dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * array, size_t size);

/// Finalize array of msg/DynamixelMsgs messages.
/**
 * It calls
 * dynamixel_rdk_msgs__msg__DynamixelMsgs__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
void
dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__fini(dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * array);

/// Create array of msg/DynamixelMsgs messages.
/**
 * It allocates the memory for the array and calls
 * dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence *
dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__create(size_t size);

/// Destroy array of msg/DynamixelMsgs messages.
/**
 * It calls
 * dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
void
dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__destroy(dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * array);

/// Check for msg/DynamixelMsgs message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_rdk_msgs
bool
dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__are_equal(const dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * lhs, const dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * rhs);

/// Copy an array of msg/DynamixelMsgs messages.
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
dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__copy(
  const dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * input,
  dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_MSGS__FUNCTIONS_H_
