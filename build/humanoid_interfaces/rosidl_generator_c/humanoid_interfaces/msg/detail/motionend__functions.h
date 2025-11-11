// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from humanoid_interfaces:msg/Motionend.idl
// generated code does not contain a copyright notice

#ifndef HUMANOID_INTERFACES__MSG__DETAIL__MOTIONEND__FUNCTIONS_H_
#define HUMANOID_INTERFACES__MSG__DETAIL__MOTIONEND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "humanoid_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "humanoid_interfaces/msg/detail/motionend__struct.h"

/// Initialize msg/Motionend message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * humanoid_interfaces__msg__Motionend
 * )) before or use
 * humanoid_interfaces__msg__Motionend__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_humanoid_interfaces
bool
humanoid_interfaces__msg__Motionend__init(humanoid_interfaces__msg__Motionend * msg);

/// Finalize msg/Motionend message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_humanoid_interfaces
void
humanoid_interfaces__msg__Motionend__fini(humanoid_interfaces__msg__Motionend * msg);

/// Create msg/Motionend message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * humanoid_interfaces__msg__Motionend__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_humanoid_interfaces
humanoid_interfaces__msg__Motionend *
humanoid_interfaces__msg__Motionend__create();

/// Destroy msg/Motionend message.
/**
 * It calls
 * humanoid_interfaces__msg__Motionend__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_humanoid_interfaces
void
humanoid_interfaces__msg__Motionend__destroy(humanoid_interfaces__msg__Motionend * msg);

/// Check for msg/Motionend message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_humanoid_interfaces
bool
humanoid_interfaces__msg__Motionend__are_equal(const humanoid_interfaces__msg__Motionend * lhs, const humanoid_interfaces__msg__Motionend * rhs);

/// Copy a msg/Motionend message.
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
ROSIDL_GENERATOR_C_PUBLIC_humanoid_interfaces
bool
humanoid_interfaces__msg__Motionend__copy(
  const humanoid_interfaces__msg__Motionend * input,
  humanoid_interfaces__msg__Motionend * output);

/// Initialize array of msg/Motionend messages.
/**
 * It allocates the memory for the number of elements and calls
 * humanoid_interfaces__msg__Motionend__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_humanoid_interfaces
bool
humanoid_interfaces__msg__Motionend__Sequence__init(humanoid_interfaces__msg__Motionend__Sequence * array, size_t size);

/// Finalize array of msg/Motionend messages.
/**
 * It calls
 * humanoid_interfaces__msg__Motionend__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_humanoid_interfaces
void
humanoid_interfaces__msg__Motionend__Sequence__fini(humanoid_interfaces__msg__Motionend__Sequence * array);

/// Create array of msg/Motionend messages.
/**
 * It allocates the memory for the array and calls
 * humanoid_interfaces__msg__Motionend__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_humanoid_interfaces
humanoid_interfaces__msg__Motionend__Sequence *
humanoid_interfaces__msg__Motionend__Sequence__create(size_t size);

/// Destroy array of msg/Motionend messages.
/**
 * It calls
 * humanoid_interfaces__msg__Motionend__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_humanoid_interfaces
void
humanoid_interfaces__msg__Motionend__Sequence__destroy(humanoid_interfaces__msg__Motionend__Sequence * array);

/// Check for msg/Motionend message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_humanoid_interfaces
bool
humanoid_interfaces__msg__Motionend__Sequence__are_equal(const humanoid_interfaces__msg__Motionend__Sequence * lhs, const humanoid_interfaces__msg__Motionend__Sequence * rhs);

/// Copy an array of msg/Motionend messages.
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
ROSIDL_GENERATOR_C_PUBLIC_humanoid_interfaces
bool
humanoid_interfaces__msg__Motionend__Sequence__copy(
  const humanoid_interfaces__msg__Motionend__Sequence * input,
  humanoid_interfaces__msg__Motionend__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // HUMANOID_INTERFACES__MSG__DETAIL__MOTIONEND__FUNCTIONS_H_
