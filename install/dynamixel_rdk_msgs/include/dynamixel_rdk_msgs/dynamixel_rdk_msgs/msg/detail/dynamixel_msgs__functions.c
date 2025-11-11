// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dynamixel_rdk_msgs:msg/DynamixelMsgs.idl
// generated code does not contain a copyright notice
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_msgs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
dynamixel_rdk_msgs__msg__DynamixelMsgs__init(dynamixel_rdk_msgs__msg__DynamixelMsgs * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    dynamixel_rdk_msgs__msg__DynamixelMsgs__fini(msg);
    return false;
  }
  // goal_position
  // profile_acceleration
  // profile_velocity
  return true;
}

void
dynamixel_rdk_msgs__msg__DynamixelMsgs__fini(dynamixel_rdk_msgs__msg__DynamixelMsgs * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // goal_position
  // profile_acceleration
  // profile_velocity
}

bool
dynamixel_rdk_msgs__msg__DynamixelMsgs__are_equal(const dynamixel_rdk_msgs__msg__DynamixelMsgs * lhs, const dynamixel_rdk_msgs__msg__DynamixelMsgs * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // goal_position
  if (lhs->goal_position != rhs->goal_position) {
    return false;
  }
  // profile_acceleration
  if (lhs->profile_acceleration != rhs->profile_acceleration) {
    return false;
  }
  // profile_velocity
  if (lhs->profile_velocity != rhs->profile_velocity) {
    return false;
  }
  return true;
}

bool
dynamixel_rdk_msgs__msg__DynamixelMsgs__copy(
  const dynamixel_rdk_msgs__msg__DynamixelMsgs * input,
  dynamixel_rdk_msgs__msg__DynamixelMsgs * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // goal_position
  output->goal_position = input->goal_position;
  // profile_acceleration
  output->profile_acceleration = input->profile_acceleration;
  // profile_velocity
  output->profile_velocity = input->profile_velocity;
  return true;
}

dynamixel_rdk_msgs__msg__DynamixelMsgs *
dynamixel_rdk_msgs__msg__DynamixelMsgs__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_rdk_msgs__msg__DynamixelMsgs * msg = (dynamixel_rdk_msgs__msg__DynamixelMsgs *)allocator.allocate(sizeof(dynamixel_rdk_msgs__msg__DynamixelMsgs), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamixel_rdk_msgs__msg__DynamixelMsgs));
  bool success = dynamixel_rdk_msgs__msg__DynamixelMsgs__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dynamixel_rdk_msgs__msg__DynamixelMsgs__destroy(dynamixel_rdk_msgs__msg__DynamixelMsgs * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dynamixel_rdk_msgs__msg__DynamixelMsgs__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__init(dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_rdk_msgs__msg__DynamixelMsgs * data = NULL;

  if (size) {
    data = (dynamixel_rdk_msgs__msg__DynamixelMsgs *)allocator.zero_allocate(size, sizeof(dynamixel_rdk_msgs__msg__DynamixelMsgs), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamixel_rdk_msgs__msg__DynamixelMsgs__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamixel_rdk_msgs__msg__DynamixelMsgs__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__fini(dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      dynamixel_rdk_msgs__msg__DynamixelMsgs__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence *
dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * array = (dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence *)allocator.allocate(sizeof(dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__destroy(dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__are_equal(const dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * lhs, const dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dynamixel_rdk_msgs__msg__DynamixelMsgs__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__copy(
  const dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * input,
  dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dynamixel_rdk_msgs__msg__DynamixelMsgs);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dynamixel_rdk_msgs__msg__DynamixelMsgs * data =
      (dynamixel_rdk_msgs__msg__DynamixelMsgs *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dynamixel_rdk_msgs__msg__DynamixelMsgs__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dynamixel_rdk_msgs__msg__DynamixelMsgs__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dynamixel_rdk_msgs__msg__DynamixelMsgs__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
