// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dynamixel_rdk_msgs:msg/DynamixelControlMsgs.idl
// generated code does not contain a copyright notice
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_control_msgs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `motor_control`
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_msgs__functions.h"

bool
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__init(dynamixel_rdk_msgs__msg__DynamixelControlMsgs * msg)
{
  if (!msg) {
    return false;
  }
  // motor_control
  if (!dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__init(&msg->motor_control, 0)) {
    dynamixel_rdk_msgs__msg__DynamixelControlMsgs__fini(msg);
    return false;
  }
  return true;
}

void
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__fini(dynamixel_rdk_msgs__msg__DynamixelControlMsgs * msg)
{
  if (!msg) {
    return;
  }
  // motor_control
  dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__fini(&msg->motor_control);
}

bool
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__are_equal(const dynamixel_rdk_msgs__msg__DynamixelControlMsgs * lhs, const dynamixel_rdk_msgs__msg__DynamixelControlMsgs * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // motor_control
  if (!dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__are_equal(
      &(lhs->motor_control), &(rhs->motor_control)))
  {
    return false;
  }
  return true;
}

bool
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__copy(
  const dynamixel_rdk_msgs__msg__DynamixelControlMsgs * input,
  dynamixel_rdk_msgs__msg__DynamixelControlMsgs * output)
{
  if (!input || !output) {
    return false;
  }
  // motor_control
  if (!dynamixel_rdk_msgs__msg__DynamixelMsgs__Sequence__copy(
      &(input->motor_control), &(output->motor_control)))
  {
    return false;
  }
  return true;
}

dynamixel_rdk_msgs__msg__DynamixelControlMsgs *
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_rdk_msgs__msg__DynamixelControlMsgs * msg = (dynamixel_rdk_msgs__msg__DynamixelControlMsgs *)allocator.allocate(sizeof(dynamixel_rdk_msgs__msg__DynamixelControlMsgs), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamixel_rdk_msgs__msg__DynamixelControlMsgs));
  bool success = dynamixel_rdk_msgs__msg__DynamixelControlMsgs__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__destroy(dynamixel_rdk_msgs__msg__DynamixelControlMsgs * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dynamixel_rdk_msgs__msg__DynamixelControlMsgs__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence__init(dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_rdk_msgs__msg__DynamixelControlMsgs * data = NULL;

  if (size) {
    data = (dynamixel_rdk_msgs__msg__DynamixelControlMsgs *)allocator.zero_allocate(size, sizeof(dynamixel_rdk_msgs__msg__DynamixelControlMsgs), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamixel_rdk_msgs__msg__DynamixelControlMsgs__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamixel_rdk_msgs__msg__DynamixelControlMsgs__fini(&data[i - 1]);
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
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence__fini(dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence * array)
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
      dynamixel_rdk_msgs__msg__DynamixelControlMsgs__fini(&array->data[i]);
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

dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence *
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence * array = (dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence *)allocator.allocate(sizeof(dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence__destroy(dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence__are_equal(const dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence * lhs, const dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dynamixel_rdk_msgs__msg__DynamixelControlMsgs__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence__copy(
  const dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence * input,
  dynamixel_rdk_msgs__msg__DynamixelControlMsgs__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dynamixel_rdk_msgs__msg__DynamixelControlMsgs);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dynamixel_rdk_msgs__msg__DynamixelControlMsgs * data =
      (dynamixel_rdk_msgs__msg__DynamixelControlMsgs *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dynamixel_rdk_msgs__msg__DynamixelControlMsgs__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dynamixel_rdk_msgs__msg__DynamixelControlMsgs__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dynamixel_rdk_msgs__msg__DynamixelControlMsgs__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
