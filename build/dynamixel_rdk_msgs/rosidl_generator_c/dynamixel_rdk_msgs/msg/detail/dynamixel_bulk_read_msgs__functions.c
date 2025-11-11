// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dynamixel_rdk_msgs:msg/DynamixelBulkReadMsgs.idl
// generated code does not contain a copyright notice
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_bulk_read_msgs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `status_msgs`
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_status_msgs__functions.h"

bool
dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__init(dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs * msg)
{
  if (!msg) {
    return false;
  }
  // status_msgs
  if (!dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence__init(&msg->status_msgs, 0)) {
    dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__fini(msg);
    return false;
  }
  return true;
}

void
dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__fini(dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs * msg)
{
  if (!msg) {
    return;
  }
  // status_msgs
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence__fini(&msg->status_msgs);
}

bool
dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__are_equal(const dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs * lhs, const dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status_msgs
  if (!dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence__are_equal(
      &(lhs->status_msgs), &(rhs->status_msgs)))
  {
    return false;
  }
  return true;
}

bool
dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__copy(
  const dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs * input,
  dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs * output)
{
  if (!input || !output) {
    return false;
  }
  // status_msgs
  if (!dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence__copy(
      &(input->status_msgs), &(output->status_msgs)))
  {
    return false;
  }
  return true;
}

dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs *
dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs * msg = (dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs *)allocator.allocate(sizeof(dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs));
  bool success = dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__destroy(dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence__init(dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs * data = NULL;

  if (size) {
    data = (dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs *)allocator.zero_allocate(size, sizeof(dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__fini(&data[i - 1]);
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
dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence__fini(dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence * array)
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
      dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__fini(&array->data[i]);
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

dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence *
dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence * array = (dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence *)allocator.allocate(sizeof(dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence__destroy(dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence__are_equal(const dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence * lhs, const dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence__copy(
  const dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence * input,
  dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs * data =
      (dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dynamixel_rdk_msgs__msg__DynamixelBulkReadMsgs__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
