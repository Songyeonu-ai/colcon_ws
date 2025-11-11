// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from humanoid_interfaces:msg/LCMsgs.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/lc_msgs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `r_lc_data`
// Member `l_lc_data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
humanoid_interfaces__msg__LCMsgs__init(humanoid_interfaces__msg__LCMsgs * msg)
{
  if (!msg) {
    return false;
  }
  // r_lc_data
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->r_lc_data, 0)) {
    humanoid_interfaces__msg__LCMsgs__fini(msg);
    return false;
  }
  // l_lc_data
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->l_lc_data, 0)) {
    humanoid_interfaces__msg__LCMsgs__fini(msg);
    return false;
  }
  return true;
}

void
humanoid_interfaces__msg__LCMsgs__fini(humanoid_interfaces__msg__LCMsgs * msg)
{
  if (!msg) {
    return;
  }
  // r_lc_data
  rosidl_runtime_c__int32__Sequence__fini(&msg->r_lc_data);
  // l_lc_data
  rosidl_runtime_c__int32__Sequence__fini(&msg->l_lc_data);
}

bool
humanoid_interfaces__msg__LCMsgs__are_equal(const humanoid_interfaces__msg__LCMsgs * lhs, const humanoid_interfaces__msg__LCMsgs * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // r_lc_data
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->r_lc_data), &(rhs->r_lc_data)))
  {
    return false;
  }
  // l_lc_data
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->l_lc_data), &(rhs->l_lc_data)))
  {
    return false;
  }
  return true;
}

bool
humanoid_interfaces__msg__LCMsgs__copy(
  const humanoid_interfaces__msg__LCMsgs * input,
  humanoid_interfaces__msg__LCMsgs * output)
{
  if (!input || !output) {
    return false;
  }
  // r_lc_data
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->r_lc_data), &(output->r_lc_data)))
  {
    return false;
  }
  // l_lc_data
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->l_lc_data), &(output->l_lc_data)))
  {
    return false;
  }
  return true;
}

humanoid_interfaces__msg__LCMsgs *
humanoid_interfaces__msg__LCMsgs__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__LCMsgs * msg = (humanoid_interfaces__msg__LCMsgs *)allocator.allocate(sizeof(humanoid_interfaces__msg__LCMsgs), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(humanoid_interfaces__msg__LCMsgs));
  bool success = humanoid_interfaces__msg__LCMsgs__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
humanoid_interfaces__msg__LCMsgs__destroy(humanoid_interfaces__msg__LCMsgs * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    humanoid_interfaces__msg__LCMsgs__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
humanoid_interfaces__msg__LCMsgs__Sequence__init(humanoid_interfaces__msg__LCMsgs__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__LCMsgs * data = NULL;

  if (size) {
    data = (humanoid_interfaces__msg__LCMsgs *)allocator.zero_allocate(size, sizeof(humanoid_interfaces__msg__LCMsgs), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = humanoid_interfaces__msg__LCMsgs__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        humanoid_interfaces__msg__LCMsgs__fini(&data[i - 1]);
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
humanoid_interfaces__msg__LCMsgs__Sequence__fini(humanoid_interfaces__msg__LCMsgs__Sequence * array)
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
      humanoid_interfaces__msg__LCMsgs__fini(&array->data[i]);
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

humanoid_interfaces__msg__LCMsgs__Sequence *
humanoid_interfaces__msg__LCMsgs__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__LCMsgs__Sequence * array = (humanoid_interfaces__msg__LCMsgs__Sequence *)allocator.allocate(sizeof(humanoid_interfaces__msg__LCMsgs__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = humanoid_interfaces__msg__LCMsgs__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
humanoid_interfaces__msg__LCMsgs__Sequence__destroy(humanoid_interfaces__msg__LCMsgs__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    humanoid_interfaces__msg__LCMsgs__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
humanoid_interfaces__msg__LCMsgs__Sequence__are_equal(const humanoid_interfaces__msg__LCMsgs__Sequence * lhs, const humanoid_interfaces__msg__LCMsgs__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!humanoid_interfaces__msg__LCMsgs__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
humanoid_interfaces__msg__LCMsgs__Sequence__copy(
  const humanoid_interfaces__msg__LCMsgs__Sequence * input,
  humanoid_interfaces__msg__LCMsgs__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(humanoid_interfaces__msg__LCMsgs);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    humanoid_interfaces__msg__LCMsgs * data =
      (humanoid_interfaces__msg__LCMsgs *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!humanoid_interfaces__msg__LCMsgs__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          humanoid_interfaces__msg__LCMsgs__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!humanoid_interfaces__msg__LCMsgs__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
