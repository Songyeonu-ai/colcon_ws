// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from humanoid_interfaces:msg/IkLTCMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/ik_ltc_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
humanoid_interfaces__msg__IkLTCMsg__init(humanoid_interfaces__msg__IkLTCMsg * msg)
{
  if (!msg) {
    return false;
  }
  // entire_time
  // swing_gain_l
  // swing_gain_r
  // warning
  // safe
  // landing_time_l
  // landing_time_r
  // landing_error_l
  // landing_error_r
  return true;
}

void
humanoid_interfaces__msg__IkLTCMsg__fini(humanoid_interfaces__msg__IkLTCMsg * msg)
{
  if (!msg) {
    return;
  }
  // entire_time
  // swing_gain_l
  // swing_gain_r
  // warning
  // safe
  // landing_time_l
  // landing_time_r
  // landing_error_l
  // landing_error_r
}

bool
humanoid_interfaces__msg__IkLTCMsg__are_equal(const humanoid_interfaces__msg__IkLTCMsg * lhs, const humanoid_interfaces__msg__IkLTCMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // entire_time
  if (lhs->entire_time != rhs->entire_time) {
    return false;
  }
  // swing_gain_l
  if (lhs->swing_gain_l != rhs->swing_gain_l) {
    return false;
  }
  // swing_gain_r
  if (lhs->swing_gain_r != rhs->swing_gain_r) {
    return false;
  }
  // warning
  if (lhs->warning != rhs->warning) {
    return false;
  }
  // safe
  if (lhs->safe != rhs->safe) {
    return false;
  }
  // landing_time_l
  if (lhs->landing_time_l != rhs->landing_time_l) {
    return false;
  }
  // landing_time_r
  if (lhs->landing_time_r != rhs->landing_time_r) {
    return false;
  }
  // landing_error_l
  if (lhs->landing_error_l != rhs->landing_error_l) {
    return false;
  }
  // landing_error_r
  if (lhs->landing_error_r != rhs->landing_error_r) {
    return false;
  }
  return true;
}

bool
humanoid_interfaces__msg__IkLTCMsg__copy(
  const humanoid_interfaces__msg__IkLTCMsg * input,
  humanoid_interfaces__msg__IkLTCMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // entire_time
  output->entire_time = input->entire_time;
  // swing_gain_l
  output->swing_gain_l = input->swing_gain_l;
  // swing_gain_r
  output->swing_gain_r = input->swing_gain_r;
  // warning
  output->warning = input->warning;
  // safe
  output->safe = input->safe;
  // landing_time_l
  output->landing_time_l = input->landing_time_l;
  // landing_time_r
  output->landing_time_r = input->landing_time_r;
  // landing_error_l
  output->landing_error_l = input->landing_error_l;
  // landing_error_r
  output->landing_error_r = input->landing_error_r;
  return true;
}

humanoid_interfaces__msg__IkLTCMsg *
humanoid_interfaces__msg__IkLTCMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__IkLTCMsg * msg = (humanoid_interfaces__msg__IkLTCMsg *)allocator.allocate(sizeof(humanoid_interfaces__msg__IkLTCMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(humanoid_interfaces__msg__IkLTCMsg));
  bool success = humanoid_interfaces__msg__IkLTCMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
humanoid_interfaces__msg__IkLTCMsg__destroy(humanoid_interfaces__msg__IkLTCMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    humanoid_interfaces__msg__IkLTCMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
humanoid_interfaces__msg__IkLTCMsg__Sequence__init(humanoid_interfaces__msg__IkLTCMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__IkLTCMsg * data = NULL;

  if (size) {
    data = (humanoid_interfaces__msg__IkLTCMsg *)allocator.zero_allocate(size, sizeof(humanoid_interfaces__msg__IkLTCMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = humanoid_interfaces__msg__IkLTCMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        humanoid_interfaces__msg__IkLTCMsg__fini(&data[i - 1]);
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
humanoid_interfaces__msg__IkLTCMsg__Sequence__fini(humanoid_interfaces__msg__IkLTCMsg__Sequence * array)
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
      humanoid_interfaces__msg__IkLTCMsg__fini(&array->data[i]);
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

humanoid_interfaces__msg__IkLTCMsg__Sequence *
humanoid_interfaces__msg__IkLTCMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__IkLTCMsg__Sequence * array = (humanoid_interfaces__msg__IkLTCMsg__Sequence *)allocator.allocate(sizeof(humanoid_interfaces__msg__IkLTCMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = humanoid_interfaces__msg__IkLTCMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
humanoid_interfaces__msg__IkLTCMsg__Sequence__destroy(humanoid_interfaces__msg__IkLTCMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    humanoid_interfaces__msg__IkLTCMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
humanoid_interfaces__msg__IkLTCMsg__Sequence__are_equal(const humanoid_interfaces__msg__IkLTCMsg__Sequence * lhs, const humanoid_interfaces__msg__IkLTCMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!humanoid_interfaces__msg__IkLTCMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
humanoid_interfaces__msg__IkLTCMsg__Sequence__copy(
  const humanoid_interfaces__msg__IkLTCMsg__Sequence * input,
  humanoid_interfaces__msg__IkLTCMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(humanoid_interfaces__msg__IkLTCMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    humanoid_interfaces__msg__IkLTCMsg * data =
      (humanoid_interfaces__msg__IkLTCMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!humanoid_interfaces__msg__IkLTCMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          humanoid_interfaces__msg__IkLTCMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!humanoid_interfaces__msg__IkLTCMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
