// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from humanoid_interfaces:msg/Robocupcontroller.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/robocupcontroller__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `secondinfo`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
humanoid_interfaces__msg__Robocupcontroller__init(humanoid_interfaces__msg__Robocupcontroller * msg)
{
  if (!msg) {
    return false;
  }
  // robotnum
  // state
  // readytime
  // penalty
  // firsthalf
  // firstside
  // kickoffteam
  // secondstate
  // secondinfo
  if (!rosidl_runtime_c__int64__Sequence__init(&msg->secondinfo, 0)) {
    humanoid_interfaces__msg__Robocupcontroller__fini(msg);
    return false;
  }
  return true;
}

void
humanoid_interfaces__msg__Robocupcontroller__fini(humanoid_interfaces__msg__Robocupcontroller * msg)
{
  if (!msg) {
    return;
  }
  // robotnum
  // state
  // readytime
  // penalty
  // firsthalf
  // firstside
  // kickoffteam
  // secondstate
  // secondinfo
  rosidl_runtime_c__int64__Sequence__fini(&msg->secondinfo);
}

bool
humanoid_interfaces__msg__Robocupcontroller__are_equal(const humanoid_interfaces__msg__Robocupcontroller * lhs, const humanoid_interfaces__msg__Robocupcontroller * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robotnum
  if (lhs->robotnum != rhs->robotnum) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  // readytime
  if (lhs->readytime != rhs->readytime) {
    return false;
  }
  // penalty
  if (lhs->penalty != rhs->penalty) {
    return false;
  }
  // firsthalf
  if (lhs->firsthalf != rhs->firsthalf) {
    return false;
  }
  // firstside
  if (lhs->firstside != rhs->firstside) {
    return false;
  }
  // kickoffteam
  if (lhs->kickoffteam != rhs->kickoffteam) {
    return false;
  }
  // secondstate
  if (lhs->secondstate != rhs->secondstate) {
    return false;
  }
  // secondinfo
  if (!rosidl_runtime_c__int64__Sequence__are_equal(
      &(lhs->secondinfo), &(rhs->secondinfo)))
  {
    return false;
  }
  return true;
}

bool
humanoid_interfaces__msg__Robocupcontroller__copy(
  const humanoid_interfaces__msg__Robocupcontroller * input,
  humanoid_interfaces__msg__Robocupcontroller * output)
{
  if (!input || !output) {
    return false;
  }
  // robotnum
  output->robotnum = input->robotnum;
  // state
  output->state = input->state;
  // readytime
  output->readytime = input->readytime;
  // penalty
  output->penalty = input->penalty;
  // firsthalf
  output->firsthalf = input->firsthalf;
  // firstside
  output->firstside = input->firstside;
  // kickoffteam
  output->kickoffteam = input->kickoffteam;
  // secondstate
  output->secondstate = input->secondstate;
  // secondinfo
  if (!rosidl_runtime_c__int64__Sequence__copy(
      &(input->secondinfo), &(output->secondinfo)))
  {
    return false;
  }
  return true;
}

humanoid_interfaces__msg__Robocupcontroller *
humanoid_interfaces__msg__Robocupcontroller__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Robocupcontroller * msg = (humanoid_interfaces__msg__Robocupcontroller *)allocator.allocate(sizeof(humanoid_interfaces__msg__Robocupcontroller), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(humanoid_interfaces__msg__Robocupcontroller));
  bool success = humanoid_interfaces__msg__Robocupcontroller__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
humanoid_interfaces__msg__Robocupcontroller__destroy(humanoid_interfaces__msg__Robocupcontroller * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    humanoid_interfaces__msg__Robocupcontroller__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
humanoid_interfaces__msg__Robocupcontroller__Sequence__init(humanoid_interfaces__msg__Robocupcontroller__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Robocupcontroller * data = NULL;

  if (size) {
    data = (humanoid_interfaces__msg__Robocupcontroller *)allocator.zero_allocate(size, sizeof(humanoid_interfaces__msg__Robocupcontroller), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = humanoid_interfaces__msg__Robocupcontroller__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        humanoid_interfaces__msg__Robocupcontroller__fini(&data[i - 1]);
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
humanoid_interfaces__msg__Robocupcontroller__Sequence__fini(humanoid_interfaces__msg__Robocupcontroller__Sequence * array)
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
      humanoid_interfaces__msg__Robocupcontroller__fini(&array->data[i]);
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

humanoid_interfaces__msg__Robocupcontroller__Sequence *
humanoid_interfaces__msg__Robocupcontroller__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Robocupcontroller__Sequence * array = (humanoid_interfaces__msg__Robocupcontroller__Sequence *)allocator.allocate(sizeof(humanoid_interfaces__msg__Robocupcontroller__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = humanoid_interfaces__msg__Robocupcontroller__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
humanoid_interfaces__msg__Robocupcontroller__Sequence__destroy(humanoid_interfaces__msg__Robocupcontroller__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    humanoid_interfaces__msg__Robocupcontroller__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
humanoid_interfaces__msg__Robocupcontroller__Sequence__are_equal(const humanoid_interfaces__msg__Robocupcontroller__Sequence * lhs, const humanoid_interfaces__msg__Robocupcontroller__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!humanoid_interfaces__msg__Robocupcontroller__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
humanoid_interfaces__msg__Robocupcontroller__Sequence__copy(
  const humanoid_interfaces__msg__Robocupcontroller__Sequence * input,
  humanoid_interfaces__msg__Robocupcontroller__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(humanoid_interfaces__msg__Robocupcontroller);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    humanoid_interfaces__msg__Robocupcontroller * data =
      (humanoid_interfaces__msg__Robocupcontroller *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!humanoid_interfaces__msg__Robocupcontroller__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          humanoid_interfaces__msg__Robocupcontroller__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!humanoid_interfaces__msg__Robocupcontroller__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
