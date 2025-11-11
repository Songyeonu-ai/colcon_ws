// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from humanoid_interfaces:msg/CPWalk.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/cp_walk__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
humanoid_interfaces__msg__CPWalk__init(humanoid_interfaces__msg__CPWalk * msg)
{
  if (!msg) {
    return false;
  }
  // cpend
  // cpflag
  // goal
  // start
  // goal_dist
  return true;
}

void
humanoid_interfaces__msg__CPWalk__fini(humanoid_interfaces__msg__CPWalk * msg)
{
  if (!msg) {
    return;
  }
  // cpend
  // cpflag
  // goal
  // start
  // goal_dist
}

bool
humanoid_interfaces__msg__CPWalk__are_equal(const humanoid_interfaces__msg__CPWalk * lhs, const humanoid_interfaces__msg__CPWalk * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cpend
  if (lhs->cpend != rhs->cpend) {
    return false;
  }
  // cpflag
  if (lhs->cpflag != rhs->cpflag) {
    return false;
  }
  // goal
  if (lhs->goal != rhs->goal) {
    return false;
  }
  // start
  if (lhs->start != rhs->start) {
    return false;
  }
  // goal_dist
  if (lhs->goal_dist != rhs->goal_dist) {
    return false;
  }
  return true;
}

bool
humanoid_interfaces__msg__CPWalk__copy(
  const humanoid_interfaces__msg__CPWalk * input,
  humanoid_interfaces__msg__CPWalk * output)
{
  if (!input || !output) {
    return false;
  }
  // cpend
  output->cpend = input->cpend;
  // cpflag
  output->cpflag = input->cpflag;
  // goal
  output->goal = input->goal;
  // start
  output->start = input->start;
  // goal_dist
  output->goal_dist = input->goal_dist;
  return true;
}

humanoid_interfaces__msg__CPWalk *
humanoid_interfaces__msg__CPWalk__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__CPWalk * msg = (humanoid_interfaces__msg__CPWalk *)allocator.allocate(sizeof(humanoid_interfaces__msg__CPWalk), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(humanoid_interfaces__msg__CPWalk));
  bool success = humanoid_interfaces__msg__CPWalk__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
humanoid_interfaces__msg__CPWalk__destroy(humanoid_interfaces__msg__CPWalk * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    humanoid_interfaces__msg__CPWalk__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
humanoid_interfaces__msg__CPWalk__Sequence__init(humanoid_interfaces__msg__CPWalk__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__CPWalk * data = NULL;

  if (size) {
    data = (humanoid_interfaces__msg__CPWalk *)allocator.zero_allocate(size, sizeof(humanoid_interfaces__msg__CPWalk), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = humanoid_interfaces__msg__CPWalk__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        humanoid_interfaces__msg__CPWalk__fini(&data[i - 1]);
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
humanoid_interfaces__msg__CPWalk__Sequence__fini(humanoid_interfaces__msg__CPWalk__Sequence * array)
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
      humanoid_interfaces__msg__CPWalk__fini(&array->data[i]);
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

humanoid_interfaces__msg__CPWalk__Sequence *
humanoid_interfaces__msg__CPWalk__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__CPWalk__Sequence * array = (humanoid_interfaces__msg__CPWalk__Sequence *)allocator.allocate(sizeof(humanoid_interfaces__msg__CPWalk__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = humanoid_interfaces__msg__CPWalk__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
humanoid_interfaces__msg__CPWalk__Sequence__destroy(humanoid_interfaces__msg__CPWalk__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    humanoid_interfaces__msg__CPWalk__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
humanoid_interfaces__msg__CPWalk__Sequence__are_equal(const humanoid_interfaces__msg__CPWalk__Sequence * lhs, const humanoid_interfaces__msg__CPWalk__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!humanoid_interfaces__msg__CPWalk__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
humanoid_interfaces__msg__CPWalk__Sequence__copy(
  const humanoid_interfaces__msg__CPWalk__Sequence * input,
  humanoid_interfaces__msg__CPWalk__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(humanoid_interfaces__msg__CPWalk);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    humanoid_interfaces__msg__CPWalk * data =
      (humanoid_interfaces__msg__CPWalk *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!humanoid_interfaces__msg__CPWalk__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          humanoid_interfaces__msg__CPWalk__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!humanoid_interfaces__msg__CPWalk__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
