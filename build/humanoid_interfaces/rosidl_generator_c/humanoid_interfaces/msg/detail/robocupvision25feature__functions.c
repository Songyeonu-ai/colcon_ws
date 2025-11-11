// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from humanoid_interfaces:msg/Robocupvision25feature.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/robocupvision25feature__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `confidence`
// Member `distance`
// Member `point_vec_x`
// Member `point_vec_y`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
humanoid_interfaces__msg__Robocupvision25feature__init(humanoid_interfaces__msg__Robocupvision25feature * msg)
{
  if (!msg) {
    return false;
  }
  // confidence
  if (!rosidl_runtime_c__double__Sequence__init(&msg->confidence, 0)) {
    humanoid_interfaces__msg__Robocupvision25feature__fini(msg);
    return false;
  }
  // distance
  if (!rosidl_runtime_c__double__Sequence__init(&msg->distance, 0)) {
    humanoid_interfaces__msg__Robocupvision25feature__fini(msg);
    return false;
  }
  // point_vec_x
  if (!rosidl_runtime_c__double__Sequence__init(&msg->point_vec_x, 0)) {
    humanoid_interfaces__msg__Robocupvision25feature__fini(msg);
    return false;
  }
  // point_vec_y
  if (!rosidl_runtime_c__double__Sequence__init(&msg->point_vec_y, 0)) {
    humanoid_interfaces__msg__Robocupvision25feature__fini(msg);
    return false;
  }
  return true;
}

void
humanoid_interfaces__msg__Robocupvision25feature__fini(humanoid_interfaces__msg__Robocupvision25feature * msg)
{
  if (!msg) {
    return;
  }
  // confidence
  rosidl_runtime_c__double__Sequence__fini(&msg->confidence);
  // distance
  rosidl_runtime_c__double__Sequence__fini(&msg->distance);
  // point_vec_x
  rosidl_runtime_c__double__Sequence__fini(&msg->point_vec_x);
  // point_vec_y
  rosidl_runtime_c__double__Sequence__fini(&msg->point_vec_y);
}

bool
humanoid_interfaces__msg__Robocupvision25feature__are_equal(const humanoid_interfaces__msg__Robocupvision25feature * lhs, const humanoid_interfaces__msg__Robocupvision25feature * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // confidence
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->confidence), &(rhs->confidence)))
  {
    return false;
  }
  // distance
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->distance), &(rhs->distance)))
  {
    return false;
  }
  // point_vec_x
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->point_vec_x), &(rhs->point_vec_x)))
  {
    return false;
  }
  // point_vec_y
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->point_vec_y), &(rhs->point_vec_y)))
  {
    return false;
  }
  return true;
}

bool
humanoid_interfaces__msg__Robocupvision25feature__copy(
  const humanoid_interfaces__msg__Robocupvision25feature * input,
  humanoid_interfaces__msg__Robocupvision25feature * output)
{
  if (!input || !output) {
    return false;
  }
  // confidence
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->confidence), &(output->confidence)))
  {
    return false;
  }
  // distance
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->distance), &(output->distance)))
  {
    return false;
  }
  // point_vec_x
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->point_vec_x), &(output->point_vec_x)))
  {
    return false;
  }
  // point_vec_y
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->point_vec_y), &(output->point_vec_y)))
  {
    return false;
  }
  return true;
}

humanoid_interfaces__msg__Robocupvision25feature *
humanoid_interfaces__msg__Robocupvision25feature__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Robocupvision25feature * msg = (humanoid_interfaces__msg__Robocupvision25feature *)allocator.allocate(sizeof(humanoid_interfaces__msg__Robocupvision25feature), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(humanoid_interfaces__msg__Robocupvision25feature));
  bool success = humanoid_interfaces__msg__Robocupvision25feature__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
humanoid_interfaces__msg__Robocupvision25feature__destroy(humanoid_interfaces__msg__Robocupvision25feature * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    humanoid_interfaces__msg__Robocupvision25feature__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
humanoid_interfaces__msg__Robocupvision25feature__Sequence__init(humanoid_interfaces__msg__Robocupvision25feature__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Robocupvision25feature * data = NULL;

  if (size) {
    data = (humanoid_interfaces__msg__Robocupvision25feature *)allocator.zero_allocate(size, sizeof(humanoid_interfaces__msg__Robocupvision25feature), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = humanoid_interfaces__msg__Robocupvision25feature__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        humanoid_interfaces__msg__Robocupvision25feature__fini(&data[i - 1]);
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
humanoid_interfaces__msg__Robocupvision25feature__Sequence__fini(humanoid_interfaces__msg__Robocupvision25feature__Sequence * array)
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
      humanoid_interfaces__msg__Robocupvision25feature__fini(&array->data[i]);
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

humanoid_interfaces__msg__Robocupvision25feature__Sequence *
humanoid_interfaces__msg__Robocupvision25feature__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Robocupvision25feature__Sequence * array = (humanoid_interfaces__msg__Robocupvision25feature__Sequence *)allocator.allocate(sizeof(humanoid_interfaces__msg__Robocupvision25feature__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = humanoid_interfaces__msg__Robocupvision25feature__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
humanoid_interfaces__msg__Robocupvision25feature__Sequence__destroy(humanoid_interfaces__msg__Robocupvision25feature__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    humanoid_interfaces__msg__Robocupvision25feature__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
humanoid_interfaces__msg__Robocupvision25feature__Sequence__are_equal(const humanoid_interfaces__msg__Robocupvision25feature__Sequence * lhs, const humanoid_interfaces__msg__Robocupvision25feature__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!humanoid_interfaces__msg__Robocupvision25feature__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
humanoid_interfaces__msg__Robocupvision25feature__Sequence__copy(
  const humanoid_interfaces__msg__Robocupvision25feature__Sequence * input,
  humanoid_interfaces__msg__Robocupvision25feature__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(humanoid_interfaces__msg__Robocupvision25feature);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    humanoid_interfaces__msg__Robocupvision25feature * data =
      (humanoid_interfaces__msg__Robocupvision25feature *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!humanoid_interfaces__msg__Robocupvision25feature__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          humanoid_interfaces__msg__Robocupvision25feature__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!humanoid_interfaces__msg__Robocupvision25feature__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
