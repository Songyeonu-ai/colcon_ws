// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from humanoid_interfaces:msg/Robocuplocalization25.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/robocuplocalization25__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
humanoid_interfaces__msg__Robocuplocalization25__init(humanoid_interfaces__msg__Robocuplocalization25 * msg)
{
  if (!msg) {
    return false;
  }
  // ball_x
  // ball_y
  // ball_speed_x
  // ball_speed_y
  // robot_x
  // robot_y
  // obstacle_x
  // obstacle_y
  // obstacle0_x
  // obstacle0_y
  // obstacle1_x
  // obstacle1_y
  // obstacle2_x
  // obstacle2_y
  // obstacle3_x
  // obstacle3_y
  return true;
}

void
humanoid_interfaces__msg__Robocuplocalization25__fini(humanoid_interfaces__msg__Robocuplocalization25 * msg)
{
  if (!msg) {
    return;
  }
  // ball_x
  // ball_y
  // ball_speed_x
  // ball_speed_y
  // robot_x
  // robot_y
  // obstacle_x
  // obstacle_y
  // obstacle0_x
  // obstacle0_y
  // obstacle1_x
  // obstacle1_y
  // obstacle2_x
  // obstacle2_y
  // obstacle3_x
  // obstacle3_y
}

bool
humanoid_interfaces__msg__Robocuplocalization25__are_equal(const humanoid_interfaces__msg__Robocuplocalization25 * lhs, const humanoid_interfaces__msg__Robocuplocalization25 * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ball_x
  if (lhs->ball_x != rhs->ball_x) {
    return false;
  }
  // ball_y
  if (lhs->ball_y != rhs->ball_y) {
    return false;
  }
  // ball_speed_x
  if (lhs->ball_speed_x != rhs->ball_speed_x) {
    return false;
  }
  // ball_speed_y
  if (lhs->ball_speed_y != rhs->ball_speed_y) {
    return false;
  }
  // robot_x
  if (lhs->robot_x != rhs->robot_x) {
    return false;
  }
  // robot_y
  if (lhs->robot_y != rhs->robot_y) {
    return false;
  }
  // obstacle_x
  if (lhs->obstacle_x != rhs->obstacle_x) {
    return false;
  }
  // obstacle_y
  if (lhs->obstacle_y != rhs->obstacle_y) {
    return false;
  }
  // obstacle0_x
  if (lhs->obstacle0_x != rhs->obstacle0_x) {
    return false;
  }
  // obstacle0_y
  if (lhs->obstacle0_y != rhs->obstacle0_y) {
    return false;
  }
  // obstacle1_x
  if (lhs->obstacle1_x != rhs->obstacle1_x) {
    return false;
  }
  // obstacle1_y
  if (lhs->obstacle1_y != rhs->obstacle1_y) {
    return false;
  }
  // obstacle2_x
  if (lhs->obstacle2_x != rhs->obstacle2_x) {
    return false;
  }
  // obstacle2_y
  if (lhs->obstacle2_y != rhs->obstacle2_y) {
    return false;
  }
  // obstacle3_x
  if (lhs->obstacle3_x != rhs->obstacle3_x) {
    return false;
  }
  // obstacle3_y
  if (lhs->obstacle3_y != rhs->obstacle3_y) {
    return false;
  }
  return true;
}

bool
humanoid_interfaces__msg__Robocuplocalization25__copy(
  const humanoid_interfaces__msg__Robocuplocalization25 * input,
  humanoid_interfaces__msg__Robocuplocalization25 * output)
{
  if (!input || !output) {
    return false;
  }
  // ball_x
  output->ball_x = input->ball_x;
  // ball_y
  output->ball_y = input->ball_y;
  // ball_speed_x
  output->ball_speed_x = input->ball_speed_x;
  // ball_speed_y
  output->ball_speed_y = input->ball_speed_y;
  // robot_x
  output->robot_x = input->robot_x;
  // robot_y
  output->robot_y = input->robot_y;
  // obstacle_x
  output->obstacle_x = input->obstacle_x;
  // obstacle_y
  output->obstacle_y = input->obstacle_y;
  // obstacle0_x
  output->obstacle0_x = input->obstacle0_x;
  // obstacle0_y
  output->obstacle0_y = input->obstacle0_y;
  // obstacle1_x
  output->obstacle1_x = input->obstacle1_x;
  // obstacle1_y
  output->obstacle1_y = input->obstacle1_y;
  // obstacle2_x
  output->obstacle2_x = input->obstacle2_x;
  // obstacle2_y
  output->obstacle2_y = input->obstacle2_y;
  // obstacle3_x
  output->obstacle3_x = input->obstacle3_x;
  // obstacle3_y
  output->obstacle3_y = input->obstacle3_y;
  return true;
}

humanoid_interfaces__msg__Robocuplocalization25 *
humanoid_interfaces__msg__Robocuplocalization25__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Robocuplocalization25 * msg = (humanoid_interfaces__msg__Robocuplocalization25 *)allocator.allocate(sizeof(humanoid_interfaces__msg__Robocuplocalization25), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(humanoid_interfaces__msg__Robocuplocalization25));
  bool success = humanoid_interfaces__msg__Robocuplocalization25__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
humanoid_interfaces__msg__Robocuplocalization25__destroy(humanoid_interfaces__msg__Robocuplocalization25 * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    humanoid_interfaces__msg__Robocuplocalization25__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
humanoid_interfaces__msg__Robocuplocalization25__Sequence__init(humanoid_interfaces__msg__Robocuplocalization25__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Robocuplocalization25 * data = NULL;

  if (size) {
    data = (humanoid_interfaces__msg__Robocuplocalization25 *)allocator.zero_allocate(size, sizeof(humanoid_interfaces__msg__Robocuplocalization25), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = humanoid_interfaces__msg__Robocuplocalization25__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        humanoid_interfaces__msg__Robocuplocalization25__fini(&data[i - 1]);
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
humanoid_interfaces__msg__Robocuplocalization25__Sequence__fini(humanoid_interfaces__msg__Robocuplocalization25__Sequence * array)
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
      humanoid_interfaces__msg__Robocuplocalization25__fini(&array->data[i]);
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

humanoid_interfaces__msg__Robocuplocalization25__Sequence *
humanoid_interfaces__msg__Robocuplocalization25__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Robocuplocalization25__Sequence * array = (humanoid_interfaces__msg__Robocuplocalization25__Sequence *)allocator.allocate(sizeof(humanoid_interfaces__msg__Robocuplocalization25__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = humanoid_interfaces__msg__Robocuplocalization25__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
humanoid_interfaces__msg__Robocuplocalization25__Sequence__destroy(humanoid_interfaces__msg__Robocuplocalization25__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    humanoid_interfaces__msg__Robocuplocalization25__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
humanoid_interfaces__msg__Robocuplocalization25__Sequence__are_equal(const humanoid_interfaces__msg__Robocuplocalization25__Sequence * lhs, const humanoid_interfaces__msg__Robocuplocalization25__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!humanoid_interfaces__msg__Robocuplocalization25__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
humanoid_interfaces__msg__Robocuplocalization25__Sequence__copy(
  const humanoid_interfaces__msg__Robocuplocalization25__Sequence * input,
  humanoid_interfaces__msg__Robocuplocalization25__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(humanoid_interfaces__msg__Robocuplocalization25);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    humanoid_interfaces__msg__Robocuplocalization25 * data =
      (humanoid_interfaces__msg__Robocuplocalization25 *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!humanoid_interfaces__msg__Robocuplocalization25__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          humanoid_interfaces__msg__Robocuplocalization25__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!humanoid_interfaces__msg__Robocuplocalization25__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
