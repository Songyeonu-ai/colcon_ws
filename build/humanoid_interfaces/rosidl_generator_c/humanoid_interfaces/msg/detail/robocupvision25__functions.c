// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from humanoid_interfaces:msg/Robocupvision25.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/robocupvision25__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `robot_vec_x`
// Member `robot_vec_y`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
humanoid_interfaces__msg__Robocupvision25__init(humanoid_interfaces__msg__Robocupvision25 * msg)
{
  if (!msg) {
    return false;
  }
  // ball_cam_x
  // ball_cam_y
  // ball_2d_x
  // ball_2d_y
  // ball_d
  // pan
  // tilt
  // ball_speed_x
  // ball_speed_y
  // robot_vec_x
  if (!rosidl_runtime_c__double__Sequence__init(&msg->robot_vec_x, 0)) {
    humanoid_interfaces__msg__Robocupvision25__fini(msg);
    return false;
  }
  // robot_vec_y
  if (!rosidl_runtime_c__double__Sequence__init(&msg->robot_vec_y, 0)) {
    humanoid_interfaces__msg__Robocupvision25__fini(msg);
    return false;
  }
  // ball_speed_level
  // scan_mode
  return true;
}

void
humanoid_interfaces__msg__Robocupvision25__fini(humanoid_interfaces__msg__Robocupvision25 * msg)
{
  if (!msg) {
    return;
  }
  // ball_cam_x
  // ball_cam_y
  // ball_2d_x
  // ball_2d_y
  // ball_d
  // pan
  // tilt
  // ball_speed_x
  // ball_speed_y
  // robot_vec_x
  rosidl_runtime_c__double__Sequence__fini(&msg->robot_vec_x);
  // robot_vec_y
  rosidl_runtime_c__double__Sequence__fini(&msg->robot_vec_y);
  // ball_speed_level
  // scan_mode
}

bool
humanoid_interfaces__msg__Robocupvision25__are_equal(const humanoid_interfaces__msg__Robocupvision25 * lhs, const humanoid_interfaces__msg__Robocupvision25 * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ball_cam_x
  if (lhs->ball_cam_x != rhs->ball_cam_x) {
    return false;
  }
  // ball_cam_y
  if (lhs->ball_cam_y != rhs->ball_cam_y) {
    return false;
  }
  // ball_2d_x
  if (lhs->ball_2d_x != rhs->ball_2d_x) {
    return false;
  }
  // ball_2d_y
  if (lhs->ball_2d_y != rhs->ball_2d_y) {
    return false;
  }
  // ball_d
  if (lhs->ball_d != rhs->ball_d) {
    return false;
  }
  // pan
  if (lhs->pan != rhs->pan) {
    return false;
  }
  // tilt
  if (lhs->tilt != rhs->tilt) {
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
  // robot_vec_x
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->robot_vec_x), &(rhs->robot_vec_x)))
  {
    return false;
  }
  // robot_vec_y
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->robot_vec_y), &(rhs->robot_vec_y)))
  {
    return false;
  }
  // ball_speed_level
  if (lhs->ball_speed_level != rhs->ball_speed_level) {
    return false;
  }
  // scan_mode
  if (lhs->scan_mode != rhs->scan_mode) {
    return false;
  }
  return true;
}

bool
humanoid_interfaces__msg__Robocupvision25__copy(
  const humanoid_interfaces__msg__Robocupvision25 * input,
  humanoid_interfaces__msg__Robocupvision25 * output)
{
  if (!input || !output) {
    return false;
  }
  // ball_cam_x
  output->ball_cam_x = input->ball_cam_x;
  // ball_cam_y
  output->ball_cam_y = input->ball_cam_y;
  // ball_2d_x
  output->ball_2d_x = input->ball_2d_x;
  // ball_2d_y
  output->ball_2d_y = input->ball_2d_y;
  // ball_d
  output->ball_d = input->ball_d;
  // pan
  output->pan = input->pan;
  // tilt
  output->tilt = input->tilt;
  // ball_speed_x
  output->ball_speed_x = input->ball_speed_x;
  // ball_speed_y
  output->ball_speed_y = input->ball_speed_y;
  // robot_vec_x
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->robot_vec_x), &(output->robot_vec_x)))
  {
    return false;
  }
  // robot_vec_y
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->robot_vec_y), &(output->robot_vec_y)))
  {
    return false;
  }
  // ball_speed_level
  output->ball_speed_level = input->ball_speed_level;
  // scan_mode
  output->scan_mode = input->scan_mode;
  return true;
}

humanoid_interfaces__msg__Robocupvision25 *
humanoid_interfaces__msg__Robocupvision25__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Robocupvision25 * msg = (humanoid_interfaces__msg__Robocupvision25 *)allocator.allocate(sizeof(humanoid_interfaces__msg__Robocupvision25), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(humanoid_interfaces__msg__Robocupvision25));
  bool success = humanoid_interfaces__msg__Robocupvision25__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
humanoid_interfaces__msg__Robocupvision25__destroy(humanoid_interfaces__msg__Robocupvision25 * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    humanoid_interfaces__msg__Robocupvision25__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
humanoid_interfaces__msg__Robocupvision25__Sequence__init(humanoid_interfaces__msg__Robocupvision25__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Robocupvision25 * data = NULL;

  if (size) {
    data = (humanoid_interfaces__msg__Robocupvision25 *)allocator.zero_allocate(size, sizeof(humanoid_interfaces__msg__Robocupvision25), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = humanoid_interfaces__msg__Robocupvision25__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        humanoid_interfaces__msg__Robocupvision25__fini(&data[i - 1]);
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
humanoid_interfaces__msg__Robocupvision25__Sequence__fini(humanoid_interfaces__msg__Robocupvision25__Sequence * array)
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
      humanoid_interfaces__msg__Robocupvision25__fini(&array->data[i]);
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

humanoid_interfaces__msg__Robocupvision25__Sequence *
humanoid_interfaces__msg__Robocupvision25__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Robocupvision25__Sequence * array = (humanoid_interfaces__msg__Robocupvision25__Sequence *)allocator.allocate(sizeof(humanoid_interfaces__msg__Robocupvision25__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = humanoid_interfaces__msg__Robocupvision25__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
humanoid_interfaces__msg__Robocupvision25__Sequence__destroy(humanoid_interfaces__msg__Robocupvision25__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    humanoid_interfaces__msg__Robocupvision25__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
humanoid_interfaces__msg__Robocupvision25__Sequence__are_equal(const humanoid_interfaces__msg__Robocupvision25__Sequence * lhs, const humanoid_interfaces__msg__Robocupvision25__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!humanoid_interfaces__msg__Robocupvision25__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
humanoid_interfaces__msg__Robocupvision25__Sequence__copy(
  const humanoid_interfaces__msg__Robocupvision25__Sequence * input,
  humanoid_interfaces__msg__Robocupvision25__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(humanoid_interfaces__msg__Robocupvision25);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    humanoid_interfaces__msg__Robocupvision25 * data =
      (humanoid_interfaces__msg__Robocupvision25 *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!humanoid_interfaces__msg__Robocupvision25__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          humanoid_interfaces__msg__Robocupvision25__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!humanoid_interfaces__msg__Robocupvision25__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
