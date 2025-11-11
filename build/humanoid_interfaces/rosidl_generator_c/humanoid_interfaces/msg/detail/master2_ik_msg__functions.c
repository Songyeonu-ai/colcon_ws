// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from humanoid_interfaces:msg/Master2IkMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/master2_ik_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
humanoid_interfaces__msg__Master2IkMsg__init(humanoid_interfaces__msg__Master2IkMsg * msg)
{
  if (!msg) {
    return false;
  }
  // x_length
  // y_length
  // yaw
  // flag
  // one_x_length
  // one_y_length
  // one_yaw
  // one_step_flag
  return true;
}

void
humanoid_interfaces__msg__Master2IkMsg__fini(humanoid_interfaces__msg__Master2IkMsg * msg)
{
  if (!msg) {
    return;
  }
  // x_length
  // y_length
  // yaw
  // flag
  // one_x_length
  // one_y_length
  // one_yaw
  // one_step_flag
}

bool
humanoid_interfaces__msg__Master2IkMsg__are_equal(const humanoid_interfaces__msg__Master2IkMsg * lhs, const humanoid_interfaces__msg__Master2IkMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x_length
  if (lhs->x_length != rhs->x_length) {
    return false;
  }
  // y_length
  if (lhs->y_length != rhs->y_length) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // flag
  if (lhs->flag != rhs->flag) {
    return false;
  }
  // one_x_length
  if (lhs->one_x_length != rhs->one_x_length) {
    return false;
  }
  // one_y_length
  if (lhs->one_y_length != rhs->one_y_length) {
    return false;
  }
  // one_yaw
  if (lhs->one_yaw != rhs->one_yaw) {
    return false;
  }
  // one_step_flag
  if (lhs->one_step_flag != rhs->one_step_flag) {
    return false;
  }
  return true;
}

bool
humanoid_interfaces__msg__Master2IkMsg__copy(
  const humanoid_interfaces__msg__Master2IkMsg * input,
  humanoid_interfaces__msg__Master2IkMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // x_length
  output->x_length = input->x_length;
  // y_length
  output->y_length = input->y_length;
  // yaw
  output->yaw = input->yaw;
  // flag
  output->flag = input->flag;
  // one_x_length
  output->one_x_length = input->one_x_length;
  // one_y_length
  output->one_y_length = input->one_y_length;
  // one_yaw
  output->one_yaw = input->one_yaw;
  // one_step_flag
  output->one_step_flag = input->one_step_flag;
  return true;
}

humanoid_interfaces__msg__Master2IkMsg *
humanoid_interfaces__msg__Master2IkMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Master2IkMsg * msg = (humanoid_interfaces__msg__Master2IkMsg *)allocator.allocate(sizeof(humanoid_interfaces__msg__Master2IkMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(humanoid_interfaces__msg__Master2IkMsg));
  bool success = humanoid_interfaces__msg__Master2IkMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
humanoid_interfaces__msg__Master2IkMsg__destroy(humanoid_interfaces__msg__Master2IkMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    humanoid_interfaces__msg__Master2IkMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
humanoid_interfaces__msg__Master2IkMsg__Sequence__init(humanoid_interfaces__msg__Master2IkMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Master2IkMsg * data = NULL;

  if (size) {
    data = (humanoid_interfaces__msg__Master2IkMsg *)allocator.zero_allocate(size, sizeof(humanoid_interfaces__msg__Master2IkMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = humanoid_interfaces__msg__Master2IkMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        humanoid_interfaces__msg__Master2IkMsg__fini(&data[i - 1]);
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
humanoid_interfaces__msg__Master2IkMsg__Sequence__fini(humanoid_interfaces__msg__Master2IkMsg__Sequence * array)
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
      humanoid_interfaces__msg__Master2IkMsg__fini(&array->data[i]);
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

humanoid_interfaces__msg__Master2IkMsg__Sequence *
humanoid_interfaces__msg__Master2IkMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Master2IkMsg__Sequence * array = (humanoid_interfaces__msg__Master2IkMsg__Sequence *)allocator.allocate(sizeof(humanoid_interfaces__msg__Master2IkMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = humanoid_interfaces__msg__Master2IkMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
humanoid_interfaces__msg__Master2IkMsg__Sequence__destroy(humanoid_interfaces__msg__Master2IkMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    humanoid_interfaces__msg__Master2IkMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
humanoid_interfaces__msg__Master2IkMsg__Sequence__are_equal(const humanoid_interfaces__msg__Master2IkMsg__Sequence * lhs, const humanoid_interfaces__msg__Master2IkMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!humanoid_interfaces__msg__Master2IkMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
humanoid_interfaces__msg__Master2IkMsg__Sequence__copy(
  const humanoid_interfaces__msg__Master2IkMsg__Sequence * input,
  humanoid_interfaces__msg__Master2IkMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(humanoid_interfaces__msg__Master2IkMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    humanoid_interfaces__msg__Master2IkMsg * data =
      (humanoid_interfaces__msg__Master2IkMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!humanoid_interfaces__msg__Master2IkMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          humanoid_interfaces__msg__Master2IkMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!humanoid_interfaces__msg__Master2IkMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
