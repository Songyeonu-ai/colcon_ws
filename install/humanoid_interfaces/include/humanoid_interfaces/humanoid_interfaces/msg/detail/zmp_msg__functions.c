// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from humanoid_interfaces:msg/ZmpMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/zmp_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
humanoid_interfaces__msg__ZmpMsg__init(humanoid_interfaces__msg__ZmpMsg * msg)
{
  if (!msg) {
    return false;
  }
  // left_x_zmp
  // left_y_zmp
  // right_x_zmp
  // right_y_zmp
  // total_x_zmp
  // total_y_zmp
  // left_foot
  // right_foot
  // both_feet
  return true;
}

void
humanoid_interfaces__msg__ZmpMsg__fini(humanoid_interfaces__msg__ZmpMsg * msg)
{
  if (!msg) {
    return;
  }
  // left_x_zmp
  // left_y_zmp
  // right_x_zmp
  // right_y_zmp
  // total_x_zmp
  // total_y_zmp
  // left_foot
  // right_foot
  // both_feet
}

bool
humanoid_interfaces__msg__ZmpMsg__are_equal(const humanoid_interfaces__msg__ZmpMsg * lhs, const humanoid_interfaces__msg__ZmpMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // left_x_zmp
  if (lhs->left_x_zmp != rhs->left_x_zmp) {
    return false;
  }
  // left_y_zmp
  if (lhs->left_y_zmp != rhs->left_y_zmp) {
    return false;
  }
  // right_x_zmp
  if (lhs->right_x_zmp != rhs->right_x_zmp) {
    return false;
  }
  // right_y_zmp
  if (lhs->right_y_zmp != rhs->right_y_zmp) {
    return false;
  }
  // total_x_zmp
  if (lhs->total_x_zmp != rhs->total_x_zmp) {
    return false;
  }
  // total_y_zmp
  if (lhs->total_y_zmp != rhs->total_y_zmp) {
    return false;
  }
  // left_foot
  if (lhs->left_foot != rhs->left_foot) {
    return false;
  }
  // right_foot
  if (lhs->right_foot != rhs->right_foot) {
    return false;
  }
  // both_feet
  if (lhs->both_feet != rhs->both_feet) {
    return false;
  }
  return true;
}

bool
humanoid_interfaces__msg__ZmpMsg__copy(
  const humanoid_interfaces__msg__ZmpMsg * input,
  humanoid_interfaces__msg__ZmpMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // left_x_zmp
  output->left_x_zmp = input->left_x_zmp;
  // left_y_zmp
  output->left_y_zmp = input->left_y_zmp;
  // right_x_zmp
  output->right_x_zmp = input->right_x_zmp;
  // right_y_zmp
  output->right_y_zmp = input->right_y_zmp;
  // total_x_zmp
  output->total_x_zmp = input->total_x_zmp;
  // total_y_zmp
  output->total_y_zmp = input->total_y_zmp;
  // left_foot
  output->left_foot = input->left_foot;
  // right_foot
  output->right_foot = input->right_foot;
  // both_feet
  output->both_feet = input->both_feet;
  return true;
}

humanoid_interfaces__msg__ZmpMsg *
humanoid_interfaces__msg__ZmpMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__ZmpMsg * msg = (humanoid_interfaces__msg__ZmpMsg *)allocator.allocate(sizeof(humanoid_interfaces__msg__ZmpMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(humanoid_interfaces__msg__ZmpMsg));
  bool success = humanoid_interfaces__msg__ZmpMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
humanoid_interfaces__msg__ZmpMsg__destroy(humanoid_interfaces__msg__ZmpMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    humanoid_interfaces__msg__ZmpMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
humanoid_interfaces__msg__ZmpMsg__Sequence__init(humanoid_interfaces__msg__ZmpMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__ZmpMsg * data = NULL;

  if (size) {
    data = (humanoid_interfaces__msg__ZmpMsg *)allocator.zero_allocate(size, sizeof(humanoid_interfaces__msg__ZmpMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = humanoid_interfaces__msg__ZmpMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        humanoid_interfaces__msg__ZmpMsg__fini(&data[i - 1]);
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
humanoid_interfaces__msg__ZmpMsg__Sequence__fini(humanoid_interfaces__msg__ZmpMsg__Sequence * array)
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
      humanoid_interfaces__msg__ZmpMsg__fini(&array->data[i]);
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

humanoid_interfaces__msg__ZmpMsg__Sequence *
humanoid_interfaces__msg__ZmpMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__ZmpMsg__Sequence * array = (humanoid_interfaces__msg__ZmpMsg__Sequence *)allocator.allocate(sizeof(humanoid_interfaces__msg__ZmpMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = humanoid_interfaces__msg__ZmpMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
humanoid_interfaces__msg__ZmpMsg__Sequence__destroy(humanoid_interfaces__msg__ZmpMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    humanoid_interfaces__msg__ZmpMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
humanoid_interfaces__msg__ZmpMsg__Sequence__are_equal(const humanoid_interfaces__msg__ZmpMsg__Sequence * lhs, const humanoid_interfaces__msg__ZmpMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!humanoid_interfaces__msg__ZmpMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
humanoid_interfaces__msg__ZmpMsg__Sequence__copy(
  const humanoid_interfaces__msg__ZmpMsg__Sequence * input,
  humanoid_interfaces__msg__ZmpMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(humanoid_interfaces__msg__ZmpMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    humanoid_interfaces__msg__ZmpMsg * data =
      (humanoid_interfaces__msg__ZmpMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!humanoid_interfaces__msg__ZmpMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          humanoid_interfaces__msg__ZmpMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!humanoid_interfaces__msg__ZmpMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
