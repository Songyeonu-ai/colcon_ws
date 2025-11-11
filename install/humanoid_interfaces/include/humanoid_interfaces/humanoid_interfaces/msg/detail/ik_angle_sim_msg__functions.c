// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from humanoid_interfaces:msg/IkAngleSimMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/ik_angle_sim_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
humanoid_interfaces__msg__IkAngleSimMsg__init(humanoid_interfaces__msg__IkAngleSimMsg * msg)
{
  if (!msg) {
    return false;
  }
  // ang_l_0
  // ang_l_1
  // ang_l_2
  // ang_l_3
  // ang_l_4
  // ang_l_5
  // ang_l_6
  // ang_l_7
  // ang_l_8
  // ang_r_0
  // ang_r_1
  // ang_r_2
  // ang_r_3
  // ang_r_4
  // ang_r_5
  // ang_r_6
  // ang_r_7
  // ang_r_8
  return true;
}

void
humanoid_interfaces__msg__IkAngleSimMsg__fini(humanoid_interfaces__msg__IkAngleSimMsg * msg)
{
  if (!msg) {
    return;
  }
  // ang_l_0
  // ang_l_1
  // ang_l_2
  // ang_l_3
  // ang_l_4
  // ang_l_5
  // ang_l_6
  // ang_l_7
  // ang_l_8
  // ang_r_0
  // ang_r_1
  // ang_r_2
  // ang_r_3
  // ang_r_4
  // ang_r_5
  // ang_r_6
  // ang_r_7
  // ang_r_8
}

bool
humanoid_interfaces__msg__IkAngleSimMsg__are_equal(const humanoid_interfaces__msg__IkAngleSimMsg * lhs, const humanoid_interfaces__msg__IkAngleSimMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ang_l_0
  if (lhs->ang_l_0 != rhs->ang_l_0) {
    return false;
  }
  // ang_l_1
  if (lhs->ang_l_1 != rhs->ang_l_1) {
    return false;
  }
  // ang_l_2
  if (lhs->ang_l_2 != rhs->ang_l_2) {
    return false;
  }
  // ang_l_3
  if (lhs->ang_l_3 != rhs->ang_l_3) {
    return false;
  }
  // ang_l_4
  if (lhs->ang_l_4 != rhs->ang_l_4) {
    return false;
  }
  // ang_l_5
  if (lhs->ang_l_5 != rhs->ang_l_5) {
    return false;
  }
  // ang_l_6
  if (lhs->ang_l_6 != rhs->ang_l_6) {
    return false;
  }
  // ang_l_7
  if (lhs->ang_l_7 != rhs->ang_l_7) {
    return false;
  }
  // ang_l_8
  if (lhs->ang_l_8 != rhs->ang_l_8) {
    return false;
  }
  // ang_r_0
  if (lhs->ang_r_0 != rhs->ang_r_0) {
    return false;
  }
  // ang_r_1
  if (lhs->ang_r_1 != rhs->ang_r_1) {
    return false;
  }
  // ang_r_2
  if (lhs->ang_r_2 != rhs->ang_r_2) {
    return false;
  }
  // ang_r_3
  if (lhs->ang_r_3 != rhs->ang_r_3) {
    return false;
  }
  // ang_r_4
  if (lhs->ang_r_4 != rhs->ang_r_4) {
    return false;
  }
  // ang_r_5
  if (lhs->ang_r_5 != rhs->ang_r_5) {
    return false;
  }
  // ang_r_6
  if (lhs->ang_r_6 != rhs->ang_r_6) {
    return false;
  }
  // ang_r_7
  if (lhs->ang_r_7 != rhs->ang_r_7) {
    return false;
  }
  // ang_r_8
  if (lhs->ang_r_8 != rhs->ang_r_8) {
    return false;
  }
  return true;
}

bool
humanoid_interfaces__msg__IkAngleSimMsg__copy(
  const humanoid_interfaces__msg__IkAngleSimMsg * input,
  humanoid_interfaces__msg__IkAngleSimMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // ang_l_0
  output->ang_l_0 = input->ang_l_0;
  // ang_l_1
  output->ang_l_1 = input->ang_l_1;
  // ang_l_2
  output->ang_l_2 = input->ang_l_2;
  // ang_l_3
  output->ang_l_3 = input->ang_l_3;
  // ang_l_4
  output->ang_l_4 = input->ang_l_4;
  // ang_l_5
  output->ang_l_5 = input->ang_l_5;
  // ang_l_6
  output->ang_l_6 = input->ang_l_6;
  // ang_l_7
  output->ang_l_7 = input->ang_l_7;
  // ang_l_8
  output->ang_l_8 = input->ang_l_8;
  // ang_r_0
  output->ang_r_0 = input->ang_r_0;
  // ang_r_1
  output->ang_r_1 = input->ang_r_1;
  // ang_r_2
  output->ang_r_2 = input->ang_r_2;
  // ang_r_3
  output->ang_r_3 = input->ang_r_3;
  // ang_r_4
  output->ang_r_4 = input->ang_r_4;
  // ang_r_5
  output->ang_r_5 = input->ang_r_5;
  // ang_r_6
  output->ang_r_6 = input->ang_r_6;
  // ang_r_7
  output->ang_r_7 = input->ang_r_7;
  // ang_r_8
  output->ang_r_8 = input->ang_r_8;
  return true;
}

humanoid_interfaces__msg__IkAngleSimMsg *
humanoid_interfaces__msg__IkAngleSimMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__IkAngleSimMsg * msg = (humanoid_interfaces__msg__IkAngleSimMsg *)allocator.allocate(sizeof(humanoid_interfaces__msg__IkAngleSimMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(humanoid_interfaces__msg__IkAngleSimMsg));
  bool success = humanoid_interfaces__msg__IkAngleSimMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
humanoid_interfaces__msg__IkAngleSimMsg__destroy(humanoid_interfaces__msg__IkAngleSimMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    humanoid_interfaces__msg__IkAngleSimMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
humanoid_interfaces__msg__IkAngleSimMsg__Sequence__init(humanoid_interfaces__msg__IkAngleSimMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__IkAngleSimMsg * data = NULL;

  if (size) {
    data = (humanoid_interfaces__msg__IkAngleSimMsg *)allocator.zero_allocate(size, sizeof(humanoid_interfaces__msg__IkAngleSimMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = humanoid_interfaces__msg__IkAngleSimMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        humanoid_interfaces__msg__IkAngleSimMsg__fini(&data[i - 1]);
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
humanoid_interfaces__msg__IkAngleSimMsg__Sequence__fini(humanoid_interfaces__msg__IkAngleSimMsg__Sequence * array)
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
      humanoid_interfaces__msg__IkAngleSimMsg__fini(&array->data[i]);
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

humanoid_interfaces__msg__IkAngleSimMsg__Sequence *
humanoid_interfaces__msg__IkAngleSimMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__IkAngleSimMsg__Sequence * array = (humanoid_interfaces__msg__IkAngleSimMsg__Sequence *)allocator.allocate(sizeof(humanoid_interfaces__msg__IkAngleSimMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = humanoid_interfaces__msg__IkAngleSimMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
humanoid_interfaces__msg__IkAngleSimMsg__Sequence__destroy(humanoid_interfaces__msg__IkAngleSimMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    humanoid_interfaces__msg__IkAngleSimMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
humanoid_interfaces__msg__IkAngleSimMsg__Sequence__are_equal(const humanoid_interfaces__msg__IkAngleSimMsg__Sequence * lhs, const humanoid_interfaces__msg__IkAngleSimMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!humanoid_interfaces__msg__IkAngleSimMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
humanoid_interfaces__msg__IkAngleSimMsg__Sequence__copy(
  const humanoid_interfaces__msg__IkAngleSimMsg__Sequence * input,
  humanoid_interfaces__msg__IkAngleSimMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(humanoid_interfaces__msg__IkAngleSimMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    humanoid_interfaces__msg__IkAngleSimMsg * data =
      (humanoid_interfaces__msg__IkAngleSimMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!humanoid_interfaces__msg__IkAngleSimMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          humanoid_interfaces__msg__IkAngleSimMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!humanoid_interfaces__msg__IkAngleSimMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
