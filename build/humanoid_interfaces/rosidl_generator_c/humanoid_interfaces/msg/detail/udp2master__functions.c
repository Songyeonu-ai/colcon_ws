// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from humanoid_interfaces:msg/Udp2master.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/udp2master__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
humanoid_interfaces__msg__Udp2master__init(humanoid_interfaces__msg__Udp2master * msg)
{
  if (!msg) {
    return false;
  }
  // robotnum
  // robotcase
  // localx
  // localy
  // localyaw
  // ballx
  // bally
  // balldist
  // balltheta
  return true;
}

void
humanoid_interfaces__msg__Udp2master__fini(humanoid_interfaces__msg__Udp2master * msg)
{
  if (!msg) {
    return;
  }
  // robotnum
  // robotcase
  // localx
  // localy
  // localyaw
  // ballx
  // bally
  // balldist
  // balltheta
}

bool
humanoid_interfaces__msg__Udp2master__are_equal(const humanoid_interfaces__msg__Udp2master * lhs, const humanoid_interfaces__msg__Udp2master * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robotnum
  if (lhs->robotnum != rhs->robotnum) {
    return false;
  }
  // robotcase
  if (lhs->robotcase != rhs->robotcase) {
    return false;
  }
  // localx
  if (lhs->localx != rhs->localx) {
    return false;
  }
  // localy
  if (lhs->localy != rhs->localy) {
    return false;
  }
  // localyaw
  if (lhs->localyaw != rhs->localyaw) {
    return false;
  }
  // ballx
  if (lhs->ballx != rhs->ballx) {
    return false;
  }
  // bally
  if (lhs->bally != rhs->bally) {
    return false;
  }
  // balldist
  if (lhs->balldist != rhs->balldist) {
    return false;
  }
  // balltheta
  if (lhs->balltheta != rhs->balltheta) {
    return false;
  }
  return true;
}

bool
humanoid_interfaces__msg__Udp2master__copy(
  const humanoid_interfaces__msg__Udp2master * input,
  humanoid_interfaces__msg__Udp2master * output)
{
  if (!input || !output) {
    return false;
  }
  // robotnum
  output->robotnum = input->robotnum;
  // robotcase
  output->robotcase = input->robotcase;
  // localx
  output->localx = input->localx;
  // localy
  output->localy = input->localy;
  // localyaw
  output->localyaw = input->localyaw;
  // ballx
  output->ballx = input->ballx;
  // bally
  output->bally = input->bally;
  // balldist
  output->balldist = input->balldist;
  // balltheta
  output->balltheta = input->balltheta;
  return true;
}

humanoid_interfaces__msg__Udp2master *
humanoid_interfaces__msg__Udp2master__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Udp2master * msg = (humanoid_interfaces__msg__Udp2master *)allocator.allocate(sizeof(humanoid_interfaces__msg__Udp2master), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(humanoid_interfaces__msg__Udp2master));
  bool success = humanoid_interfaces__msg__Udp2master__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
humanoid_interfaces__msg__Udp2master__destroy(humanoid_interfaces__msg__Udp2master * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    humanoid_interfaces__msg__Udp2master__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
humanoid_interfaces__msg__Udp2master__Sequence__init(humanoid_interfaces__msg__Udp2master__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Udp2master * data = NULL;

  if (size) {
    data = (humanoid_interfaces__msg__Udp2master *)allocator.zero_allocate(size, sizeof(humanoid_interfaces__msg__Udp2master), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = humanoid_interfaces__msg__Udp2master__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        humanoid_interfaces__msg__Udp2master__fini(&data[i - 1]);
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
humanoid_interfaces__msg__Udp2master__Sequence__fini(humanoid_interfaces__msg__Udp2master__Sequence * array)
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
      humanoid_interfaces__msg__Udp2master__fini(&array->data[i]);
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

humanoid_interfaces__msg__Udp2master__Sequence *
humanoid_interfaces__msg__Udp2master__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Udp2master__Sequence * array = (humanoid_interfaces__msg__Udp2master__Sequence *)allocator.allocate(sizeof(humanoid_interfaces__msg__Udp2master__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = humanoid_interfaces__msg__Udp2master__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
humanoid_interfaces__msg__Udp2master__Sequence__destroy(humanoid_interfaces__msg__Udp2master__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    humanoid_interfaces__msg__Udp2master__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
humanoid_interfaces__msg__Udp2master__Sequence__are_equal(const humanoid_interfaces__msg__Udp2master__Sequence * lhs, const humanoid_interfaces__msg__Udp2master__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!humanoid_interfaces__msg__Udp2master__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
humanoid_interfaces__msg__Udp2master__Sequence__copy(
  const humanoid_interfaces__msg__Udp2master__Sequence * input,
  humanoid_interfaces__msg__Udp2master__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(humanoid_interfaces__msg__Udp2master);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    humanoid_interfaces__msg__Udp2master * data =
      (humanoid_interfaces__msg__Udp2master *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!humanoid_interfaces__msg__Udp2master__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          humanoid_interfaces__msg__Udp2master__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!humanoid_interfaces__msg__Udp2master__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
