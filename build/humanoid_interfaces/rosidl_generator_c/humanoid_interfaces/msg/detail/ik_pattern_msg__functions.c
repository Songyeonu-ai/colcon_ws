// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from humanoid_interfaces:msg/IkPatternMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/ik_pattern_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
humanoid_interfaces__msg__IkPatternMsg__init(humanoid_interfaces__msg__IkPatternMsg * msg)
{
  if (!msg) {
    return false;
  }
  // xlpattern
  // ylpattern
  // zlpattern
  // slpattern
  // tlpattern
  // xrpattern
  // yrpattern
  // zrpattern
  // srpattern
  // trpattern
  return true;
}

void
humanoid_interfaces__msg__IkPatternMsg__fini(humanoid_interfaces__msg__IkPatternMsg * msg)
{
  if (!msg) {
    return;
  }
  // xlpattern
  // ylpattern
  // zlpattern
  // slpattern
  // tlpattern
  // xrpattern
  // yrpattern
  // zrpattern
  // srpattern
  // trpattern
}

bool
humanoid_interfaces__msg__IkPatternMsg__are_equal(const humanoid_interfaces__msg__IkPatternMsg * lhs, const humanoid_interfaces__msg__IkPatternMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // xlpattern
  if (lhs->xlpattern != rhs->xlpattern) {
    return false;
  }
  // ylpattern
  if (lhs->ylpattern != rhs->ylpattern) {
    return false;
  }
  // zlpattern
  if (lhs->zlpattern != rhs->zlpattern) {
    return false;
  }
  // slpattern
  if (lhs->slpattern != rhs->slpattern) {
    return false;
  }
  // tlpattern
  if (lhs->tlpattern != rhs->tlpattern) {
    return false;
  }
  // xrpattern
  if (lhs->xrpattern != rhs->xrpattern) {
    return false;
  }
  // yrpattern
  if (lhs->yrpattern != rhs->yrpattern) {
    return false;
  }
  // zrpattern
  if (lhs->zrpattern != rhs->zrpattern) {
    return false;
  }
  // srpattern
  if (lhs->srpattern != rhs->srpattern) {
    return false;
  }
  // trpattern
  if (lhs->trpattern != rhs->trpattern) {
    return false;
  }
  return true;
}

bool
humanoid_interfaces__msg__IkPatternMsg__copy(
  const humanoid_interfaces__msg__IkPatternMsg * input,
  humanoid_interfaces__msg__IkPatternMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // xlpattern
  output->xlpattern = input->xlpattern;
  // ylpattern
  output->ylpattern = input->ylpattern;
  // zlpattern
  output->zlpattern = input->zlpattern;
  // slpattern
  output->slpattern = input->slpattern;
  // tlpattern
  output->tlpattern = input->tlpattern;
  // xrpattern
  output->xrpattern = input->xrpattern;
  // yrpattern
  output->yrpattern = input->yrpattern;
  // zrpattern
  output->zrpattern = input->zrpattern;
  // srpattern
  output->srpattern = input->srpattern;
  // trpattern
  output->trpattern = input->trpattern;
  return true;
}

humanoid_interfaces__msg__IkPatternMsg *
humanoid_interfaces__msg__IkPatternMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__IkPatternMsg * msg = (humanoid_interfaces__msg__IkPatternMsg *)allocator.allocate(sizeof(humanoid_interfaces__msg__IkPatternMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(humanoid_interfaces__msg__IkPatternMsg));
  bool success = humanoid_interfaces__msg__IkPatternMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
humanoid_interfaces__msg__IkPatternMsg__destroy(humanoid_interfaces__msg__IkPatternMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    humanoid_interfaces__msg__IkPatternMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
humanoid_interfaces__msg__IkPatternMsg__Sequence__init(humanoid_interfaces__msg__IkPatternMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__IkPatternMsg * data = NULL;

  if (size) {
    data = (humanoid_interfaces__msg__IkPatternMsg *)allocator.zero_allocate(size, sizeof(humanoid_interfaces__msg__IkPatternMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = humanoid_interfaces__msg__IkPatternMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        humanoid_interfaces__msg__IkPatternMsg__fini(&data[i - 1]);
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
humanoid_interfaces__msg__IkPatternMsg__Sequence__fini(humanoid_interfaces__msg__IkPatternMsg__Sequence * array)
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
      humanoid_interfaces__msg__IkPatternMsg__fini(&array->data[i]);
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

humanoid_interfaces__msg__IkPatternMsg__Sequence *
humanoid_interfaces__msg__IkPatternMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__IkPatternMsg__Sequence * array = (humanoid_interfaces__msg__IkPatternMsg__Sequence *)allocator.allocate(sizeof(humanoid_interfaces__msg__IkPatternMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = humanoid_interfaces__msg__IkPatternMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
humanoid_interfaces__msg__IkPatternMsg__Sequence__destroy(humanoid_interfaces__msg__IkPatternMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    humanoid_interfaces__msg__IkPatternMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
humanoid_interfaces__msg__IkPatternMsg__Sequence__are_equal(const humanoid_interfaces__msg__IkPatternMsg__Sequence * lhs, const humanoid_interfaces__msg__IkPatternMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!humanoid_interfaces__msg__IkPatternMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
humanoid_interfaces__msg__IkPatternMsg__Sequence__copy(
  const humanoid_interfaces__msg__IkPatternMsg__Sequence * input,
  humanoid_interfaces__msg__IkPatternMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(humanoid_interfaces__msg__IkPatternMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    humanoid_interfaces__msg__IkPatternMsg * data =
      (humanoid_interfaces__msg__IkPatternMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!humanoid_interfaces__msg__IkPatternMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          humanoid_interfaces__msg__IkPatternMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!humanoid_interfaces__msg__IkPatternMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
