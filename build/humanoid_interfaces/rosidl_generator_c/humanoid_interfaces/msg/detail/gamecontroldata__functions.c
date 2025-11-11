// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from humanoid_interfaces:msg/Gamecontroldata.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/gamecontroldata__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `secondinfo`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
humanoid_interfaces__msg__Gamecontroldata__init(humanoid_interfaces__msg__Gamecontroldata * msg)
{
  if (!msg) {
    return false;
  }
  // robotnum
  // position
  // myteam
  // state
  // myside
  // iskickoff
  // readytime
  // penalty
  // secondstate
  // secondinfo
  if (!rosidl_runtime_c__int64__Sequence__init(&msg->secondinfo, 0)) {
    humanoid_interfaces__msg__Gamecontroldata__fini(msg);
    return false;
  }
  return true;
}

void
humanoid_interfaces__msg__Gamecontroldata__fini(humanoid_interfaces__msg__Gamecontroldata * msg)
{
  if (!msg) {
    return;
  }
  // robotnum
  // position
  // myteam
  // state
  // myside
  // iskickoff
  // readytime
  // penalty
  // secondstate
  // secondinfo
  rosidl_runtime_c__int64__Sequence__fini(&msg->secondinfo);
}

bool
humanoid_interfaces__msg__Gamecontroldata__are_equal(const humanoid_interfaces__msg__Gamecontroldata * lhs, const humanoid_interfaces__msg__Gamecontroldata * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robotnum
  if (lhs->robotnum != rhs->robotnum) {
    return false;
  }
  // position
  if (lhs->position != rhs->position) {
    return false;
  }
  // myteam
  if (lhs->myteam != rhs->myteam) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  // myside
  if (lhs->myside != rhs->myside) {
    return false;
  }
  // iskickoff
  if (lhs->iskickoff != rhs->iskickoff) {
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
humanoid_interfaces__msg__Gamecontroldata__copy(
  const humanoid_interfaces__msg__Gamecontroldata * input,
  humanoid_interfaces__msg__Gamecontroldata * output)
{
  if (!input || !output) {
    return false;
  }
  // robotnum
  output->robotnum = input->robotnum;
  // position
  output->position = input->position;
  // myteam
  output->myteam = input->myteam;
  // state
  output->state = input->state;
  // myside
  output->myside = input->myside;
  // iskickoff
  output->iskickoff = input->iskickoff;
  // readytime
  output->readytime = input->readytime;
  // penalty
  output->penalty = input->penalty;
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

humanoid_interfaces__msg__Gamecontroldata *
humanoid_interfaces__msg__Gamecontroldata__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Gamecontroldata * msg = (humanoid_interfaces__msg__Gamecontroldata *)allocator.allocate(sizeof(humanoid_interfaces__msg__Gamecontroldata), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(humanoid_interfaces__msg__Gamecontroldata));
  bool success = humanoid_interfaces__msg__Gamecontroldata__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
humanoid_interfaces__msg__Gamecontroldata__destroy(humanoid_interfaces__msg__Gamecontroldata * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    humanoid_interfaces__msg__Gamecontroldata__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
humanoid_interfaces__msg__Gamecontroldata__Sequence__init(humanoid_interfaces__msg__Gamecontroldata__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Gamecontroldata * data = NULL;

  if (size) {
    data = (humanoid_interfaces__msg__Gamecontroldata *)allocator.zero_allocate(size, sizeof(humanoid_interfaces__msg__Gamecontroldata), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = humanoid_interfaces__msg__Gamecontroldata__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        humanoid_interfaces__msg__Gamecontroldata__fini(&data[i - 1]);
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
humanoid_interfaces__msg__Gamecontroldata__Sequence__fini(humanoid_interfaces__msg__Gamecontroldata__Sequence * array)
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
      humanoid_interfaces__msg__Gamecontroldata__fini(&array->data[i]);
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

humanoid_interfaces__msg__Gamecontroldata__Sequence *
humanoid_interfaces__msg__Gamecontroldata__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__Gamecontroldata__Sequence * array = (humanoid_interfaces__msg__Gamecontroldata__Sequence *)allocator.allocate(sizeof(humanoid_interfaces__msg__Gamecontroldata__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = humanoid_interfaces__msg__Gamecontroldata__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
humanoid_interfaces__msg__Gamecontroldata__Sequence__destroy(humanoid_interfaces__msg__Gamecontroldata__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    humanoid_interfaces__msg__Gamecontroldata__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
humanoid_interfaces__msg__Gamecontroldata__Sequence__are_equal(const humanoid_interfaces__msg__Gamecontroldata__Sequence * lhs, const humanoid_interfaces__msg__Gamecontroldata__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!humanoid_interfaces__msg__Gamecontroldata__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
humanoid_interfaces__msg__Gamecontroldata__Sequence__copy(
  const humanoid_interfaces__msg__Gamecontroldata__Sequence * input,
  humanoid_interfaces__msg__Gamecontroldata__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(humanoid_interfaces__msg__Gamecontroldata);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    humanoid_interfaces__msg__Gamecontroldata * data =
      (humanoid_interfaces__msg__Gamecontroldata *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!humanoid_interfaces__msg__Gamecontroldata__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          humanoid_interfaces__msg__Gamecontroldata__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!humanoid_interfaces__msg__Gamecontroldata__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
