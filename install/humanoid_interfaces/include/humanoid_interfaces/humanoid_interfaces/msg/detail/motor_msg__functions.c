// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from humanoid_interfaces:msg/MotorMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/motor_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `id`
// Member `speed`
// Member `position`
// Member `type`
// Member `acceleration`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
humanoid_interfaces__msg__MotorMsg__init(humanoid_interfaces__msg__MotorMsg * msg)
{
  if (!msg) {
    return false;
  }
  // length
  // mode
  // id
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->id, 0)) {
    humanoid_interfaces__msg__MotorMsg__fini(msg);
    return false;
  }
  // speed
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->speed, 0)) {
    humanoid_interfaces__msg__MotorMsg__fini(msg);
    return false;
  }
  // position
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->position, 0)) {
    humanoid_interfaces__msg__MotorMsg__fini(msg);
    return false;
  }
  // type
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->type, 0)) {
    humanoid_interfaces__msg__MotorMsg__fini(msg);
    return false;
  }
  // acceleration
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->acceleration, 0)) {
    humanoid_interfaces__msg__MotorMsg__fini(msg);
    return false;
  }
  return true;
}

void
humanoid_interfaces__msg__MotorMsg__fini(humanoid_interfaces__msg__MotorMsg * msg)
{
  if (!msg) {
    return;
  }
  // length
  // mode
  // id
  rosidl_runtime_c__int32__Sequence__fini(&msg->id);
  // speed
  rosidl_runtime_c__int32__Sequence__fini(&msg->speed);
  // position
  rosidl_runtime_c__int32__Sequence__fini(&msg->position);
  // type
  rosidl_runtime_c__int32__Sequence__fini(&msg->type);
  // acceleration
  rosidl_runtime_c__int32__Sequence__fini(&msg->acceleration);
}

bool
humanoid_interfaces__msg__MotorMsg__are_equal(const humanoid_interfaces__msg__MotorMsg * lhs, const humanoid_interfaces__msg__MotorMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // length
  if (lhs->length != rhs->length) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->id), &(rhs->id)))
  {
    return false;
  }
  // speed
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->speed), &(rhs->speed)))
  {
    return false;
  }
  // position
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // type
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->type), &(rhs->type)))
  {
    return false;
  }
  // acceleration
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->acceleration), &(rhs->acceleration)))
  {
    return false;
  }
  return true;
}

bool
humanoid_interfaces__msg__MotorMsg__copy(
  const humanoid_interfaces__msg__MotorMsg * input,
  humanoid_interfaces__msg__MotorMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // length
  output->length = input->length;
  // mode
  output->mode = input->mode;
  // id
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->id), &(output->id)))
  {
    return false;
  }
  // speed
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->speed), &(output->speed)))
  {
    return false;
  }
  // position
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // type
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->type), &(output->type)))
  {
    return false;
  }
  // acceleration
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->acceleration), &(output->acceleration)))
  {
    return false;
  }
  return true;
}

humanoid_interfaces__msg__MotorMsg *
humanoid_interfaces__msg__MotorMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__MotorMsg * msg = (humanoid_interfaces__msg__MotorMsg *)allocator.allocate(sizeof(humanoid_interfaces__msg__MotorMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(humanoid_interfaces__msg__MotorMsg));
  bool success = humanoid_interfaces__msg__MotorMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
humanoid_interfaces__msg__MotorMsg__destroy(humanoid_interfaces__msg__MotorMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    humanoid_interfaces__msg__MotorMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
humanoid_interfaces__msg__MotorMsg__Sequence__init(humanoid_interfaces__msg__MotorMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__MotorMsg * data = NULL;

  if (size) {
    data = (humanoid_interfaces__msg__MotorMsg *)allocator.zero_allocate(size, sizeof(humanoid_interfaces__msg__MotorMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = humanoid_interfaces__msg__MotorMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        humanoid_interfaces__msg__MotorMsg__fini(&data[i - 1]);
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
humanoid_interfaces__msg__MotorMsg__Sequence__fini(humanoid_interfaces__msg__MotorMsg__Sequence * array)
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
      humanoid_interfaces__msg__MotorMsg__fini(&array->data[i]);
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

humanoid_interfaces__msg__MotorMsg__Sequence *
humanoid_interfaces__msg__MotorMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__MotorMsg__Sequence * array = (humanoid_interfaces__msg__MotorMsg__Sequence *)allocator.allocate(sizeof(humanoid_interfaces__msg__MotorMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = humanoid_interfaces__msg__MotorMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
humanoid_interfaces__msg__MotorMsg__Sequence__destroy(humanoid_interfaces__msg__MotorMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    humanoid_interfaces__msg__MotorMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
humanoid_interfaces__msg__MotorMsg__Sequence__are_equal(const humanoid_interfaces__msg__MotorMsg__Sequence * lhs, const humanoid_interfaces__msg__MotorMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!humanoid_interfaces__msg__MotorMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
humanoid_interfaces__msg__MotorMsg__Sequence__copy(
  const humanoid_interfaces__msg__MotorMsg__Sequence * input,
  humanoid_interfaces__msg__MotorMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(humanoid_interfaces__msg__MotorMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    humanoid_interfaces__msg__MotorMsg * data =
      (humanoid_interfaces__msg__MotorMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!humanoid_interfaces__msg__MotorMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          humanoid_interfaces__msg__MotorMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!humanoid_interfaces__msg__MotorMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
