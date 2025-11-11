// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dynamixel_rdk_msgs:msg/DynamixelStatusMsgs.idl
// generated code does not contain a copyright notice
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_status_msgs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__init(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__fini(msg);
    return false;
  }
  // id
  // torque_enabled
  // error_status
  // present_position
  // present_velocity
  // present_acceleration
  // present_current
  // present_voltage
  // present_temperature
  // min_max_position
  return true;
}

void
dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__fini(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // id
  // torque_enabled
  // error_status
  // present_position
  // present_velocity
  // present_acceleration
  // present_current
  // present_voltage
  // present_temperature
  // min_max_position
}

bool
dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__are_equal(const dynamixel_rdk_msgs__msg__DynamixelStatusMsgs * lhs, const dynamixel_rdk_msgs__msg__DynamixelStatusMsgs * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // torque_enabled
  if (lhs->torque_enabled != rhs->torque_enabled) {
    return false;
  }
  // error_status
  if (lhs->error_status != rhs->error_status) {
    return false;
  }
  // present_position
  if (lhs->present_position != rhs->present_position) {
    return false;
  }
  // present_velocity
  if (lhs->present_velocity != rhs->present_velocity) {
    return false;
  }
  // present_acceleration
  if (lhs->present_acceleration != rhs->present_acceleration) {
    return false;
  }
  // present_current
  if (lhs->present_current != rhs->present_current) {
    return false;
  }
  // present_voltage
  if (lhs->present_voltage != rhs->present_voltage) {
    return false;
  }
  // present_temperature
  if (lhs->present_temperature != rhs->present_temperature) {
    return false;
  }
  // min_max_position
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->min_max_position[i] != rhs->min_max_position[i]) {
      return false;
    }
  }
  return true;
}

bool
dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__copy(
  const dynamixel_rdk_msgs__msg__DynamixelStatusMsgs * input,
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // id
  output->id = input->id;
  // torque_enabled
  output->torque_enabled = input->torque_enabled;
  // error_status
  output->error_status = input->error_status;
  // present_position
  output->present_position = input->present_position;
  // present_velocity
  output->present_velocity = input->present_velocity;
  // present_acceleration
  output->present_acceleration = input->present_acceleration;
  // present_current
  output->present_current = input->present_current;
  // present_voltage
  output->present_voltage = input->present_voltage;
  // present_temperature
  output->present_temperature = input->present_temperature;
  // min_max_position
  for (size_t i = 0; i < 2; ++i) {
    output->min_max_position[i] = input->min_max_position[i];
  }
  return true;
}

dynamixel_rdk_msgs__msg__DynamixelStatusMsgs *
dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs * msg = (dynamixel_rdk_msgs__msg__DynamixelStatusMsgs *)allocator.allocate(sizeof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs));
  bool success = dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__destroy(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence__init(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs * data = NULL;

  if (size) {
    data = (dynamixel_rdk_msgs__msg__DynamixelStatusMsgs *)allocator.zero_allocate(size, sizeof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__fini(&data[i - 1]);
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
dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence__fini(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence * array)
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
      dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__fini(&array->data[i]);
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

dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence *
dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence * array = (dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence *)allocator.allocate(sizeof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence__destroy(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence__are_equal(const dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence * lhs, const dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence__copy(
  const dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence * input,
  dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dynamixel_rdk_msgs__msg__DynamixelStatusMsgs);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dynamixel_rdk_msgs__msg__DynamixelStatusMsgs * data =
      (dynamixel_rdk_msgs__msg__DynamixelStatusMsgs *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dynamixel_rdk_msgs__msg__DynamixelStatusMsgs__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
