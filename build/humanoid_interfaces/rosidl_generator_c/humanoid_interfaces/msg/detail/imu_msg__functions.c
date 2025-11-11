// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from humanoid_interfaces:msg/ImuMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/imu_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
humanoid_interfaces__msg__ImuMsg__init(humanoid_interfaces__msg__ImuMsg * msg)
{
  if (!msg) {
    return false;
  }
  // pitch
  // roll
  // yaw
  // q_x
  // q_y
  // q_z
  // q_w
  // x_gyro
  // y_gyro
  // z_gyro
  // x_acc
  // y_acc
  // z_acc
  // x_vel
  // y_vel
  // z_vel
  // x_pos
  // y_pos
  // z_pos
  return true;
}

void
humanoid_interfaces__msg__ImuMsg__fini(humanoid_interfaces__msg__ImuMsg * msg)
{
  if (!msg) {
    return;
  }
  // pitch
  // roll
  // yaw
  // q_x
  // q_y
  // q_z
  // q_w
  // x_gyro
  // y_gyro
  // z_gyro
  // x_acc
  // y_acc
  // z_acc
  // x_vel
  // y_vel
  // z_vel
  // x_pos
  // y_pos
  // z_pos
}

bool
humanoid_interfaces__msg__ImuMsg__are_equal(const humanoid_interfaces__msg__ImuMsg * lhs, const humanoid_interfaces__msg__ImuMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // q_x
  if (lhs->q_x != rhs->q_x) {
    return false;
  }
  // q_y
  if (lhs->q_y != rhs->q_y) {
    return false;
  }
  // q_z
  if (lhs->q_z != rhs->q_z) {
    return false;
  }
  // q_w
  if (lhs->q_w != rhs->q_w) {
    return false;
  }
  // x_gyro
  if (lhs->x_gyro != rhs->x_gyro) {
    return false;
  }
  // y_gyro
  if (lhs->y_gyro != rhs->y_gyro) {
    return false;
  }
  // z_gyro
  if (lhs->z_gyro != rhs->z_gyro) {
    return false;
  }
  // x_acc
  if (lhs->x_acc != rhs->x_acc) {
    return false;
  }
  // y_acc
  if (lhs->y_acc != rhs->y_acc) {
    return false;
  }
  // z_acc
  if (lhs->z_acc != rhs->z_acc) {
    return false;
  }
  // x_vel
  if (lhs->x_vel != rhs->x_vel) {
    return false;
  }
  // y_vel
  if (lhs->y_vel != rhs->y_vel) {
    return false;
  }
  // z_vel
  if (lhs->z_vel != rhs->z_vel) {
    return false;
  }
  // x_pos
  if (lhs->x_pos != rhs->x_pos) {
    return false;
  }
  // y_pos
  if (lhs->y_pos != rhs->y_pos) {
    return false;
  }
  // z_pos
  if (lhs->z_pos != rhs->z_pos) {
    return false;
  }
  return true;
}

bool
humanoid_interfaces__msg__ImuMsg__copy(
  const humanoid_interfaces__msg__ImuMsg * input,
  humanoid_interfaces__msg__ImuMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // pitch
  output->pitch = input->pitch;
  // roll
  output->roll = input->roll;
  // yaw
  output->yaw = input->yaw;
  // q_x
  output->q_x = input->q_x;
  // q_y
  output->q_y = input->q_y;
  // q_z
  output->q_z = input->q_z;
  // q_w
  output->q_w = input->q_w;
  // x_gyro
  output->x_gyro = input->x_gyro;
  // y_gyro
  output->y_gyro = input->y_gyro;
  // z_gyro
  output->z_gyro = input->z_gyro;
  // x_acc
  output->x_acc = input->x_acc;
  // y_acc
  output->y_acc = input->y_acc;
  // z_acc
  output->z_acc = input->z_acc;
  // x_vel
  output->x_vel = input->x_vel;
  // y_vel
  output->y_vel = input->y_vel;
  // z_vel
  output->z_vel = input->z_vel;
  // x_pos
  output->x_pos = input->x_pos;
  // y_pos
  output->y_pos = input->y_pos;
  // z_pos
  output->z_pos = input->z_pos;
  return true;
}

humanoid_interfaces__msg__ImuMsg *
humanoid_interfaces__msg__ImuMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__ImuMsg * msg = (humanoid_interfaces__msg__ImuMsg *)allocator.allocate(sizeof(humanoid_interfaces__msg__ImuMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(humanoid_interfaces__msg__ImuMsg));
  bool success = humanoid_interfaces__msg__ImuMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
humanoid_interfaces__msg__ImuMsg__destroy(humanoid_interfaces__msg__ImuMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    humanoid_interfaces__msg__ImuMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
humanoid_interfaces__msg__ImuMsg__Sequence__init(humanoid_interfaces__msg__ImuMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__ImuMsg * data = NULL;

  if (size) {
    data = (humanoid_interfaces__msg__ImuMsg *)allocator.zero_allocate(size, sizeof(humanoid_interfaces__msg__ImuMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = humanoid_interfaces__msg__ImuMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        humanoid_interfaces__msg__ImuMsg__fini(&data[i - 1]);
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
humanoid_interfaces__msg__ImuMsg__Sequence__fini(humanoid_interfaces__msg__ImuMsg__Sequence * array)
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
      humanoid_interfaces__msg__ImuMsg__fini(&array->data[i]);
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

humanoid_interfaces__msg__ImuMsg__Sequence *
humanoid_interfaces__msg__ImuMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  humanoid_interfaces__msg__ImuMsg__Sequence * array = (humanoid_interfaces__msg__ImuMsg__Sequence *)allocator.allocate(sizeof(humanoid_interfaces__msg__ImuMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = humanoid_interfaces__msg__ImuMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
humanoid_interfaces__msg__ImuMsg__Sequence__destroy(humanoid_interfaces__msg__ImuMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    humanoid_interfaces__msg__ImuMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
humanoid_interfaces__msg__ImuMsg__Sequence__are_equal(const humanoid_interfaces__msg__ImuMsg__Sequence * lhs, const humanoid_interfaces__msg__ImuMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!humanoid_interfaces__msg__ImuMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
humanoid_interfaces__msg__ImuMsg__Sequence__copy(
  const humanoid_interfaces__msg__ImuMsg__Sequence * input,
  humanoid_interfaces__msg__ImuMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(humanoid_interfaces__msg__ImuMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    humanoid_interfaces__msg__ImuMsg * data =
      (humanoid_interfaces__msg__ImuMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!humanoid_interfaces__msg__ImuMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          humanoid_interfaces__msg__ImuMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!humanoid_interfaces__msg__ImuMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
