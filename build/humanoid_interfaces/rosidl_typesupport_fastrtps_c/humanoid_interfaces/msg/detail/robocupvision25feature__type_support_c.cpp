// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from humanoid_interfaces:msg/Robocupvision25feature.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/robocupvision25feature__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "humanoid_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "humanoid_interfaces/msg/detail/robocupvision25feature__struct.h"
#include "humanoid_interfaces/msg/detail/robocupvision25feature__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // confidence, distance, point_vec_x, point_vec_y
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // confidence, distance, point_vec_x, point_vec_y

// forward declare type support functions


using _Robocupvision25feature__ros_msg_type = humanoid_interfaces__msg__Robocupvision25feature;

static bool _Robocupvision25feature__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Robocupvision25feature__ros_msg_type * ros_message = static_cast<const _Robocupvision25feature__ros_msg_type *>(untyped_ros_message);
  // Field name: confidence
  {
    size_t size = ros_message->confidence.size;
    auto array_ptr = ros_message->confidence.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: distance
  {
    size_t size = ros_message->distance.size;
    auto array_ptr = ros_message->distance.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: point_vec_x
  {
    size_t size = ros_message->point_vec_x.size;
    auto array_ptr = ros_message->point_vec_x.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: point_vec_y
  {
    size_t size = ros_message->point_vec_y.size;
    auto array_ptr = ros_message->point_vec_y.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _Robocupvision25feature__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Robocupvision25feature__ros_msg_type * ros_message = static_cast<_Robocupvision25feature__ros_msg_type *>(untyped_ros_message);
  // Field name: confidence
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->confidence.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->confidence);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->confidence, size)) {
      fprintf(stderr, "failed to create array for field 'confidence'");
      return false;
    }
    auto array_ptr = ros_message->confidence.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: distance
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->distance.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->distance);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->distance, size)) {
      fprintf(stderr, "failed to create array for field 'distance'");
      return false;
    }
    auto array_ptr = ros_message->distance.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: point_vec_x
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->point_vec_x.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->point_vec_x);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->point_vec_x, size)) {
      fprintf(stderr, "failed to create array for field 'point_vec_x'");
      return false;
    }
    auto array_ptr = ros_message->point_vec_x.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: point_vec_y
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->point_vec_y.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->point_vec_y);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->point_vec_y, size)) {
      fprintf(stderr, "failed to create array for field 'point_vec_y'");
      return false;
    }
    auto array_ptr = ros_message->point_vec_y.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t get_serialized_size_humanoid_interfaces__msg__Robocupvision25feature(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Robocupvision25feature__ros_msg_type * ros_message = static_cast<const _Robocupvision25feature__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name confidence
  {
    size_t array_size = ros_message->confidence.size;
    auto array_ptr = ros_message->confidence.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name distance
  {
    size_t array_size = ros_message->distance.size;
    auto array_ptr = ros_message->distance.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name point_vec_x
  {
    size_t array_size = ros_message->point_vec_x.size;
    auto array_ptr = ros_message->point_vec_x.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name point_vec_y
  {
    size_t array_size = ros_message->point_vec_y.size;
    auto array_ptr = ros_message->point_vec_y.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Robocupvision25feature__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_humanoid_interfaces__msg__Robocupvision25feature(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t max_serialized_size_humanoid_interfaces__msg__Robocupvision25feature(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: confidence
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: distance
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: point_vec_x
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: point_vec_y
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = humanoid_interfaces__msg__Robocupvision25feature;
    is_plain =
      (
      offsetof(DataType, point_vec_y) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Robocupvision25feature__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_humanoid_interfaces__msg__Robocupvision25feature(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Robocupvision25feature = {
  "humanoid_interfaces::msg",
  "Robocupvision25feature",
  _Robocupvision25feature__cdr_serialize,
  _Robocupvision25feature__cdr_deserialize,
  _Robocupvision25feature__get_serialized_size,
  _Robocupvision25feature__max_serialized_size
};

static rosidl_message_type_support_t _Robocupvision25feature__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Robocupvision25feature,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, humanoid_interfaces, msg, Robocupvision25feature)() {
  return &_Robocupvision25feature__type_support;
}

#if defined(__cplusplus)
}
#endif
