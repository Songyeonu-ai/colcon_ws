// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from humanoid_interfaces:msg/Robocupvision25.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/robocupvision25__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "humanoid_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "humanoid_interfaces/msg/detail/robocupvision25__struct.h"
#include "humanoid_interfaces/msg/detail/robocupvision25__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // robot_vec_x, robot_vec_y
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // robot_vec_x, robot_vec_y

// forward declare type support functions


using _Robocupvision25__ros_msg_type = humanoid_interfaces__msg__Robocupvision25;

static bool _Robocupvision25__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Robocupvision25__ros_msg_type * ros_message = static_cast<const _Robocupvision25__ros_msg_type *>(untyped_ros_message);
  // Field name: ball_cam_x
  {
    cdr << ros_message->ball_cam_x;
  }

  // Field name: ball_cam_y
  {
    cdr << ros_message->ball_cam_y;
  }

  // Field name: ball_2d_x
  {
    cdr << ros_message->ball_2d_x;
  }

  // Field name: ball_2d_y
  {
    cdr << ros_message->ball_2d_y;
  }

  // Field name: ball_d
  {
    cdr << ros_message->ball_d;
  }

  // Field name: pan
  {
    cdr << ros_message->pan;
  }

  // Field name: tilt
  {
    cdr << ros_message->tilt;
  }

  // Field name: ball_speed_x
  {
    cdr << ros_message->ball_speed_x;
  }

  // Field name: ball_speed_y
  {
    cdr << ros_message->ball_speed_y;
  }

  // Field name: robot_vec_x
  {
    size_t size = ros_message->robot_vec_x.size;
    auto array_ptr = ros_message->robot_vec_x.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: robot_vec_y
  {
    size_t size = ros_message->robot_vec_y.size;
    auto array_ptr = ros_message->robot_vec_y.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: ball_speed_level
  {
    cdr << ros_message->ball_speed_level;
  }

  // Field name: scan_mode
  {
    cdr << ros_message->scan_mode;
  }

  return true;
}

static bool _Robocupvision25__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Robocupvision25__ros_msg_type * ros_message = static_cast<_Robocupvision25__ros_msg_type *>(untyped_ros_message);
  // Field name: ball_cam_x
  {
    cdr >> ros_message->ball_cam_x;
  }

  // Field name: ball_cam_y
  {
    cdr >> ros_message->ball_cam_y;
  }

  // Field name: ball_2d_x
  {
    cdr >> ros_message->ball_2d_x;
  }

  // Field name: ball_2d_y
  {
    cdr >> ros_message->ball_2d_y;
  }

  // Field name: ball_d
  {
    cdr >> ros_message->ball_d;
  }

  // Field name: pan
  {
    cdr >> ros_message->pan;
  }

  // Field name: tilt
  {
    cdr >> ros_message->tilt;
  }

  // Field name: ball_speed_x
  {
    cdr >> ros_message->ball_speed_x;
  }

  // Field name: ball_speed_y
  {
    cdr >> ros_message->ball_speed_y;
  }

  // Field name: robot_vec_x
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->robot_vec_x.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->robot_vec_x);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->robot_vec_x, size)) {
      fprintf(stderr, "failed to create array for field 'robot_vec_x'");
      return false;
    }
    auto array_ptr = ros_message->robot_vec_x.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: robot_vec_y
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->robot_vec_y.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->robot_vec_y);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->robot_vec_y, size)) {
      fprintf(stderr, "failed to create array for field 'robot_vec_y'");
      return false;
    }
    auto array_ptr = ros_message->robot_vec_y.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: ball_speed_level
  {
    cdr >> ros_message->ball_speed_level;
  }

  // Field name: scan_mode
  {
    cdr >> ros_message->scan_mode;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t get_serialized_size_humanoid_interfaces__msg__Robocupvision25(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Robocupvision25__ros_msg_type * ros_message = static_cast<const _Robocupvision25__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name ball_cam_x
  {
    size_t item_size = sizeof(ros_message->ball_cam_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ball_cam_y
  {
    size_t item_size = sizeof(ros_message->ball_cam_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ball_2d_x
  {
    size_t item_size = sizeof(ros_message->ball_2d_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ball_2d_y
  {
    size_t item_size = sizeof(ros_message->ball_2d_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ball_d
  {
    size_t item_size = sizeof(ros_message->ball_d);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pan
  {
    size_t item_size = sizeof(ros_message->pan);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tilt
  {
    size_t item_size = sizeof(ros_message->tilt);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ball_speed_x
  {
    size_t item_size = sizeof(ros_message->ball_speed_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ball_speed_y
  {
    size_t item_size = sizeof(ros_message->ball_speed_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name robot_vec_x
  {
    size_t array_size = ros_message->robot_vec_x.size;
    auto array_ptr = ros_message->robot_vec_x.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name robot_vec_y
  {
    size_t array_size = ros_message->robot_vec_y.size;
    auto array_ptr = ros_message->robot_vec_y.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ball_speed_level
  {
    size_t item_size = sizeof(ros_message->ball_speed_level);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name scan_mode
  {
    size_t item_size = sizeof(ros_message->scan_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Robocupvision25__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_humanoid_interfaces__msg__Robocupvision25(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t max_serialized_size_humanoid_interfaces__msg__Robocupvision25(
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

  // member: ball_cam_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ball_cam_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ball_2d_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ball_2d_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ball_d
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: pan
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: tilt
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ball_speed_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ball_speed_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: robot_vec_x
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
  // member: robot_vec_y
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
  // member: ball_speed_level
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: scan_mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = humanoid_interfaces__msg__Robocupvision25;
    is_plain =
      (
      offsetof(DataType, scan_mode) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Robocupvision25__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_humanoid_interfaces__msg__Robocupvision25(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Robocupvision25 = {
  "humanoid_interfaces::msg",
  "Robocupvision25",
  _Robocupvision25__cdr_serialize,
  _Robocupvision25__cdr_deserialize,
  _Robocupvision25__get_serialized_size,
  _Robocupvision25__max_serialized_size
};

static rosidl_message_type_support_t _Robocupvision25__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Robocupvision25,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, humanoid_interfaces, msg, Robocupvision25)() {
  return &_Robocupvision25__type_support;
}

#if defined(__cplusplus)
}
#endif
