// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from humanoid_interfaces:msg/Robocuplocalization25.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/robocuplocalization25__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "humanoid_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "humanoid_interfaces/msg/detail/robocuplocalization25__struct.h"
#include "humanoid_interfaces/msg/detail/robocuplocalization25__functions.h"
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


// forward declare type support functions


using _Robocuplocalization25__ros_msg_type = humanoid_interfaces__msg__Robocuplocalization25;

static bool _Robocuplocalization25__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Robocuplocalization25__ros_msg_type * ros_message = static_cast<const _Robocuplocalization25__ros_msg_type *>(untyped_ros_message);
  // Field name: ball_x
  {
    cdr << ros_message->ball_x;
  }

  // Field name: ball_y
  {
    cdr << ros_message->ball_y;
  }

  // Field name: ball_speed_x
  {
    cdr << ros_message->ball_speed_x;
  }

  // Field name: ball_speed_y
  {
    cdr << ros_message->ball_speed_y;
  }

  // Field name: robot_x
  {
    cdr << ros_message->robot_x;
  }

  // Field name: robot_y
  {
    cdr << ros_message->robot_y;
  }

  // Field name: obstacle_x
  {
    cdr << ros_message->obstacle_x;
  }

  // Field name: obstacle_y
  {
    cdr << ros_message->obstacle_y;
  }

  // Field name: obstacle0_x
  {
    cdr << ros_message->obstacle0_x;
  }

  // Field name: obstacle0_y
  {
    cdr << ros_message->obstacle0_y;
  }

  // Field name: obstacle1_x
  {
    cdr << ros_message->obstacle1_x;
  }

  // Field name: obstacle1_y
  {
    cdr << ros_message->obstacle1_y;
  }

  // Field name: obstacle2_x
  {
    cdr << ros_message->obstacle2_x;
  }

  // Field name: obstacle2_y
  {
    cdr << ros_message->obstacle2_y;
  }

  // Field name: obstacle3_x
  {
    cdr << ros_message->obstacle3_x;
  }

  // Field name: obstacle3_y
  {
    cdr << ros_message->obstacle3_y;
  }

  return true;
}

static bool _Robocuplocalization25__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Robocuplocalization25__ros_msg_type * ros_message = static_cast<_Robocuplocalization25__ros_msg_type *>(untyped_ros_message);
  // Field name: ball_x
  {
    cdr >> ros_message->ball_x;
  }

  // Field name: ball_y
  {
    cdr >> ros_message->ball_y;
  }

  // Field name: ball_speed_x
  {
    cdr >> ros_message->ball_speed_x;
  }

  // Field name: ball_speed_y
  {
    cdr >> ros_message->ball_speed_y;
  }

  // Field name: robot_x
  {
    cdr >> ros_message->robot_x;
  }

  // Field name: robot_y
  {
    cdr >> ros_message->robot_y;
  }

  // Field name: obstacle_x
  {
    cdr >> ros_message->obstacle_x;
  }

  // Field name: obstacle_y
  {
    cdr >> ros_message->obstacle_y;
  }

  // Field name: obstacle0_x
  {
    cdr >> ros_message->obstacle0_x;
  }

  // Field name: obstacle0_y
  {
    cdr >> ros_message->obstacle0_y;
  }

  // Field name: obstacle1_x
  {
    cdr >> ros_message->obstacle1_x;
  }

  // Field name: obstacle1_y
  {
    cdr >> ros_message->obstacle1_y;
  }

  // Field name: obstacle2_x
  {
    cdr >> ros_message->obstacle2_x;
  }

  // Field name: obstacle2_y
  {
    cdr >> ros_message->obstacle2_y;
  }

  // Field name: obstacle3_x
  {
    cdr >> ros_message->obstacle3_x;
  }

  // Field name: obstacle3_y
  {
    cdr >> ros_message->obstacle3_y;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t get_serialized_size_humanoid_interfaces__msg__Robocuplocalization25(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Robocuplocalization25__ros_msg_type * ros_message = static_cast<const _Robocuplocalization25__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name ball_x
  {
    size_t item_size = sizeof(ros_message->ball_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ball_y
  {
    size_t item_size = sizeof(ros_message->ball_y);
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
  // field.name robot_x
  {
    size_t item_size = sizeof(ros_message->robot_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name robot_y
  {
    size_t item_size = sizeof(ros_message->robot_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name obstacle_x
  {
    size_t item_size = sizeof(ros_message->obstacle_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name obstacle_y
  {
    size_t item_size = sizeof(ros_message->obstacle_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name obstacle0_x
  {
    size_t item_size = sizeof(ros_message->obstacle0_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name obstacle0_y
  {
    size_t item_size = sizeof(ros_message->obstacle0_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name obstacle1_x
  {
    size_t item_size = sizeof(ros_message->obstacle1_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name obstacle1_y
  {
    size_t item_size = sizeof(ros_message->obstacle1_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name obstacle2_x
  {
    size_t item_size = sizeof(ros_message->obstacle2_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name obstacle2_y
  {
    size_t item_size = sizeof(ros_message->obstacle2_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name obstacle3_x
  {
    size_t item_size = sizeof(ros_message->obstacle3_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name obstacle3_y
  {
    size_t item_size = sizeof(ros_message->obstacle3_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Robocuplocalization25__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_humanoid_interfaces__msg__Robocuplocalization25(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t max_serialized_size_humanoid_interfaces__msg__Robocuplocalization25(
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

  // member: ball_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ball_y
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
  // member: robot_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: robot_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: obstacle_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: obstacle_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: obstacle0_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: obstacle0_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: obstacle1_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: obstacle1_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: obstacle2_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: obstacle2_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: obstacle3_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: obstacle3_y
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
    using DataType = humanoid_interfaces__msg__Robocuplocalization25;
    is_plain =
      (
      offsetof(DataType, obstacle3_y) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Robocuplocalization25__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_humanoid_interfaces__msg__Robocuplocalization25(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Robocuplocalization25 = {
  "humanoid_interfaces::msg",
  "Robocuplocalization25",
  _Robocuplocalization25__cdr_serialize,
  _Robocuplocalization25__cdr_deserialize,
  _Robocuplocalization25__get_serialized_size,
  _Robocuplocalization25__max_serialized_size
};

static rosidl_message_type_support_t _Robocuplocalization25__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Robocuplocalization25,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, humanoid_interfaces, msg, Robocuplocalization25)() {
  return &_Robocuplocalization25__type_support;
}

#if defined(__cplusplus)
}
#endif
