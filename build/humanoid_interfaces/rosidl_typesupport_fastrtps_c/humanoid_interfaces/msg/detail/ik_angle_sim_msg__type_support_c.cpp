// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from humanoid_interfaces:msg/IkAngleSimMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/ik_angle_sim_msg__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "humanoid_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "humanoid_interfaces/msg/detail/ik_angle_sim_msg__struct.h"
#include "humanoid_interfaces/msg/detail/ik_angle_sim_msg__functions.h"
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


using _IkAngleSimMsg__ros_msg_type = humanoid_interfaces__msg__IkAngleSimMsg;

static bool _IkAngleSimMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _IkAngleSimMsg__ros_msg_type * ros_message = static_cast<const _IkAngleSimMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: ang_l_0
  {
    cdr << ros_message->ang_l_0;
  }

  // Field name: ang_l_1
  {
    cdr << ros_message->ang_l_1;
  }

  // Field name: ang_l_2
  {
    cdr << ros_message->ang_l_2;
  }

  // Field name: ang_l_3
  {
    cdr << ros_message->ang_l_3;
  }

  // Field name: ang_l_4
  {
    cdr << ros_message->ang_l_4;
  }

  // Field name: ang_l_5
  {
    cdr << ros_message->ang_l_5;
  }

  // Field name: ang_l_6
  {
    cdr << ros_message->ang_l_6;
  }

  // Field name: ang_l_7
  {
    cdr << ros_message->ang_l_7;
  }

  // Field name: ang_l_8
  {
    cdr << ros_message->ang_l_8;
  }

  // Field name: ang_r_0
  {
    cdr << ros_message->ang_r_0;
  }

  // Field name: ang_r_1
  {
    cdr << ros_message->ang_r_1;
  }

  // Field name: ang_r_2
  {
    cdr << ros_message->ang_r_2;
  }

  // Field name: ang_r_3
  {
    cdr << ros_message->ang_r_3;
  }

  // Field name: ang_r_4
  {
    cdr << ros_message->ang_r_4;
  }

  // Field name: ang_r_5
  {
    cdr << ros_message->ang_r_5;
  }

  // Field name: ang_r_6
  {
    cdr << ros_message->ang_r_6;
  }

  // Field name: ang_r_7
  {
    cdr << ros_message->ang_r_7;
  }

  // Field name: ang_r_8
  {
    cdr << ros_message->ang_r_8;
  }

  return true;
}

static bool _IkAngleSimMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _IkAngleSimMsg__ros_msg_type * ros_message = static_cast<_IkAngleSimMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: ang_l_0
  {
    cdr >> ros_message->ang_l_0;
  }

  // Field name: ang_l_1
  {
    cdr >> ros_message->ang_l_1;
  }

  // Field name: ang_l_2
  {
    cdr >> ros_message->ang_l_2;
  }

  // Field name: ang_l_3
  {
    cdr >> ros_message->ang_l_3;
  }

  // Field name: ang_l_4
  {
    cdr >> ros_message->ang_l_4;
  }

  // Field name: ang_l_5
  {
    cdr >> ros_message->ang_l_5;
  }

  // Field name: ang_l_6
  {
    cdr >> ros_message->ang_l_6;
  }

  // Field name: ang_l_7
  {
    cdr >> ros_message->ang_l_7;
  }

  // Field name: ang_l_8
  {
    cdr >> ros_message->ang_l_8;
  }

  // Field name: ang_r_0
  {
    cdr >> ros_message->ang_r_0;
  }

  // Field name: ang_r_1
  {
    cdr >> ros_message->ang_r_1;
  }

  // Field name: ang_r_2
  {
    cdr >> ros_message->ang_r_2;
  }

  // Field name: ang_r_3
  {
    cdr >> ros_message->ang_r_3;
  }

  // Field name: ang_r_4
  {
    cdr >> ros_message->ang_r_4;
  }

  // Field name: ang_r_5
  {
    cdr >> ros_message->ang_r_5;
  }

  // Field name: ang_r_6
  {
    cdr >> ros_message->ang_r_6;
  }

  // Field name: ang_r_7
  {
    cdr >> ros_message->ang_r_7;
  }

  // Field name: ang_r_8
  {
    cdr >> ros_message->ang_r_8;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t get_serialized_size_humanoid_interfaces__msg__IkAngleSimMsg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _IkAngleSimMsg__ros_msg_type * ros_message = static_cast<const _IkAngleSimMsg__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name ang_l_0
  {
    size_t item_size = sizeof(ros_message->ang_l_0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_l_1
  {
    size_t item_size = sizeof(ros_message->ang_l_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_l_2
  {
    size_t item_size = sizeof(ros_message->ang_l_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_l_3
  {
    size_t item_size = sizeof(ros_message->ang_l_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_l_4
  {
    size_t item_size = sizeof(ros_message->ang_l_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_l_5
  {
    size_t item_size = sizeof(ros_message->ang_l_5);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_l_6
  {
    size_t item_size = sizeof(ros_message->ang_l_6);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_l_7
  {
    size_t item_size = sizeof(ros_message->ang_l_7);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_l_8
  {
    size_t item_size = sizeof(ros_message->ang_l_8);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_r_0
  {
    size_t item_size = sizeof(ros_message->ang_r_0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_r_1
  {
    size_t item_size = sizeof(ros_message->ang_r_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_r_2
  {
    size_t item_size = sizeof(ros_message->ang_r_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_r_3
  {
    size_t item_size = sizeof(ros_message->ang_r_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_r_4
  {
    size_t item_size = sizeof(ros_message->ang_r_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_r_5
  {
    size_t item_size = sizeof(ros_message->ang_r_5);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_r_6
  {
    size_t item_size = sizeof(ros_message->ang_r_6);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_r_7
  {
    size_t item_size = sizeof(ros_message->ang_r_7);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ang_r_8
  {
    size_t item_size = sizeof(ros_message->ang_r_8);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _IkAngleSimMsg__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_humanoid_interfaces__msg__IkAngleSimMsg(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t max_serialized_size_humanoid_interfaces__msg__IkAngleSimMsg(
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

  // member: ang_l_0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_l_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_l_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_l_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_l_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_l_5
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_l_6
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_l_7
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_l_8
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_r_0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_r_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_r_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_r_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_r_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_r_5
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_r_6
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_r_7
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ang_r_8
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
    using DataType = humanoid_interfaces__msg__IkAngleSimMsg;
    is_plain =
      (
      offsetof(DataType, ang_r_8) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _IkAngleSimMsg__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_humanoid_interfaces__msg__IkAngleSimMsg(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_IkAngleSimMsg = {
  "humanoid_interfaces::msg",
  "IkAngleSimMsg",
  _IkAngleSimMsg__cdr_serialize,
  _IkAngleSimMsg__cdr_deserialize,
  _IkAngleSimMsg__get_serialized_size,
  _IkAngleSimMsg__max_serialized_size
};

static rosidl_message_type_support_t _IkAngleSimMsg__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_IkAngleSimMsg,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, humanoid_interfaces, msg, IkAngleSimMsg)() {
  return &_IkAngleSimMsg__type_support;
}

#if defined(__cplusplus)
}
#endif
