// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from humanoid_interfaces:msg/IkPatternMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/ik_pattern_msg__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "humanoid_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "humanoid_interfaces/msg/detail/ik_pattern_msg__struct.h"
#include "humanoid_interfaces/msg/detail/ik_pattern_msg__functions.h"
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


using _IkPatternMsg__ros_msg_type = humanoid_interfaces__msg__IkPatternMsg;

static bool _IkPatternMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _IkPatternMsg__ros_msg_type * ros_message = static_cast<const _IkPatternMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: xlpattern
  {
    cdr << ros_message->xlpattern;
  }

  // Field name: ylpattern
  {
    cdr << ros_message->ylpattern;
  }

  // Field name: zlpattern
  {
    cdr << ros_message->zlpattern;
  }

  // Field name: slpattern
  {
    cdr << ros_message->slpattern;
  }

  // Field name: tlpattern
  {
    cdr << ros_message->tlpattern;
  }

  // Field name: xrpattern
  {
    cdr << ros_message->xrpattern;
  }

  // Field name: yrpattern
  {
    cdr << ros_message->yrpattern;
  }

  // Field name: zrpattern
  {
    cdr << ros_message->zrpattern;
  }

  // Field name: srpattern
  {
    cdr << ros_message->srpattern;
  }

  // Field name: trpattern
  {
    cdr << ros_message->trpattern;
  }

  return true;
}

static bool _IkPatternMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _IkPatternMsg__ros_msg_type * ros_message = static_cast<_IkPatternMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: xlpattern
  {
    cdr >> ros_message->xlpattern;
  }

  // Field name: ylpattern
  {
    cdr >> ros_message->ylpattern;
  }

  // Field name: zlpattern
  {
    cdr >> ros_message->zlpattern;
  }

  // Field name: slpattern
  {
    cdr >> ros_message->slpattern;
  }

  // Field name: tlpattern
  {
    cdr >> ros_message->tlpattern;
  }

  // Field name: xrpattern
  {
    cdr >> ros_message->xrpattern;
  }

  // Field name: yrpattern
  {
    cdr >> ros_message->yrpattern;
  }

  // Field name: zrpattern
  {
    cdr >> ros_message->zrpattern;
  }

  // Field name: srpattern
  {
    cdr >> ros_message->srpattern;
  }

  // Field name: trpattern
  {
    cdr >> ros_message->trpattern;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t get_serialized_size_humanoid_interfaces__msg__IkPatternMsg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _IkPatternMsg__ros_msg_type * ros_message = static_cast<const _IkPatternMsg__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name xlpattern
  {
    size_t item_size = sizeof(ros_message->xlpattern);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ylpattern
  {
    size_t item_size = sizeof(ros_message->ylpattern);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name zlpattern
  {
    size_t item_size = sizeof(ros_message->zlpattern);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name slpattern
  {
    size_t item_size = sizeof(ros_message->slpattern);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tlpattern
  {
    size_t item_size = sizeof(ros_message->tlpattern);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name xrpattern
  {
    size_t item_size = sizeof(ros_message->xrpattern);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name yrpattern
  {
    size_t item_size = sizeof(ros_message->yrpattern);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name zrpattern
  {
    size_t item_size = sizeof(ros_message->zrpattern);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name srpattern
  {
    size_t item_size = sizeof(ros_message->srpattern);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name trpattern
  {
    size_t item_size = sizeof(ros_message->trpattern);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _IkPatternMsg__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_humanoid_interfaces__msg__IkPatternMsg(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t max_serialized_size_humanoid_interfaces__msg__IkPatternMsg(
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

  // member: xlpattern
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ylpattern
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: zlpattern
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: slpattern
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: tlpattern
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: xrpattern
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: yrpattern
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: zrpattern
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: srpattern
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: trpattern
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
    using DataType = humanoid_interfaces__msg__IkPatternMsg;
    is_plain =
      (
      offsetof(DataType, trpattern) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _IkPatternMsg__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_humanoid_interfaces__msg__IkPatternMsg(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_IkPatternMsg = {
  "humanoid_interfaces::msg",
  "IkPatternMsg",
  _IkPatternMsg__cdr_serialize,
  _IkPatternMsg__cdr_deserialize,
  _IkPatternMsg__get_serialized_size,
  _IkPatternMsg__max_serialized_size
};

static rosidl_message_type_support_t _IkPatternMsg__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_IkPatternMsg,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, humanoid_interfaces, msg, IkPatternMsg)() {
  return &_IkPatternMsg__type_support;
}

#if defined(__cplusplus)
}
#endif
