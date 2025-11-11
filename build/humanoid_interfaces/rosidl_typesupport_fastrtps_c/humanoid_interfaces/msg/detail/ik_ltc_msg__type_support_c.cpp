// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from humanoid_interfaces:msg/IkLTCMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/ik_ltc_msg__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "humanoid_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "humanoid_interfaces/msg/detail/ik_ltc_msg__struct.h"
#include "humanoid_interfaces/msg/detail/ik_ltc_msg__functions.h"
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


using _IkLTCMsg__ros_msg_type = humanoid_interfaces__msg__IkLTCMsg;

static bool _IkLTCMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _IkLTCMsg__ros_msg_type * ros_message = static_cast<const _IkLTCMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: entire_time
  {
    cdr << ros_message->entire_time;
  }

  // Field name: swing_gain_l
  {
    cdr << ros_message->swing_gain_l;
  }

  // Field name: swing_gain_r
  {
    cdr << ros_message->swing_gain_r;
  }

  // Field name: warning
  {
    cdr << ros_message->warning;
  }

  // Field name: safe
  {
    cdr << ros_message->safe;
  }

  // Field name: landing_time_l
  {
    cdr << ros_message->landing_time_l;
  }

  // Field name: landing_time_r
  {
    cdr << ros_message->landing_time_r;
  }

  // Field name: landing_error_l
  {
    cdr << ros_message->landing_error_l;
  }

  // Field name: landing_error_r
  {
    cdr << ros_message->landing_error_r;
  }

  return true;
}

static bool _IkLTCMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _IkLTCMsg__ros_msg_type * ros_message = static_cast<_IkLTCMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: entire_time
  {
    cdr >> ros_message->entire_time;
  }

  // Field name: swing_gain_l
  {
    cdr >> ros_message->swing_gain_l;
  }

  // Field name: swing_gain_r
  {
    cdr >> ros_message->swing_gain_r;
  }

  // Field name: warning
  {
    cdr >> ros_message->warning;
  }

  // Field name: safe
  {
    cdr >> ros_message->safe;
  }

  // Field name: landing_time_l
  {
    cdr >> ros_message->landing_time_l;
  }

  // Field name: landing_time_r
  {
    cdr >> ros_message->landing_time_r;
  }

  // Field name: landing_error_l
  {
    cdr >> ros_message->landing_error_l;
  }

  // Field name: landing_error_r
  {
    cdr >> ros_message->landing_error_r;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t get_serialized_size_humanoid_interfaces__msg__IkLTCMsg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _IkLTCMsg__ros_msg_type * ros_message = static_cast<const _IkLTCMsg__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name entire_time
  {
    size_t item_size = sizeof(ros_message->entire_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name swing_gain_l
  {
    size_t item_size = sizeof(ros_message->swing_gain_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name swing_gain_r
  {
    size_t item_size = sizeof(ros_message->swing_gain_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name warning
  {
    size_t item_size = sizeof(ros_message->warning);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name safe
  {
    size_t item_size = sizeof(ros_message->safe);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name landing_time_l
  {
    size_t item_size = sizeof(ros_message->landing_time_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name landing_time_r
  {
    size_t item_size = sizeof(ros_message->landing_time_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name landing_error_l
  {
    size_t item_size = sizeof(ros_message->landing_error_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name landing_error_r
  {
    size_t item_size = sizeof(ros_message->landing_error_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _IkLTCMsg__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_humanoid_interfaces__msg__IkLTCMsg(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t max_serialized_size_humanoid_interfaces__msg__IkLTCMsg(
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

  // member: entire_time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: swing_gain_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: swing_gain_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: warning
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: safe
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: landing_time_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: landing_time_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: landing_error_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: landing_error_r
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
    using DataType = humanoid_interfaces__msg__IkLTCMsg;
    is_plain =
      (
      offsetof(DataType, landing_error_r) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _IkLTCMsg__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_humanoid_interfaces__msg__IkLTCMsg(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_IkLTCMsg = {
  "humanoid_interfaces::msg",
  "IkLTCMsg",
  _IkLTCMsg__cdr_serialize,
  _IkLTCMsg__cdr_deserialize,
  _IkLTCMsg__get_serialized_size,
  _IkLTCMsg__max_serialized_size
};

static rosidl_message_type_support_t _IkLTCMsg__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_IkLTCMsg,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, humanoid_interfaces, msg, IkLTCMsg)() {
  return &_IkLTCMsg__type_support;
}

#if defined(__cplusplus)
}
#endif
