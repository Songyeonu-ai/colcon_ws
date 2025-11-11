// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from humanoid_interfaces:msg/ZmpMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/zmp_msg__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "humanoid_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "humanoid_interfaces/msg/detail/zmp_msg__struct.h"
#include "humanoid_interfaces/msg/detail/zmp_msg__functions.h"
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


using _ZmpMsg__ros_msg_type = humanoid_interfaces__msg__ZmpMsg;

static bool _ZmpMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ZmpMsg__ros_msg_type * ros_message = static_cast<const _ZmpMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: left_x_zmp
  {
    cdr << ros_message->left_x_zmp;
  }

  // Field name: left_y_zmp
  {
    cdr << ros_message->left_y_zmp;
  }

  // Field name: right_x_zmp
  {
    cdr << ros_message->right_x_zmp;
  }

  // Field name: right_y_zmp
  {
    cdr << ros_message->right_y_zmp;
  }

  // Field name: total_x_zmp
  {
    cdr << ros_message->total_x_zmp;
  }

  // Field name: total_y_zmp
  {
    cdr << ros_message->total_y_zmp;
  }

  // Field name: left_foot
  {
    cdr << (ros_message->left_foot ? true : false);
  }

  // Field name: right_foot
  {
    cdr << (ros_message->right_foot ? true : false);
  }

  // Field name: both_feet
  {
    cdr << (ros_message->both_feet ? true : false);
  }

  return true;
}

static bool _ZmpMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ZmpMsg__ros_msg_type * ros_message = static_cast<_ZmpMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: left_x_zmp
  {
    cdr >> ros_message->left_x_zmp;
  }

  // Field name: left_y_zmp
  {
    cdr >> ros_message->left_y_zmp;
  }

  // Field name: right_x_zmp
  {
    cdr >> ros_message->right_x_zmp;
  }

  // Field name: right_y_zmp
  {
    cdr >> ros_message->right_y_zmp;
  }

  // Field name: total_x_zmp
  {
    cdr >> ros_message->total_x_zmp;
  }

  // Field name: total_y_zmp
  {
    cdr >> ros_message->total_y_zmp;
  }

  // Field name: left_foot
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->left_foot = tmp ? true : false;
  }

  // Field name: right_foot
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->right_foot = tmp ? true : false;
  }

  // Field name: both_feet
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->both_feet = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t get_serialized_size_humanoid_interfaces__msg__ZmpMsg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ZmpMsg__ros_msg_type * ros_message = static_cast<const _ZmpMsg__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name left_x_zmp
  {
    size_t item_size = sizeof(ros_message->left_x_zmp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name left_y_zmp
  {
    size_t item_size = sizeof(ros_message->left_y_zmp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name right_x_zmp
  {
    size_t item_size = sizeof(ros_message->right_x_zmp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name right_y_zmp
  {
    size_t item_size = sizeof(ros_message->right_y_zmp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name total_x_zmp
  {
    size_t item_size = sizeof(ros_message->total_x_zmp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name total_y_zmp
  {
    size_t item_size = sizeof(ros_message->total_y_zmp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name left_foot
  {
    size_t item_size = sizeof(ros_message->left_foot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name right_foot
  {
    size_t item_size = sizeof(ros_message->right_foot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name both_feet
  {
    size_t item_size = sizeof(ros_message->both_feet);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ZmpMsg__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_humanoid_interfaces__msg__ZmpMsg(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t max_serialized_size_humanoid_interfaces__msg__ZmpMsg(
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

  // member: left_x_zmp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: left_y_zmp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: right_x_zmp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: right_y_zmp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: total_x_zmp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: total_y_zmp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: left_foot
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: right_foot
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: both_feet
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = humanoid_interfaces__msg__ZmpMsg;
    is_plain =
      (
      offsetof(DataType, both_feet) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ZmpMsg__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_humanoid_interfaces__msg__ZmpMsg(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ZmpMsg = {
  "humanoid_interfaces::msg",
  "ZmpMsg",
  _ZmpMsg__cdr_serialize,
  _ZmpMsg__cdr_deserialize,
  _ZmpMsg__get_serialized_size,
  _ZmpMsg__max_serialized_size
};

static rosidl_message_type_support_t _ZmpMsg__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ZmpMsg,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, humanoid_interfaces, msg, ZmpMsg)() {
  return &_ZmpMsg__type_support;
}

#if defined(__cplusplus)
}
#endif
