// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from humanoid_interfaces:msg/Udp2master.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/udp2master__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "humanoid_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "humanoid_interfaces/msg/detail/udp2master__struct.h"
#include "humanoid_interfaces/msg/detail/udp2master__functions.h"
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


using _Udp2master__ros_msg_type = humanoid_interfaces__msg__Udp2master;

static bool _Udp2master__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Udp2master__ros_msg_type * ros_message = static_cast<const _Udp2master__ros_msg_type *>(untyped_ros_message);
  // Field name: robotnum
  {
    cdr << ros_message->robotnum;
  }

  // Field name: robotcase
  {
    cdr << ros_message->robotcase;
  }

  // Field name: localx
  {
    cdr << ros_message->localx;
  }

  // Field name: localy
  {
    cdr << ros_message->localy;
  }

  // Field name: localyaw
  {
    cdr << ros_message->localyaw;
  }

  // Field name: ballx
  {
    cdr << ros_message->ballx;
  }

  // Field name: bally
  {
    cdr << ros_message->bally;
  }

  // Field name: balldist
  {
    cdr << ros_message->balldist;
  }

  // Field name: balltheta
  {
    cdr << ros_message->balltheta;
  }

  return true;
}

static bool _Udp2master__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Udp2master__ros_msg_type * ros_message = static_cast<_Udp2master__ros_msg_type *>(untyped_ros_message);
  // Field name: robotnum
  {
    cdr >> ros_message->robotnum;
  }

  // Field name: robotcase
  {
    cdr >> ros_message->robotcase;
  }

  // Field name: localx
  {
    cdr >> ros_message->localx;
  }

  // Field name: localy
  {
    cdr >> ros_message->localy;
  }

  // Field name: localyaw
  {
    cdr >> ros_message->localyaw;
  }

  // Field name: ballx
  {
    cdr >> ros_message->ballx;
  }

  // Field name: bally
  {
    cdr >> ros_message->bally;
  }

  // Field name: balldist
  {
    cdr >> ros_message->balldist;
  }

  // Field name: balltheta
  {
    cdr >> ros_message->balltheta;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t get_serialized_size_humanoid_interfaces__msg__Udp2master(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Udp2master__ros_msg_type * ros_message = static_cast<const _Udp2master__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name robotnum
  {
    size_t item_size = sizeof(ros_message->robotnum);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name robotcase
  {
    size_t item_size = sizeof(ros_message->robotcase);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name localx
  {
    size_t item_size = sizeof(ros_message->localx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name localy
  {
    size_t item_size = sizeof(ros_message->localy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name localyaw
  {
    size_t item_size = sizeof(ros_message->localyaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ballx
  {
    size_t item_size = sizeof(ros_message->ballx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bally
  {
    size_t item_size = sizeof(ros_message->bally);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balldist
  {
    size_t item_size = sizeof(ros_message->balldist);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name balltheta
  {
    size_t item_size = sizeof(ros_message->balltheta);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Udp2master__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_humanoid_interfaces__msg__Udp2master(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_humanoid_interfaces
size_t max_serialized_size_humanoid_interfaces__msg__Udp2master(
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

  // member: robotnum
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: robotcase
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: localx
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: localy
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: localyaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ballx
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: bally
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balldist
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: balltheta
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
    using DataType = humanoid_interfaces__msg__Udp2master;
    is_plain =
      (
      offsetof(DataType, balltheta) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Udp2master__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_humanoid_interfaces__msg__Udp2master(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Udp2master = {
  "humanoid_interfaces::msg",
  "Udp2master",
  _Udp2master__cdr_serialize,
  _Udp2master__cdr_deserialize,
  _Udp2master__get_serialized_size,
  _Udp2master__max_serialized_size
};

static rosidl_message_type_support_t _Udp2master__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Udp2master,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, humanoid_interfaces, msg, Udp2master)() {
  return &_Udp2master__type_support;
}

#if defined(__cplusplus)
}
#endif
