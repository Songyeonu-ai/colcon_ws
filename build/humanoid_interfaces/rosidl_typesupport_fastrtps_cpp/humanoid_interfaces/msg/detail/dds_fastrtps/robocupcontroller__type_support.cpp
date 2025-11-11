// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from humanoid_interfaces:msg/Robocupcontroller.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/robocupcontroller__rosidl_typesupport_fastrtps_cpp.hpp"
#include "humanoid_interfaces/msg/detail/robocupcontroller__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace humanoid_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
cdr_serialize(
  const humanoid_interfaces::msg::Robocupcontroller & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: robotnum
  cdr << ros_message.robotnum;
  // Member: state
  cdr << ros_message.state;
  // Member: readytime
  cdr << ros_message.readytime;
  // Member: penalty
  cdr << ros_message.penalty;
  // Member: firsthalf
  cdr << ros_message.firsthalf;
  // Member: firstside
  cdr << ros_message.firstside;
  // Member: kickoffteam
  cdr << ros_message.kickoffteam;
  // Member: secondstate
  cdr << ros_message.secondstate;
  // Member: secondinfo
  {
    cdr << ros_message.secondinfo;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  humanoid_interfaces::msg::Robocupcontroller & ros_message)
{
  // Member: robotnum
  cdr >> ros_message.robotnum;

  // Member: state
  cdr >> ros_message.state;

  // Member: readytime
  cdr >> ros_message.readytime;

  // Member: penalty
  cdr >> ros_message.penalty;

  // Member: firsthalf
  cdr >> ros_message.firsthalf;

  // Member: firstside
  cdr >> ros_message.firstside;

  // Member: kickoffteam
  cdr >> ros_message.kickoffteam;

  // Member: secondstate
  cdr >> ros_message.secondstate;

  // Member: secondinfo
  {
    cdr >> ros_message.secondinfo;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
get_serialized_size(
  const humanoid_interfaces::msg::Robocupcontroller & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: robotnum
  {
    size_t item_size = sizeof(ros_message.robotnum);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: state
  {
    size_t item_size = sizeof(ros_message.state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: readytime
  {
    size_t item_size = sizeof(ros_message.readytime);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: penalty
  {
    size_t item_size = sizeof(ros_message.penalty);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: firsthalf
  {
    size_t item_size = sizeof(ros_message.firsthalf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: firstside
  {
    size_t item_size = sizeof(ros_message.firstside);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kickoffteam
  {
    size_t item_size = sizeof(ros_message.kickoffteam);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: secondstate
  {
    size_t item_size = sizeof(ros_message.secondstate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: secondinfo
  {
    size_t array_size = ros_message.secondinfo.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.secondinfo[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
max_serialized_size_Robocupcontroller(
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


  // Member: robotnum
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: readytime
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: penalty
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: firsthalf
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: firstside
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: kickoffteam
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: secondstate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: secondinfo
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
    using DataType = humanoid_interfaces::msg::Robocupcontroller;
    is_plain =
      (
      offsetof(DataType, secondinfo) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Robocupcontroller__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const humanoid_interfaces::msg::Robocupcontroller *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Robocupcontroller__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<humanoid_interfaces::msg::Robocupcontroller *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Robocupcontroller__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const humanoid_interfaces::msg::Robocupcontroller *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Robocupcontroller__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Robocupcontroller(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Robocupcontroller__callbacks = {
  "humanoid_interfaces::msg",
  "Robocupcontroller",
  _Robocupcontroller__cdr_serialize,
  _Robocupcontroller__cdr_deserialize,
  _Robocupcontroller__get_serialized_size,
  _Robocupcontroller__max_serialized_size
};

static rosidl_message_type_support_t _Robocupcontroller__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Robocupcontroller__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace humanoid_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_humanoid_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<humanoid_interfaces::msg::Robocupcontroller>()
{
  return &humanoid_interfaces::msg::typesupport_fastrtps_cpp::_Robocupcontroller__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, humanoid_interfaces, msg, Robocupcontroller)() {
  return &humanoid_interfaces::msg::typesupport_fastrtps_cpp::_Robocupcontroller__handle;
}

#ifdef __cplusplus
}
#endif
