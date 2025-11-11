// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from humanoid_interfaces:msg/Udp2master.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/udp2master__rosidl_typesupport_fastrtps_cpp.hpp"
#include "humanoid_interfaces/msg/detail/udp2master__struct.hpp"

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
  const humanoid_interfaces::msg::Udp2master & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: robotnum
  cdr << ros_message.robotnum;
  // Member: robotcase
  cdr << ros_message.robotcase;
  // Member: localx
  cdr << ros_message.localx;
  // Member: localy
  cdr << ros_message.localy;
  // Member: localyaw
  cdr << ros_message.localyaw;
  // Member: ballx
  cdr << ros_message.ballx;
  // Member: bally
  cdr << ros_message.bally;
  // Member: balldist
  cdr << ros_message.balldist;
  // Member: balltheta
  cdr << ros_message.balltheta;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  humanoid_interfaces::msg::Udp2master & ros_message)
{
  // Member: robotnum
  cdr >> ros_message.robotnum;

  // Member: robotcase
  cdr >> ros_message.robotcase;

  // Member: localx
  cdr >> ros_message.localx;

  // Member: localy
  cdr >> ros_message.localy;

  // Member: localyaw
  cdr >> ros_message.localyaw;

  // Member: ballx
  cdr >> ros_message.ballx;

  // Member: bally
  cdr >> ros_message.bally;

  // Member: balldist
  cdr >> ros_message.balldist;

  // Member: balltheta
  cdr >> ros_message.balltheta;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
get_serialized_size(
  const humanoid_interfaces::msg::Udp2master & ros_message,
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
  // Member: robotcase
  {
    size_t item_size = sizeof(ros_message.robotcase);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: localx
  {
    size_t item_size = sizeof(ros_message.localx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: localy
  {
    size_t item_size = sizeof(ros_message.localy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: localyaw
  {
    size_t item_size = sizeof(ros_message.localyaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ballx
  {
    size_t item_size = sizeof(ros_message.ballx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: bally
  {
    size_t item_size = sizeof(ros_message.bally);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balldist
  {
    size_t item_size = sizeof(ros_message.balldist);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: balltheta
  {
    size_t item_size = sizeof(ros_message.balltheta);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
max_serialized_size_Udp2master(
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

  // Member: robotcase
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: localx
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: localy
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: localyaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ballx
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: bally
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balldist
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: balltheta
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
    using DataType = humanoid_interfaces::msg::Udp2master;
    is_plain =
      (
      offsetof(DataType, balltheta) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Udp2master__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const humanoid_interfaces::msg::Udp2master *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Udp2master__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<humanoid_interfaces::msg::Udp2master *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Udp2master__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const humanoid_interfaces::msg::Udp2master *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Udp2master__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Udp2master(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Udp2master__callbacks = {
  "humanoid_interfaces::msg",
  "Udp2master",
  _Udp2master__cdr_serialize,
  _Udp2master__cdr_deserialize,
  _Udp2master__get_serialized_size,
  _Udp2master__max_serialized_size
};

static rosidl_message_type_support_t _Udp2master__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Udp2master__callbacks,
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
get_message_type_support_handle<humanoid_interfaces::msg::Udp2master>()
{
  return &humanoid_interfaces::msg::typesupport_fastrtps_cpp::_Udp2master__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, humanoid_interfaces, msg, Udp2master)() {
  return &humanoid_interfaces::msg::typesupport_fastrtps_cpp::_Udp2master__handle;
}

#ifdef __cplusplus
}
#endif
