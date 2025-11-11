// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from humanoid_interfaces:msg/ZmpMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/zmp_msg__rosidl_typesupport_fastrtps_cpp.hpp"
#include "humanoid_interfaces/msg/detail/zmp_msg__struct.hpp"

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
  const humanoid_interfaces::msg::ZmpMsg & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: left_x_zmp
  cdr << ros_message.left_x_zmp;
  // Member: left_y_zmp
  cdr << ros_message.left_y_zmp;
  // Member: right_x_zmp
  cdr << ros_message.right_x_zmp;
  // Member: right_y_zmp
  cdr << ros_message.right_y_zmp;
  // Member: total_x_zmp
  cdr << ros_message.total_x_zmp;
  // Member: total_y_zmp
  cdr << ros_message.total_y_zmp;
  // Member: left_foot
  cdr << (ros_message.left_foot ? true : false);
  // Member: right_foot
  cdr << (ros_message.right_foot ? true : false);
  // Member: both_feet
  cdr << (ros_message.both_feet ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  humanoid_interfaces::msg::ZmpMsg & ros_message)
{
  // Member: left_x_zmp
  cdr >> ros_message.left_x_zmp;

  // Member: left_y_zmp
  cdr >> ros_message.left_y_zmp;

  // Member: right_x_zmp
  cdr >> ros_message.right_x_zmp;

  // Member: right_y_zmp
  cdr >> ros_message.right_y_zmp;

  // Member: total_x_zmp
  cdr >> ros_message.total_x_zmp;

  // Member: total_y_zmp
  cdr >> ros_message.total_y_zmp;

  // Member: left_foot
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.left_foot = tmp ? true : false;
  }

  // Member: right_foot
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.right_foot = tmp ? true : false;
  }

  // Member: both_feet
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.both_feet = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
get_serialized_size(
  const humanoid_interfaces::msg::ZmpMsg & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: left_x_zmp
  {
    size_t item_size = sizeof(ros_message.left_x_zmp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_y_zmp
  {
    size_t item_size = sizeof(ros_message.left_y_zmp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_x_zmp
  {
    size_t item_size = sizeof(ros_message.right_x_zmp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_y_zmp
  {
    size_t item_size = sizeof(ros_message.right_y_zmp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: total_x_zmp
  {
    size_t item_size = sizeof(ros_message.total_x_zmp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: total_y_zmp
  {
    size_t item_size = sizeof(ros_message.total_y_zmp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_foot
  {
    size_t item_size = sizeof(ros_message.left_foot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_foot
  {
    size_t item_size = sizeof(ros_message.right_foot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: both_feet
  {
    size_t item_size = sizeof(ros_message.both_feet);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
max_serialized_size_ZmpMsg(
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


  // Member: left_x_zmp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: left_y_zmp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: right_x_zmp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: right_y_zmp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: total_x_zmp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: total_y_zmp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: left_foot
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: right_foot
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: both_feet
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
    using DataType = humanoid_interfaces::msg::ZmpMsg;
    is_plain =
      (
      offsetof(DataType, both_feet) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ZmpMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const humanoid_interfaces::msg::ZmpMsg *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ZmpMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<humanoid_interfaces::msg::ZmpMsg *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ZmpMsg__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const humanoid_interfaces::msg::ZmpMsg *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ZmpMsg__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ZmpMsg(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ZmpMsg__callbacks = {
  "humanoid_interfaces::msg",
  "ZmpMsg",
  _ZmpMsg__cdr_serialize,
  _ZmpMsg__cdr_deserialize,
  _ZmpMsg__get_serialized_size,
  _ZmpMsg__max_serialized_size
};

static rosidl_message_type_support_t _ZmpMsg__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ZmpMsg__callbacks,
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
get_message_type_support_handle<humanoid_interfaces::msg::ZmpMsg>()
{
  return &humanoid_interfaces::msg::typesupport_fastrtps_cpp::_ZmpMsg__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, humanoid_interfaces, msg, ZmpMsg)() {
  return &humanoid_interfaces::msg::typesupport_fastrtps_cpp::_ZmpMsg__handle;
}

#ifdef __cplusplus
}
#endif
