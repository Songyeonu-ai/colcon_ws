// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from humanoid_interfaces:msg/Master2IkMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/master2_ik_msg__rosidl_typesupport_fastrtps_cpp.hpp"
#include "humanoid_interfaces/msg/detail/master2_ik_msg__struct.hpp"

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
  const humanoid_interfaces::msg::Master2IkMsg & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: x_length
  cdr << ros_message.x_length;
  // Member: y_length
  cdr << ros_message.y_length;
  // Member: yaw
  cdr << ros_message.yaw;
  // Member: flag
  cdr << ros_message.flag;
  // Member: one_x_length
  cdr << ros_message.one_x_length;
  // Member: one_y_length
  cdr << ros_message.one_y_length;
  // Member: one_yaw
  cdr << ros_message.one_yaw;
  // Member: one_step_flag
  cdr << ros_message.one_step_flag;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  humanoid_interfaces::msg::Master2IkMsg & ros_message)
{
  // Member: x_length
  cdr >> ros_message.x_length;

  // Member: y_length
  cdr >> ros_message.y_length;

  // Member: yaw
  cdr >> ros_message.yaw;

  // Member: flag
  cdr >> ros_message.flag;

  // Member: one_x_length
  cdr >> ros_message.one_x_length;

  // Member: one_y_length
  cdr >> ros_message.one_y_length;

  // Member: one_yaw
  cdr >> ros_message.one_yaw;

  // Member: one_step_flag
  cdr >> ros_message.one_step_flag;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
get_serialized_size(
  const humanoid_interfaces::msg::Master2IkMsg & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: x_length
  {
    size_t item_size = sizeof(ros_message.x_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: y_length
  {
    size_t item_size = sizeof(ros_message.y_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: yaw
  {
    size_t item_size = sizeof(ros_message.yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: flag
  {
    size_t item_size = sizeof(ros_message.flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: one_x_length
  {
    size_t item_size = sizeof(ros_message.one_x_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: one_y_length
  {
    size_t item_size = sizeof(ros_message.one_y_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: one_yaw
  {
    size_t item_size = sizeof(ros_message.one_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: one_step_flag
  {
    size_t item_size = sizeof(ros_message.one_step_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
max_serialized_size_Master2IkMsg(
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


  // Member: x_length
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: y_length
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: yaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: one_x_length
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: one_y_length
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: one_yaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: one_step_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = humanoid_interfaces::msg::Master2IkMsg;
    is_plain =
      (
      offsetof(DataType, one_step_flag) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Master2IkMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const humanoid_interfaces::msg::Master2IkMsg *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Master2IkMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<humanoid_interfaces::msg::Master2IkMsg *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Master2IkMsg__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const humanoid_interfaces::msg::Master2IkMsg *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Master2IkMsg__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Master2IkMsg(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Master2IkMsg__callbacks = {
  "humanoid_interfaces::msg",
  "Master2IkMsg",
  _Master2IkMsg__cdr_serialize,
  _Master2IkMsg__cdr_deserialize,
  _Master2IkMsg__get_serialized_size,
  _Master2IkMsg__max_serialized_size
};

static rosidl_message_type_support_t _Master2IkMsg__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Master2IkMsg__callbacks,
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
get_message_type_support_handle<humanoid_interfaces::msg::Master2IkMsg>()
{
  return &humanoid_interfaces::msg::typesupport_fastrtps_cpp::_Master2IkMsg__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, humanoid_interfaces, msg, Master2IkMsg)() {
  return &humanoid_interfaces::msg::typesupport_fastrtps_cpp::_Master2IkMsg__handle;
}

#ifdef __cplusplus
}
#endif
