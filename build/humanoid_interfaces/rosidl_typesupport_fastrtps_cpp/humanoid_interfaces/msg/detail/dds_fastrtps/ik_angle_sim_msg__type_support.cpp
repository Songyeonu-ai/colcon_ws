// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from humanoid_interfaces:msg/IkAngleSimMsg.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/ik_angle_sim_msg__rosidl_typesupport_fastrtps_cpp.hpp"
#include "humanoid_interfaces/msg/detail/ik_angle_sim_msg__struct.hpp"

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
  const humanoid_interfaces::msg::IkAngleSimMsg & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: ang_l_0
  cdr << ros_message.ang_l_0;
  // Member: ang_l_1
  cdr << ros_message.ang_l_1;
  // Member: ang_l_2
  cdr << ros_message.ang_l_2;
  // Member: ang_l_3
  cdr << ros_message.ang_l_3;
  // Member: ang_l_4
  cdr << ros_message.ang_l_4;
  // Member: ang_l_5
  cdr << ros_message.ang_l_5;
  // Member: ang_l_6
  cdr << ros_message.ang_l_6;
  // Member: ang_l_7
  cdr << ros_message.ang_l_7;
  // Member: ang_l_8
  cdr << ros_message.ang_l_8;
  // Member: ang_r_0
  cdr << ros_message.ang_r_0;
  // Member: ang_r_1
  cdr << ros_message.ang_r_1;
  // Member: ang_r_2
  cdr << ros_message.ang_r_2;
  // Member: ang_r_3
  cdr << ros_message.ang_r_3;
  // Member: ang_r_4
  cdr << ros_message.ang_r_4;
  // Member: ang_r_5
  cdr << ros_message.ang_r_5;
  // Member: ang_r_6
  cdr << ros_message.ang_r_6;
  // Member: ang_r_7
  cdr << ros_message.ang_r_7;
  // Member: ang_r_8
  cdr << ros_message.ang_r_8;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  humanoid_interfaces::msg::IkAngleSimMsg & ros_message)
{
  // Member: ang_l_0
  cdr >> ros_message.ang_l_0;

  // Member: ang_l_1
  cdr >> ros_message.ang_l_1;

  // Member: ang_l_2
  cdr >> ros_message.ang_l_2;

  // Member: ang_l_3
  cdr >> ros_message.ang_l_3;

  // Member: ang_l_4
  cdr >> ros_message.ang_l_4;

  // Member: ang_l_5
  cdr >> ros_message.ang_l_5;

  // Member: ang_l_6
  cdr >> ros_message.ang_l_6;

  // Member: ang_l_7
  cdr >> ros_message.ang_l_7;

  // Member: ang_l_8
  cdr >> ros_message.ang_l_8;

  // Member: ang_r_0
  cdr >> ros_message.ang_r_0;

  // Member: ang_r_1
  cdr >> ros_message.ang_r_1;

  // Member: ang_r_2
  cdr >> ros_message.ang_r_2;

  // Member: ang_r_3
  cdr >> ros_message.ang_r_3;

  // Member: ang_r_4
  cdr >> ros_message.ang_r_4;

  // Member: ang_r_5
  cdr >> ros_message.ang_r_5;

  // Member: ang_r_6
  cdr >> ros_message.ang_r_6;

  // Member: ang_r_7
  cdr >> ros_message.ang_r_7;

  // Member: ang_r_8
  cdr >> ros_message.ang_r_8;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
get_serialized_size(
  const humanoid_interfaces::msg::IkAngleSimMsg & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: ang_l_0
  {
    size_t item_size = sizeof(ros_message.ang_l_0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_l_1
  {
    size_t item_size = sizeof(ros_message.ang_l_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_l_2
  {
    size_t item_size = sizeof(ros_message.ang_l_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_l_3
  {
    size_t item_size = sizeof(ros_message.ang_l_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_l_4
  {
    size_t item_size = sizeof(ros_message.ang_l_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_l_5
  {
    size_t item_size = sizeof(ros_message.ang_l_5);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_l_6
  {
    size_t item_size = sizeof(ros_message.ang_l_6);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_l_7
  {
    size_t item_size = sizeof(ros_message.ang_l_7);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_l_8
  {
    size_t item_size = sizeof(ros_message.ang_l_8);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_r_0
  {
    size_t item_size = sizeof(ros_message.ang_r_0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_r_1
  {
    size_t item_size = sizeof(ros_message.ang_r_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_r_2
  {
    size_t item_size = sizeof(ros_message.ang_r_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_r_3
  {
    size_t item_size = sizeof(ros_message.ang_r_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_r_4
  {
    size_t item_size = sizeof(ros_message.ang_r_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_r_5
  {
    size_t item_size = sizeof(ros_message.ang_r_5);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_r_6
  {
    size_t item_size = sizeof(ros_message.ang_r_6);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_r_7
  {
    size_t item_size = sizeof(ros_message.ang_r_7);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ang_r_8
  {
    size_t item_size = sizeof(ros_message.ang_r_8);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
max_serialized_size_IkAngleSimMsg(
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


  // Member: ang_l_0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_l_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_l_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_l_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_l_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_l_5
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_l_6
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_l_7
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_l_8
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_r_0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_r_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_r_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_r_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_r_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_r_5
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_r_6
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_r_7
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ang_r_8
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
    using DataType = humanoid_interfaces::msg::IkAngleSimMsg;
    is_plain =
      (
      offsetof(DataType, ang_r_8) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _IkAngleSimMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const humanoid_interfaces::msg::IkAngleSimMsg *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _IkAngleSimMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<humanoid_interfaces::msg::IkAngleSimMsg *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _IkAngleSimMsg__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const humanoid_interfaces::msg::IkAngleSimMsg *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _IkAngleSimMsg__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_IkAngleSimMsg(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _IkAngleSimMsg__callbacks = {
  "humanoid_interfaces::msg",
  "IkAngleSimMsg",
  _IkAngleSimMsg__cdr_serialize,
  _IkAngleSimMsg__cdr_deserialize,
  _IkAngleSimMsg__get_serialized_size,
  _IkAngleSimMsg__max_serialized_size
};

static rosidl_message_type_support_t _IkAngleSimMsg__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_IkAngleSimMsg__callbacks,
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
get_message_type_support_handle<humanoid_interfaces::msg::IkAngleSimMsg>()
{
  return &humanoid_interfaces::msg::typesupport_fastrtps_cpp::_IkAngleSimMsg__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, humanoid_interfaces, msg, IkAngleSimMsg)() {
  return &humanoid_interfaces::msg::typesupport_fastrtps_cpp::_IkAngleSimMsg__handle;
}

#ifdef __cplusplus
}
#endif
