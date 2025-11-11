// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from humanoid_interfaces:msg/Robocuplocalization25.idl
// generated code does not contain a copyright notice
#include "humanoid_interfaces/msg/detail/robocuplocalization25__rosidl_typesupport_fastrtps_cpp.hpp"
#include "humanoid_interfaces/msg/detail/robocuplocalization25__struct.hpp"

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
  const humanoid_interfaces::msg::Robocuplocalization25 & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: ball_x
  cdr << ros_message.ball_x;
  // Member: ball_y
  cdr << ros_message.ball_y;
  // Member: ball_speed_x
  cdr << ros_message.ball_speed_x;
  // Member: ball_speed_y
  cdr << ros_message.ball_speed_y;
  // Member: robot_x
  cdr << ros_message.robot_x;
  // Member: robot_y
  cdr << ros_message.robot_y;
  // Member: obstacle_x
  cdr << ros_message.obstacle_x;
  // Member: obstacle_y
  cdr << ros_message.obstacle_y;
  // Member: obstacle0_x
  cdr << ros_message.obstacle0_x;
  // Member: obstacle0_y
  cdr << ros_message.obstacle0_y;
  // Member: obstacle1_x
  cdr << ros_message.obstacle1_x;
  // Member: obstacle1_y
  cdr << ros_message.obstacle1_y;
  // Member: obstacle2_x
  cdr << ros_message.obstacle2_x;
  // Member: obstacle2_y
  cdr << ros_message.obstacle2_y;
  // Member: obstacle3_x
  cdr << ros_message.obstacle3_x;
  // Member: obstacle3_y
  cdr << ros_message.obstacle3_y;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  humanoid_interfaces::msg::Robocuplocalization25 & ros_message)
{
  // Member: ball_x
  cdr >> ros_message.ball_x;

  // Member: ball_y
  cdr >> ros_message.ball_y;

  // Member: ball_speed_x
  cdr >> ros_message.ball_speed_x;

  // Member: ball_speed_y
  cdr >> ros_message.ball_speed_y;

  // Member: robot_x
  cdr >> ros_message.robot_x;

  // Member: robot_y
  cdr >> ros_message.robot_y;

  // Member: obstacle_x
  cdr >> ros_message.obstacle_x;

  // Member: obstacle_y
  cdr >> ros_message.obstacle_y;

  // Member: obstacle0_x
  cdr >> ros_message.obstacle0_x;

  // Member: obstacle0_y
  cdr >> ros_message.obstacle0_y;

  // Member: obstacle1_x
  cdr >> ros_message.obstacle1_x;

  // Member: obstacle1_y
  cdr >> ros_message.obstacle1_y;

  // Member: obstacle2_x
  cdr >> ros_message.obstacle2_x;

  // Member: obstacle2_y
  cdr >> ros_message.obstacle2_y;

  // Member: obstacle3_x
  cdr >> ros_message.obstacle3_x;

  // Member: obstacle3_y
  cdr >> ros_message.obstacle3_y;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
get_serialized_size(
  const humanoid_interfaces::msg::Robocuplocalization25 & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: ball_x
  {
    size_t item_size = sizeof(ros_message.ball_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ball_y
  {
    size_t item_size = sizeof(ros_message.ball_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ball_speed_x
  {
    size_t item_size = sizeof(ros_message.ball_speed_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ball_speed_y
  {
    size_t item_size = sizeof(ros_message.ball_speed_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: robot_x
  {
    size_t item_size = sizeof(ros_message.robot_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: robot_y
  {
    size_t item_size = sizeof(ros_message.robot_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: obstacle_x
  {
    size_t item_size = sizeof(ros_message.obstacle_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: obstacle_y
  {
    size_t item_size = sizeof(ros_message.obstacle_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: obstacle0_x
  {
    size_t item_size = sizeof(ros_message.obstacle0_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: obstacle0_y
  {
    size_t item_size = sizeof(ros_message.obstacle0_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: obstacle1_x
  {
    size_t item_size = sizeof(ros_message.obstacle1_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: obstacle1_y
  {
    size_t item_size = sizeof(ros_message.obstacle1_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: obstacle2_x
  {
    size_t item_size = sizeof(ros_message.obstacle2_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: obstacle2_y
  {
    size_t item_size = sizeof(ros_message.obstacle2_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: obstacle3_x
  {
    size_t item_size = sizeof(ros_message.obstacle3_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: obstacle3_y
  {
    size_t item_size = sizeof(ros_message.obstacle3_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_humanoid_interfaces
max_serialized_size_Robocuplocalization25(
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


  // Member: ball_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ball_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ball_speed_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ball_speed_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: robot_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: robot_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: obstacle_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: obstacle_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: obstacle0_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: obstacle0_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: obstacle1_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: obstacle1_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: obstacle2_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: obstacle2_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: obstacle3_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: obstacle3_y
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
    using DataType = humanoid_interfaces::msg::Robocuplocalization25;
    is_plain =
      (
      offsetof(DataType, obstacle3_y) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Robocuplocalization25__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const humanoid_interfaces::msg::Robocuplocalization25 *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Robocuplocalization25__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<humanoid_interfaces::msg::Robocuplocalization25 *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Robocuplocalization25__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const humanoid_interfaces::msg::Robocuplocalization25 *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Robocuplocalization25__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Robocuplocalization25(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Robocuplocalization25__callbacks = {
  "humanoid_interfaces::msg",
  "Robocuplocalization25",
  _Robocuplocalization25__cdr_serialize,
  _Robocuplocalization25__cdr_deserialize,
  _Robocuplocalization25__get_serialized_size,
  _Robocuplocalization25__max_serialized_size
};

static rosidl_message_type_support_t _Robocuplocalization25__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Robocuplocalization25__callbacks,
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
get_message_type_support_handle<humanoid_interfaces::msg::Robocuplocalization25>()
{
  return &humanoid_interfaces::msg::typesupport_fastrtps_cpp::_Robocuplocalization25__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, humanoid_interfaces, msg, Robocuplocalization25)() {
  return &humanoid_interfaces::msg::typesupport_fastrtps_cpp::_Robocuplocalization25__handle;
}

#ifdef __cplusplus
}
#endif
