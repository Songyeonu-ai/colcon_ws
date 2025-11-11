// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from dynamixel_rdk_msgs:msg/DynamixelControlMsgs.idl
// generated code does not contain a copyright notice
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_control_msgs__rosidl_typesupport_fastrtps_cpp.hpp"
#include "dynamixel_rdk_msgs/msg/detail/dynamixel_control_msgs__struct.hpp"

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
namespace dynamixel_rdk_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const dynamixel_rdk_msgs::msg::DynamixelMsgs &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  dynamixel_rdk_msgs::msg::DynamixelMsgs &);
size_t get_serialized_size(
  const dynamixel_rdk_msgs::msg::DynamixelMsgs &,
  size_t current_alignment);
size_t
max_serialized_size_DynamixelMsgs(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace dynamixel_rdk_msgs


namespace dynamixel_rdk_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dynamixel_rdk_msgs
cdr_serialize(
  const dynamixel_rdk_msgs::msg::DynamixelControlMsgs & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: motor_control
  {
    size_t size = ros_message.motor_control.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      dynamixel_rdk_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.motor_control[i],
        cdr);
    }
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dynamixel_rdk_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  dynamixel_rdk_msgs::msg::DynamixelControlMsgs & ros_message)
{
  // Member: motor_control
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.motor_control.resize(size);
    for (size_t i = 0; i < size; i++) {
      dynamixel_rdk_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.motor_control[i]);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dynamixel_rdk_msgs
get_serialized_size(
  const dynamixel_rdk_msgs::msg::DynamixelControlMsgs & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: motor_control
  {
    size_t array_size = ros_message.motor_control.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        dynamixel_rdk_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.motor_control[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dynamixel_rdk_msgs
max_serialized_size_DynamixelControlMsgs(
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


  // Member: motor_control
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        dynamixel_rdk_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_DynamixelMsgs(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dynamixel_rdk_msgs::msg::DynamixelControlMsgs;
    is_plain =
      (
      offsetof(DataType, motor_control) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _DynamixelControlMsgs__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const dynamixel_rdk_msgs::msg::DynamixelControlMsgs *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _DynamixelControlMsgs__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<dynamixel_rdk_msgs::msg::DynamixelControlMsgs *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _DynamixelControlMsgs__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const dynamixel_rdk_msgs::msg::DynamixelControlMsgs *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _DynamixelControlMsgs__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_DynamixelControlMsgs(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _DynamixelControlMsgs__callbacks = {
  "dynamixel_rdk_msgs::msg",
  "DynamixelControlMsgs",
  _DynamixelControlMsgs__cdr_serialize,
  _DynamixelControlMsgs__cdr_deserialize,
  _DynamixelControlMsgs__get_serialized_size,
  _DynamixelControlMsgs__max_serialized_size
};

static rosidl_message_type_support_t _DynamixelControlMsgs__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_DynamixelControlMsgs__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace dynamixel_rdk_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_dynamixel_rdk_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<dynamixel_rdk_msgs::msg::DynamixelControlMsgs>()
{
  return &dynamixel_rdk_msgs::msg::typesupport_fastrtps_cpp::_DynamixelControlMsgs__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dynamixel_rdk_msgs, msg, DynamixelControlMsgs)() {
  return &dynamixel_rdk_msgs::msg::typesupport_fastrtps_cpp::_DynamixelControlMsgs__handle;
}

#ifdef __cplusplus
}
#endif
