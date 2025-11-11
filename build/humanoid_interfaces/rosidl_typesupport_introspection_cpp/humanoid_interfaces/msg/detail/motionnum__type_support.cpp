// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from humanoid_interfaces:msg/Motionnum.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "humanoid_interfaces/msg/detail/motionnum__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace humanoid_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Motionnum_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) humanoid_interfaces::msg::Motionnum(_init);
}

void Motionnum_fini_function(void * message_memory)
{
  auto typed_message = static_cast<humanoid_interfaces::msg::Motionnum *>(message_memory);
  typed_message->~Motionnum();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Motionnum_message_member_array[2] = {
  {
    "motionmode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces::msg::Motionnum, motionmode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "motionnum",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces::msg::Motionnum, motionnum),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Motionnum_message_members = {
  "humanoid_interfaces::msg",  // message namespace
  "Motionnum",  // message name
  2,  // number of fields
  sizeof(humanoid_interfaces::msg::Motionnum),
  Motionnum_message_member_array,  // message members
  Motionnum_init_function,  // function to initialize message memory (memory has to be allocated)
  Motionnum_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Motionnum_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Motionnum_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace humanoid_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<humanoid_interfaces::msg::Motionnum>()
{
  return &::humanoid_interfaces::msg::rosidl_typesupport_introspection_cpp::Motionnum_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, humanoid_interfaces, msg, Motionnum)() {
  return &::humanoid_interfaces::msg::rosidl_typesupport_introspection_cpp::Motionnum_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
