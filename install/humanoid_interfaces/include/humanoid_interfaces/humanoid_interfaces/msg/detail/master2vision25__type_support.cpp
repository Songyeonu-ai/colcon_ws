// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from humanoid_interfaces:msg/Master2vision25.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "humanoid_interfaces/msg/detail/master2vision25__struct.hpp"
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

void Master2vision25_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) humanoid_interfaces::msg::Master2vision25(_init);
}

void Master2vision25_fini_function(void * message_memory)
{
  auto typed_message = static_cast<humanoid_interfaces::msg::Master2vision25 *>(message_memory);
  typed_message->~Master2vision25();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Master2vision25_message_member_array[3] = {
  {
    "scanmode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces::msg::Master2vision25, scanmode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pan",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces::msg::Master2vision25, pan),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "tilt",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces::msg::Master2vision25, tilt),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Master2vision25_message_members = {
  "humanoid_interfaces::msg",  // message namespace
  "Master2vision25",  // message name
  3,  // number of fields
  sizeof(humanoid_interfaces::msg::Master2vision25),
  Master2vision25_message_member_array,  // message members
  Master2vision25_init_function,  // function to initialize message memory (memory has to be allocated)
  Master2vision25_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Master2vision25_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Master2vision25_message_members,
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
get_message_type_support_handle<humanoid_interfaces::msg::Master2vision25>()
{
  return &::humanoid_interfaces::msg::rosidl_typesupport_introspection_cpp::Master2vision25_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, humanoid_interfaces, msg, Master2vision25)() {
  return &::humanoid_interfaces::msg::rosidl_typesupport_introspection_cpp::Master2vision25_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
