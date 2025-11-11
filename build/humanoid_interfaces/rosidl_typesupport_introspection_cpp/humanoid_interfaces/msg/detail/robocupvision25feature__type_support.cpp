// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from humanoid_interfaces:msg/Robocupvision25feature.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "humanoid_interfaces/msg/detail/robocupvision25feature__struct.hpp"
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

void Robocupvision25feature_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) humanoid_interfaces::msg::Robocupvision25feature(_init);
}

void Robocupvision25feature_fini_function(void * message_memory)
{
  auto typed_message = static_cast<humanoid_interfaces::msg::Robocupvision25feature *>(message_memory);
  typed_message->~Robocupvision25feature();
}

size_t size_function__Robocupvision25feature__confidence(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Robocupvision25feature__confidence(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Robocupvision25feature__confidence(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Robocupvision25feature__confidence(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Robocupvision25feature__confidence(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Robocupvision25feature__confidence(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Robocupvision25feature__confidence(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Robocupvision25feature__confidence(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Robocupvision25feature__distance(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Robocupvision25feature__distance(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Robocupvision25feature__distance(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Robocupvision25feature__distance(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Robocupvision25feature__distance(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Robocupvision25feature__distance(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Robocupvision25feature__distance(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Robocupvision25feature__distance(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Robocupvision25feature__point_vec_x(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Robocupvision25feature__point_vec_x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Robocupvision25feature__point_vec_x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Robocupvision25feature__point_vec_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Robocupvision25feature__point_vec_x(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Robocupvision25feature__point_vec_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Robocupvision25feature__point_vec_x(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Robocupvision25feature__point_vec_x(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Robocupvision25feature__point_vec_y(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Robocupvision25feature__point_vec_y(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Robocupvision25feature__point_vec_y(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Robocupvision25feature__point_vec_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Robocupvision25feature__point_vec_y(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Robocupvision25feature__point_vec_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Robocupvision25feature__point_vec_y(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Robocupvision25feature__point_vec_y(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Robocupvision25feature_message_member_array[4] = {
  {
    "confidence",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces::msg::Robocupvision25feature, confidence),  // bytes offset in struct
    nullptr,  // default value
    size_function__Robocupvision25feature__confidence,  // size() function pointer
    get_const_function__Robocupvision25feature__confidence,  // get_const(index) function pointer
    get_function__Robocupvision25feature__confidence,  // get(index) function pointer
    fetch_function__Robocupvision25feature__confidence,  // fetch(index, &value) function pointer
    assign_function__Robocupvision25feature__confidence,  // assign(index, value) function pointer
    resize_function__Robocupvision25feature__confidence  // resize(index) function pointer
  },
  {
    "distance",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces::msg::Robocupvision25feature, distance),  // bytes offset in struct
    nullptr,  // default value
    size_function__Robocupvision25feature__distance,  // size() function pointer
    get_const_function__Robocupvision25feature__distance,  // get_const(index) function pointer
    get_function__Robocupvision25feature__distance,  // get(index) function pointer
    fetch_function__Robocupvision25feature__distance,  // fetch(index, &value) function pointer
    assign_function__Robocupvision25feature__distance,  // assign(index, value) function pointer
    resize_function__Robocupvision25feature__distance  // resize(index) function pointer
  },
  {
    "point_vec_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces::msg::Robocupvision25feature, point_vec_x),  // bytes offset in struct
    nullptr,  // default value
    size_function__Robocupvision25feature__point_vec_x,  // size() function pointer
    get_const_function__Robocupvision25feature__point_vec_x,  // get_const(index) function pointer
    get_function__Robocupvision25feature__point_vec_x,  // get(index) function pointer
    fetch_function__Robocupvision25feature__point_vec_x,  // fetch(index, &value) function pointer
    assign_function__Robocupvision25feature__point_vec_x,  // assign(index, value) function pointer
    resize_function__Robocupvision25feature__point_vec_x  // resize(index) function pointer
  },
  {
    "point_vec_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(humanoid_interfaces::msg::Robocupvision25feature, point_vec_y),  // bytes offset in struct
    nullptr,  // default value
    size_function__Robocupvision25feature__point_vec_y,  // size() function pointer
    get_const_function__Robocupvision25feature__point_vec_y,  // get_const(index) function pointer
    get_function__Robocupvision25feature__point_vec_y,  // get(index) function pointer
    fetch_function__Robocupvision25feature__point_vec_y,  // fetch(index, &value) function pointer
    assign_function__Robocupvision25feature__point_vec_y,  // assign(index, value) function pointer
    resize_function__Robocupvision25feature__point_vec_y  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Robocupvision25feature_message_members = {
  "humanoid_interfaces::msg",  // message namespace
  "Robocupvision25feature",  // message name
  4,  // number of fields
  sizeof(humanoid_interfaces::msg::Robocupvision25feature),
  Robocupvision25feature_message_member_array,  // message members
  Robocupvision25feature_init_function,  // function to initialize message memory (memory has to be allocated)
  Robocupvision25feature_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Robocupvision25feature_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Robocupvision25feature_message_members,
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
get_message_type_support_handle<humanoid_interfaces::msg::Robocupvision25feature>()
{
  return &::humanoid_interfaces::msg::rosidl_typesupport_introspection_cpp::Robocupvision25feature_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, humanoid_interfaces, msg, Robocupvision25feature)() {
  return &::humanoid_interfaces::msg::rosidl_typesupport_introspection_cpp::Robocupvision25feature_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
