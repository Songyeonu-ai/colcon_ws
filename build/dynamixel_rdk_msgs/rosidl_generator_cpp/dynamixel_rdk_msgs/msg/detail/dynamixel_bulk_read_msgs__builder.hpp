// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dynamixel_rdk_msgs:msg/DynamixelBulkReadMsgs.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_BULK_READ_MSGS__BUILDER_HPP_
#define DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_BULK_READ_MSGS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dynamixel_rdk_msgs/msg/detail/dynamixel_bulk_read_msgs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dynamixel_rdk_msgs
{

namespace msg
{

namespace builder
{

class Init_DynamixelBulkReadMsgs_status_msgs
{
public:
  Init_DynamixelBulkReadMsgs_status_msgs()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs status_msgs(::dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs::_status_msgs_type arg)
  {
    msg_.status_msgs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamixel_rdk_msgs::msg::DynamixelBulkReadMsgs>()
{
  return dynamixel_rdk_msgs::msg::builder::Init_DynamixelBulkReadMsgs_status_msgs();
}

}  // namespace dynamixel_rdk_msgs

#endif  // DYNAMIXEL_RDK_MSGS__MSG__DETAIL__DYNAMIXEL_BULK_READ_MSGS__BUILDER_HPP_
