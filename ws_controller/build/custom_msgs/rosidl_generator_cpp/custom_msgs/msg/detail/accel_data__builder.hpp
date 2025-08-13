// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/AccelData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__ACCEL_DATA__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__ACCEL_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/accel_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_AccelData_z
{
public:
  explicit Init_AccelData_z(::custom_msgs::msg::AccelData & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::AccelData z(::custom_msgs::msg::AccelData::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::AccelData msg_;
};

class Init_AccelData_y
{
public:
  explicit Init_AccelData_y(::custom_msgs::msg::AccelData & msg)
  : msg_(msg)
  {}
  Init_AccelData_z y(::custom_msgs::msg::AccelData::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_AccelData_z(msg_);
  }

private:
  ::custom_msgs::msg::AccelData msg_;
};

class Init_AccelData_x
{
public:
  Init_AccelData_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AccelData_y x(::custom_msgs::msg::AccelData::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_AccelData_y(msg_);
  }

private:
  ::custom_msgs::msg::AccelData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::AccelData>()
{
  return custom_msgs::msg::builder::Init_AccelData_x();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__ACCEL_DATA__BUILDER_HPP_
