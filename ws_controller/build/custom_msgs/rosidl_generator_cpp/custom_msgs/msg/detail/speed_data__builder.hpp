// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/SpeedData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__SPEED_DATA__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__SPEED_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/speed_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_SpeedData_z
{
public:
  explicit Init_SpeedData_z(::custom_msgs::msg::SpeedData & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::SpeedData z(::custom_msgs::msg::SpeedData::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::SpeedData msg_;
};

class Init_SpeedData_y
{
public:
  explicit Init_SpeedData_y(::custom_msgs::msg::SpeedData & msg)
  : msg_(msg)
  {}
  Init_SpeedData_z y(::custom_msgs::msg::SpeedData::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_SpeedData_z(msg_);
  }

private:
  ::custom_msgs::msg::SpeedData msg_;
};

class Init_SpeedData_x
{
public:
  Init_SpeedData_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SpeedData_y x(::custom_msgs::msg::SpeedData::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_SpeedData_y(msg_);
  }

private:
  ::custom_msgs::msg::SpeedData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::SpeedData>()
{
  return custom_msgs::msg::builder::Init_SpeedData_x();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__SPEED_DATA__BUILDER_HPP_
