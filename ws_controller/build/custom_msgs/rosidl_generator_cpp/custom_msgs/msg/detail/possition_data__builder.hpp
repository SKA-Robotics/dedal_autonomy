// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/PossitionData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__POSSITION_DATA__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__POSSITION_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/possition_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_PossitionData_z
{
public:
  explicit Init_PossitionData_z(::custom_msgs::msg::PossitionData & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::PossitionData z(::custom_msgs::msg::PossitionData::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::PossitionData msg_;
};

class Init_PossitionData_y
{
public:
  explicit Init_PossitionData_y(::custom_msgs::msg::PossitionData & msg)
  : msg_(msg)
  {}
  Init_PossitionData_z y(::custom_msgs::msg::PossitionData::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_PossitionData_z(msg_);
  }

private:
  ::custom_msgs::msg::PossitionData msg_;
};

class Init_PossitionData_x
{
public:
  Init_PossitionData_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PossitionData_y x(::custom_msgs::msg::PossitionData::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_PossitionData_y(msg_);
  }

private:
  ::custom_msgs::msg::PossitionData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::PossitionData>()
{
  return custom_msgs::msg::builder::Init_PossitionData_x();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__POSSITION_DATA__BUILDER_HPP_
