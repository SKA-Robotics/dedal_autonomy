// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/DataXYZ.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__DATA_XYZ__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__DATA_XYZ__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/data_xyz__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_DataXYZ_z
{
public:
  explicit Init_DataXYZ_z(::custom_msgs::msg::DataXYZ & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::DataXYZ z(::custom_msgs::msg::DataXYZ::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::DataXYZ msg_;
};

class Init_DataXYZ_y
{
public:
  explicit Init_DataXYZ_y(::custom_msgs::msg::DataXYZ & msg)
  : msg_(msg)
  {}
  Init_DataXYZ_z y(::custom_msgs::msg::DataXYZ::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_DataXYZ_z(msg_);
  }

private:
  ::custom_msgs::msg::DataXYZ msg_;
};

class Init_DataXYZ_x
{
public:
  Init_DataXYZ_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DataXYZ_y x(::custom_msgs::msg::DataXYZ::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_DataXYZ_y(msg_);
  }

private:
  ::custom_msgs::msg::DataXYZ msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::DataXYZ>()
{
  return custom_msgs::msg::builder::Init_DataXYZ_x();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__DATA_XYZ__BUILDER_HPP_
