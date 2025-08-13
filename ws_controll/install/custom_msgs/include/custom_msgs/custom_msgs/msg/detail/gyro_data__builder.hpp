// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/GyroData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__GYRO_DATA__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__GYRO_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/gyro_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_GyroData_z
{
public:
  explicit Init_GyroData_z(::custom_msgs::msg::GyroData & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::GyroData z(::custom_msgs::msg::GyroData::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::GyroData msg_;
};

class Init_GyroData_y
{
public:
  explicit Init_GyroData_y(::custom_msgs::msg::GyroData & msg)
  : msg_(msg)
  {}
  Init_GyroData_z y(::custom_msgs::msg::GyroData::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_GyroData_z(msg_);
  }

private:
  ::custom_msgs::msg::GyroData msg_;
};

class Init_GyroData_x
{
public:
  Init_GyroData_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GyroData_y x(::custom_msgs::msg::GyroData::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_GyroData_y(msg_);
  }

private:
  ::custom_msgs::msg::GyroData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::GyroData>()
{
  return custom_msgs::msg::builder::Init_GyroData_x();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__GYRO_DATA__BUILDER_HPP_
