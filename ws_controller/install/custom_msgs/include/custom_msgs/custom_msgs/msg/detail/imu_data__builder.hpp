// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/ImuData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__IMU_DATA__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__IMU_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/imu_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_ImuData_gyro
{
public:
  explicit Init_ImuData_gyro(::custom_msgs::msg::ImuData & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::ImuData gyro(::custom_msgs::msg::ImuData::_gyro_type arg)
  {
    msg_.gyro = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::ImuData msg_;
};

class Init_ImuData_accel
{
public:
  explicit Init_ImuData_accel(::custom_msgs::msg::ImuData & msg)
  : msg_(msg)
  {}
  Init_ImuData_gyro accel(::custom_msgs::msg::ImuData::_accel_type arg)
  {
    msg_.accel = std::move(arg);
    return Init_ImuData_gyro(msg_);
  }

private:
  ::custom_msgs::msg::ImuData msg_;
};

class Init_ImuData_timestamp
{
public:
  Init_ImuData_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ImuData_accel timestamp(::custom_msgs::msg::ImuData::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_ImuData_accel(msg_);
  }

private:
  ::custom_msgs::msg::ImuData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::ImuData>()
{
  return custom_msgs::msg::builder::Init_ImuData_timestamp();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__IMU_DATA__BUILDER_HPP_
