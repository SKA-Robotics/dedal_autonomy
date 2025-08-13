// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/EstimatorData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__ESTIMATOR_DATA__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__ESTIMATOR_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/estimator_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_EstimatorData_raw_data
{
public:
  explicit Init_EstimatorData_raw_data(::custom_msgs::msg::EstimatorData & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::EstimatorData raw_data(::custom_msgs::msg::EstimatorData::_raw_data_type arg)
  {
    msg_.raw_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::EstimatorData msg_;
};

class Init_EstimatorData_orientation
{
public:
  explicit Init_EstimatorData_orientation(::custom_msgs::msg::EstimatorData & msg)
  : msg_(msg)
  {}
  Init_EstimatorData_raw_data orientation(::custom_msgs::msg::EstimatorData::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_EstimatorData_raw_data(msg_);
  }

private:
  ::custom_msgs::msg::EstimatorData msg_;
};

class Init_EstimatorData_accel
{
public:
  explicit Init_EstimatorData_accel(::custom_msgs::msg::EstimatorData & msg)
  : msg_(msg)
  {}
  Init_EstimatorData_orientation accel(::custom_msgs::msg::EstimatorData::_accel_type arg)
  {
    msg_.accel = std::move(arg);
    return Init_EstimatorData_orientation(msg_);
  }

private:
  ::custom_msgs::msg::EstimatorData msg_;
};

class Init_EstimatorData_speed
{
public:
  explicit Init_EstimatorData_speed(::custom_msgs::msg::EstimatorData & msg)
  : msg_(msg)
  {}
  Init_EstimatorData_accel speed(::custom_msgs::msg::EstimatorData::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_EstimatorData_accel(msg_);
  }

private:
  ::custom_msgs::msg::EstimatorData msg_;
};

class Init_EstimatorData_possition
{
public:
  explicit Init_EstimatorData_possition(::custom_msgs::msg::EstimatorData & msg)
  : msg_(msg)
  {}
  Init_EstimatorData_speed possition(::custom_msgs::msg::EstimatorData::_possition_type arg)
  {
    msg_.possition = std::move(arg);
    return Init_EstimatorData_speed(msg_);
  }

private:
  ::custom_msgs::msg::EstimatorData msg_;
};

class Init_EstimatorData_timestamp
{
public:
  Init_EstimatorData_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EstimatorData_possition timestamp(::custom_msgs::msg::EstimatorData::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_EstimatorData_possition(msg_);
  }

private:
  ::custom_msgs::msg::EstimatorData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::EstimatorData>()
{
  return custom_msgs::msg::builder::Init_EstimatorData_timestamp();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__ESTIMATOR_DATA__BUILDER_HPP_
