// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/TagLocation.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__TAG_LOCATION__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__TAG_LOCATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/tag_location__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_TagLocation_yaw_rads
{
public:
  explicit Init_TagLocation_yaw_rads(::custom_msgs::msg::TagLocation & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::TagLocation yaw_rads(::custom_msgs::msg::TagLocation::_yaw_rads_type arg)
  {
    msg_.yaw_rads = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::TagLocation msg_;
};

class Init_TagLocation_roll_rads
{
public:
  explicit Init_TagLocation_roll_rads(::custom_msgs::msg::TagLocation & msg)
  : msg_(msg)
  {}
  Init_TagLocation_yaw_rads roll_rads(::custom_msgs::msg::TagLocation::_roll_rads_type arg)
  {
    msg_.roll_rads = std::move(arg);
    return Init_TagLocation_yaw_rads(msg_);
  }

private:
  ::custom_msgs::msg::TagLocation msg_;
};

class Init_TagLocation_pitch_rads
{
public:
  explicit Init_TagLocation_pitch_rads(::custom_msgs::msg::TagLocation & msg)
  : msg_(msg)
  {}
  Init_TagLocation_roll_rads pitch_rads(::custom_msgs::msg::TagLocation::_pitch_rads_type arg)
  {
    msg_.pitch_rads = std::move(arg);
    return Init_TagLocation_roll_rads(msg_);
  }

private:
  ::custom_msgs::msg::TagLocation msg_;
};

class Init_TagLocation_z_distance
{
public:
  explicit Init_TagLocation_z_distance(::custom_msgs::msg::TagLocation & msg)
  : msg_(msg)
  {}
  Init_TagLocation_pitch_rads z_distance(::custom_msgs::msg::TagLocation::_z_distance_type arg)
  {
    msg_.z_distance = std::move(arg);
    return Init_TagLocation_pitch_rads(msg_);
  }

private:
  ::custom_msgs::msg::TagLocation msg_;
};

class Init_TagLocation_y_distance
{
public:
  explicit Init_TagLocation_y_distance(::custom_msgs::msg::TagLocation & msg)
  : msg_(msg)
  {}
  Init_TagLocation_z_distance y_distance(::custom_msgs::msg::TagLocation::_y_distance_type arg)
  {
    msg_.y_distance = std::move(arg);
    return Init_TagLocation_z_distance(msg_);
  }

private:
  ::custom_msgs::msg::TagLocation msg_;
};

class Init_TagLocation_x_distance
{
public:
  Init_TagLocation_x_distance()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TagLocation_y_distance x_distance(::custom_msgs::msg::TagLocation::_x_distance_type arg)
  {
    msg_.x_distance = std::move(arg);
    return Init_TagLocation_y_distance(msg_);
  }

private:
  ::custom_msgs::msg::TagLocation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::TagLocation>()
{
  return custom_msgs::msg::builder::Init_TagLocation_x_distance();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__TAG_LOCATION__BUILDER_HPP_
