// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/GeoData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__GEO_DATA__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__GEO_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/geo_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_GeoData_altitude
{
public:
  explicit Init_GeoData_altitude(::custom_msgs::msg::GeoData & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::GeoData altitude(::custom_msgs::msg::GeoData::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::GeoData msg_;
};

class Init_GeoData_longitude
{
public:
  explicit Init_GeoData_longitude(::custom_msgs::msg::GeoData & msg)
  : msg_(msg)
  {}
  Init_GeoData_altitude longitude(::custom_msgs::msg::GeoData::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_GeoData_altitude(msg_);
  }

private:
  ::custom_msgs::msg::GeoData msg_;
};

class Init_GeoData_latitude
{
public:
  Init_GeoData_latitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GeoData_longitude latitude(::custom_msgs::msg::GeoData::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_GeoData_longitude(msg_);
  }

private:
  ::custom_msgs::msg::GeoData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::GeoData>()
{
  return custom_msgs::msg::builder::Init_GeoData_latitude();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__GEO_DATA__BUILDER_HPP_
