// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/DroneStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__DRONE_STATUS__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__DRONE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/drone_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_DroneStatus_ekf_position
{
public:
  explicit Init_DroneStatus_ekf_position(::custom_msgs::msg::DroneStatus & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::DroneStatus ekf_position(::custom_msgs::msg::DroneStatus::_ekf_position_type arg)
  {
    msg_.ekf_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::DroneStatus msg_;
};

class Init_DroneStatus_battery_voltage
{
public:
  explicit Init_DroneStatus_battery_voltage(::custom_msgs::msg::DroneStatus & msg)
  : msg_(msg)
  {}
  Init_DroneStatus_ekf_position battery_voltage(::custom_msgs::msg::DroneStatus::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_DroneStatus_ekf_position(msg_);
  }

private:
  ::custom_msgs::msg::DroneStatus msg_;
};

class Init_DroneStatus_is_moving
{
public:
  explicit Init_DroneStatus_is_moving(::custom_msgs::msg::DroneStatus & msg)
  : msg_(msg)
  {}
  Init_DroneStatus_battery_voltage is_moving(::custom_msgs::msg::DroneStatus::_is_moving_type arg)
  {
    msg_.is_moving = std::move(arg);
    return Init_DroneStatus_battery_voltage(msg_);
  }

private:
  ::custom_msgs::msg::DroneStatus msg_;
};

class Init_DroneStatus_is_autonomy_active
{
public:
  explicit Init_DroneStatus_is_autonomy_active(::custom_msgs::msg::DroneStatus & msg)
  : msg_(msg)
  {}
  Init_DroneStatus_is_moving is_autonomy_active(::custom_msgs::msg::DroneStatus::_is_autonomy_active_type arg)
  {
    msg_.is_autonomy_active = std::move(arg);
    return Init_DroneStatus_is_moving(msg_);
  }

private:
  ::custom_msgs::msg::DroneStatus msg_;
};

class Init_DroneStatus_is_armed
{
public:
  Init_DroneStatus_is_armed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DroneStatus_is_autonomy_active is_armed(::custom_msgs::msg::DroneStatus::_is_armed_type arg)
  {
    msg_.is_armed = std::move(arg);
    return Init_DroneStatus_is_autonomy_active(msg_);
  }

private:
  ::custom_msgs::msg::DroneStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::DroneStatus>()
{
  return custom_msgs::msg::builder::Init_DroneStatus_is_armed();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__DRONE_STATUS__BUILDER_HPP_
