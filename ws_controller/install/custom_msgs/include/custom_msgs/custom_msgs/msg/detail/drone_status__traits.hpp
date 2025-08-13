// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:msg/DroneStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__DRONE_STATUS__TRAITS_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__DRONE_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msgs/msg/detail/drone_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'ekf_position'
#include "custom_msgs/msg/detail/geo_data__traits.hpp"

namespace custom_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DroneStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: is_autonomy_active
  {
    out << "is_autonomy_active: ";
    rosidl_generator_traits::value_to_yaml(msg.is_autonomy_active, out);
    out << ", ";
  }

  // member: battery_voltage
  {
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << ", ";
  }

  // member: ekf_position
  {
    out << "ekf_position: ";
    to_flow_style_yaml(msg.ekf_position, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DroneStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: is_autonomy_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_autonomy_active: ";
    rosidl_generator_traits::value_to_yaml(msg.is_autonomy_active, out);
    out << "\n";
  }

  // member: battery_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << "\n";
  }

  // member: ekf_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ekf_position:\n";
    to_block_style_yaml(msg.ekf_position, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DroneStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_msgs

namespace rosidl_generator_traits
{

[[deprecated("use custom_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_msgs::msg::DroneStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msgs::msg::DroneStatus & msg)
{
  return custom_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msgs::msg::DroneStatus>()
{
  return "custom_msgs::msg::DroneStatus";
}

template<>
inline const char * name<custom_msgs::msg::DroneStatus>()
{
  return "custom_msgs/msg/DroneStatus";
}

template<>
struct has_fixed_size<custom_msgs::msg::DroneStatus>
  : std::integral_constant<bool, has_fixed_size<custom_msgs::msg::GeoData>::value> {};

template<>
struct has_bounded_size<custom_msgs::msg::DroneStatus>
  : std::integral_constant<bool, has_bounded_size<custom_msgs::msg::GeoData>::value> {};

template<>
struct is_message<custom_msgs::msg::DroneStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__MSG__DETAIL__DRONE_STATUS__TRAITS_HPP_
