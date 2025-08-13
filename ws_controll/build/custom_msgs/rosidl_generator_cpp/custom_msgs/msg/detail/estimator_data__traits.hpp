// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:msg/EstimatorData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__ESTIMATOR_DATA__TRAITS_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__ESTIMATOR_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msgs/msg/detail/estimator_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'possition'
// Member 'speed'
// Member 'accel'
// Member 'orientation'
// Member 'raw_data'
#include "custom_msgs/msg/detail/data_xyz__traits.hpp"

namespace custom_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EstimatorData & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: possition
  {
    out << "possition: ";
    to_flow_style_yaml(msg.possition, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    to_flow_style_yaml(msg.speed, out);
    out << ", ";
  }

  // member: accel
  {
    out << "accel: ";
    to_flow_style_yaml(msg.accel, out);
    out << ", ";
  }

  // member: orientation
  {
    out << "orientation: ";
    to_flow_style_yaml(msg.orientation, out);
    out << ", ";
  }

  // member: raw_data
  {
    out << "raw_data: ";
    to_flow_style_yaml(msg.raw_data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EstimatorData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: possition
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "possition:\n";
    to_block_style_yaml(msg.possition, out, indentation + 2);
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed:\n";
    to_block_style_yaml(msg.speed, out, indentation + 2);
  }

  // member: accel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel:\n";
    to_block_style_yaml(msg.accel, out, indentation + 2);
  }

  // member: orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation:\n";
    to_block_style_yaml(msg.orientation, out, indentation + 2);
  }

  // member: raw_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "raw_data:\n";
    to_block_style_yaml(msg.raw_data, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EstimatorData & msg, bool use_flow_style = false)
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
  const custom_msgs::msg::EstimatorData & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msgs::msg::EstimatorData & msg)
{
  return custom_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msgs::msg::EstimatorData>()
{
  return "custom_msgs::msg::EstimatorData";
}

template<>
inline const char * name<custom_msgs::msg::EstimatorData>()
{
  return "custom_msgs/msg/EstimatorData";
}

template<>
struct has_fixed_size<custom_msgs::msg::EstimatorData>
  : std::integral_constant<bool, has_fixed_size<custom_msgs::msg::DataXYZ>::value> {};

template<>
struct has_bounded_size<custom_msgs::msg::EstimatorData>
  : std::integral_constant<bool, has_bounded_size<custom_msgs::msg::DataXYZ>::value> {};

template<>
struct is_message<custom_msgs::msg::EstimatorData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__MSG__DETAIL__ESTIMATOR_DATA__TRAITS_HPP_
