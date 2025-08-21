// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:msg/TagLocation.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__TAG_LOCATION__TRAITS_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__TAG_LOCATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msgs/msg/detail/tag_location__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TagLocation & msg,
  std::ostream & out)
{
  out << "{";
  // member: x_distance
  {
    out << "x_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.x_distance, out);
    out << ", ";
  }

  // member: y_distance
  {
    out << "y_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.y_distance, out);
    out << ", ";
  }

  // member: z_distance
  {
    out << "z_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.z_distance, out);
    out << ", ";
  }

  // member: pitch_rads
  {
    out << "pitch_rads: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_rads, out);
    out << ", ";
  }

  // member: roll_rads
  {
    out << "roll_rads: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_rads, out);
    out << ", ";
  }

  // member: yaw_rads
  {
    out << "yaw_rads: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rads, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TagLocation & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.x_distance, out);
    out << "\n";
  }

  // member: y_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.y_distance, out);
    out << "\n";
  }

  // member: z_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.z_distance, out);
    out << "\n";
  }

  // member: pitch_rads
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_rads: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_rads, out);
    out << "\n";
  }

  // member: roll_rads
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_rads: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_rads, out);
    out << "\n";
  }

  // member: yaw_rads
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_rads: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rads, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TagLocation & msg, bool use_flow_style = false)
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
  const custom_msgs::msg::TagLocation & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msgs::msg::TagLocation & msg)
{
  return custom_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msgs::msg::TagLocation>()
{
  return "custom_msgs::msg::TagLocation";
}

template<>
inline const char * name<custom_msgs::msg::TagLocation>()
{
  return "custom_msgs/msg/TagLocation";
}

template<>
struct has_fixed_size<custom_msgs::msg::TagLocation>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_msgs::msg::TagLocation>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_msgs::msg::TagLocation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__MSG__DETAIL__TAG_LOCATION__TRAITS_HPP_
