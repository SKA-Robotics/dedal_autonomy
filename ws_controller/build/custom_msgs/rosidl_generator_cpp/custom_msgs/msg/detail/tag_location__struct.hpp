// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/TagLocation.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__TAG_LOCATION__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__TAG_LOCATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__TagLocation __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__TagLocation __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TagLocation_
{
  using Type = TagLocation_<ContainerAllocator>;

  explicit TagLocation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_distance = 0.0;
      this->y_distance = 0.0;
      this->z_distance = 0.0;
      this->pitch_rads = 0.0;
      this->roll_rads = 0.0;
      this->yaw_rads = 0.0;
    }
  }

  explicit TagLocation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_distance = 0.0;
      this->y_distance = 0.0;
      this->z_distance = 0.0;
      this->pitch_rads = 0.0;
      this->roll_rads = 0.0;
      this->yaw_rads = 0.0;
    }
  }

  // field types and members
  using _x_distance_type =
    double;
  _x_distance_type x_distance;
  using _y_distance_type =
    double;
  _y_distance_type y_distance;
  using _z_distance_type =
    double;
  _z_distance_type z_distance;
  using _pitch_rads_type =
    double;
  _pitch_rads_type pitch_rads;
  using _roll_rads_type =
    double;
  _roll_rads_type roll_rads;
  using _yaw_rads_type =
    double;
  _yaw_rads_type yaw_rads;

  // setters for named parameter idiom
  Type & set__x_distance(
    const double & _arg)
  {
    this->x_distance = _arg;
    return *this;
  }
  Type & set__y_distance(
    const double & _arg)
  {
    this->y_distance = _arg;
    return *this;
  }
  Type & set__z_distance(
    const double & _arg)
  {
    this->z_distance = _arg;
    return *this;
  }
  Type & set__pitch_rads(
    const double & _arg)
  {
    this->pitch_rads = _arg;
    return *this;
  }
  Type & set__roll_rads(
    const double & _arg)
  {
    this->roll_rads = _arg;
    return *this;
  }
  Type & set__yaw_rads(
    const double & _arg)
  {
    this->yaw_rads = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::TagLocation_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::TagLocation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::TagLocation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::TagLocation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::TagLocation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::TagLocation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::TagLocation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::TagLocation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::TagLocation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::TagLocation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__TagLocation
    std::shared_ptr<custom_msgs::msg::TagLocation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__TagLocation
    std::shared_ptr<custom_msgs::msg::TagLocation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TagLocation_ & other) const
  {
    if (this->x_distance != other.x_distance) {
      return false;
    }
    if (this->y_distance != other.y_distance) {
      return false;
    }
    if (this->z_distance != other.z_distance) {
      return false;
    }
    if (this->pitch_rads != other.pitch_rads) {
      return false;
    }
    if (this->roll_rads != other.roll_rads) {
      return false;
    }
    if (this->yaw_rads != other.yaw_rads) {
      return false;
    }
    return true;
  }
  bool operator!=(const TagLocation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TagLocation_

// alias to use template instance with default allocator
using TagLocation =
  custom_msgs::msg::TagLocation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__TAG_LOCATION__STRUCT_HPP_
