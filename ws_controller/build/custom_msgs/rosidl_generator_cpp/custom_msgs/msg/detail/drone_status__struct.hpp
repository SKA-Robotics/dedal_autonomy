// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/DroneStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__DRONE_STATUS__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__DRONE_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'ekf_position'
#include "custom_msgs/msg/detail/geo_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__DroneStatus __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__DroneStatus __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DroneStatus_
{
  using Type = DroneStatus_<ContainerAllocator>;

  explicit DroneStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ekf_position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_autonomy_active = false;
      this->battery_voltage = 0.0f;
    }
  }

  explicit DroneStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ekf_position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_autonomy_active = false;
      this->battery_voltage = 0.0f;
    }
  }

  // field types and members
  using _is_autonomy_active_type =
    bool;
  _is_autonomy_active_type is_autonomy_active;
  using _battery_voltage_type =
    float;
  _battery_voltage_type battery_voltage;
  using _ekf_position_type =
    custom_msgs::msg::GeoData_<ContainerAllocator>;
  _ekf_position_type ekf_position;

  // setters for named parameter idiom
  Type & set__is_autonomy_active(
    const bool & _arg)
  {
    this->is_autonomy_active = _arg;
    return *this;
  }
  Type & set__battery_voltage(
    const float & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__ekf_position(
    const custom_msgs::msg::GeoData_<ContainerAllocator> & _arg)
  {
    this->ekf_position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::DroneStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::DroneStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::DroneStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::DroneStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::DroneStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::DroneStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::DroneStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::DroneStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::DroneStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::DroneStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__DroneStatus
    std::shared_ptr<custom_msgs::msg::DroneStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__DroneStatus
    std::shared_ptr<custom_msgs::msg::DroneStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DroneStatus_ & other) const
  {
    if (this->is_autonomy_active != other.is_autonomy_active) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->ekf_position != other.ekf_position) {
      return false;
    }
    return true;
  }
  bool operator!=(const DroneStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DroneStatus_

// alias to use template instance with default allocator
using DroneStatus =
  custom_msgs::msg::DroneStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__DRONE_STATUS__STRUCT_HPP_
