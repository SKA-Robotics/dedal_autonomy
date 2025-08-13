// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/ImuData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'accel'
#include "custom_msgs/msg/detail/accel_data__struct.hpp"
// Member 'gyro'
#include "custom_msgs/msg/detail/gyro_data__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__ImuData __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__ImuData __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ImuData_
{
  using Type = ImuData_<ContainerAllocator>;

  explicit ImuData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : accel(_init),
    gyro(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ll;
    }
  }

  explicit ImuData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : accel(_alloc, _init),
    gyro(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ll;
    }
  }

  // field types and members
  using _timestamp_type =
    int64_t;
  _timestamp_type timestamp;
  using _accel_type =
    custom_msgs::msg::AccelData_<ContainerAllocator>;
  _accel_type accel;
  using _gyro_type =
    custom_msgs::msg::GyroData_<ContainerAllocator>;
  _gyro_type gyro;

  // setters for named parameter idiom
  Type & set__timestamp(
    const int64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__accel(
    const custom_msgs::msg::AccelData_<ContainerAllocator> & _arg)
  {
    this->accel = _arg;
    return *this;
  }
  Type & set__gyro(
    const custom_msgs::msg::GyroData_<ContainerAllocator> & _arg)
  {
    this->gyro = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::ImuData_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::ImuData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::ImuData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::ImuData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::ImuData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::ImuData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::ImuData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::ImuData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::ImuData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::ImuData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__ImuData
    std::shared_ptr<custom_msgs::msg::ImuData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__ImuData
    std::shared_ptr<custom_msgs::msg::ImuData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ImuData_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->accel != other.accel) {
      return false;
    }
    if (this->gyro != other.gyro) {
      return false;
    }
    return true;
  }
  bool operator!=(const ImuData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ImuData_

// alias to use template instance with default allocator
using ImuData =
  custom_msgs::msg::ImuData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_HPP_
