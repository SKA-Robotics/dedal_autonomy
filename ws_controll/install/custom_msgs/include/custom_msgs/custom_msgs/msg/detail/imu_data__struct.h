// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/ImuData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'accel'
#include "custom_msgs/msg/detail/accel_data__struct.h"
// Member 'gyro'
#include "custom_msgs/msg/detail/gyro_data__struct.h"

/// Struct defined in msg/ImuData in the package custom_msgs.
typedef struct custom_msgs__msg__ImuData
{
  int64_t timestamp;
  custom_msgs__msg__AccelData accel;
  custom_msgs__msg__GyroData gyro;
} custom_msgs__msg__ImuData;

// Struct for a sequence of custom_msgs__msg__ImuData.
typedef struct custom_msgs__msg__ImuData__Sequence
{
  custom_msgs__msg__ImuData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__ImuData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__IMU_DATA__STRUCT_H_
