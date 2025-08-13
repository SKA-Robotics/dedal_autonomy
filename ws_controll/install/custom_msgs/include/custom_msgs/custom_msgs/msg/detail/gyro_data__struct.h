// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/GyroData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__GYRO_DATA__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__GYRO_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/GyroData in the package custom_msgs.
typedef struct custom_msgs__msg__GyroData
{
  double x;
  double y;
  double z;
} custom_msgs__msg__GyroData;

// Struct for a sequence of custom_msgs__msg__GyroData.
typedef struct custom_msgs__msg__GyroData__Sequence
{
  custom_msgs__msg__GyroData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__GyroData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__GYRO_DATA__STRUCT_H_
