// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/EstimatorData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__ESTIMATOR_DATA__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__ESTIMATOR_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'possition'
// Member 'speed'
// Member 'accel'
// Member 'orientation'
// Member 'raw_data'
#include "custom_msgs/msg/detail/data_xyz__struct.h"

/// Struct defined in msg/EstimatorData in the package custom_msgs.
typedef struct custom_msgs__msg__EstimatorData
{
  int64_t timestamp;
  custom_msgs__msg__DataXYZ possition;
  custom_msgs__msg__DataXYZ speed;
  custom_msgs__msg__DataXYZ accel;
  custom_msgs__msg__DataXYZ orientation;
  custom_msgs__msg__DataXYZ raw_data;
} custom_msgs__msg__EstimatorData;

// Struct for a sequence of custom_msgs__msg__EstimatorData.
typedef struct custom_msgs__msg__EstimatorData__Sequence
{
  custom_msgs__msg__EstimatorData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__EstimatorData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__ESTIMATOR_DATA__STRUCT_H_
