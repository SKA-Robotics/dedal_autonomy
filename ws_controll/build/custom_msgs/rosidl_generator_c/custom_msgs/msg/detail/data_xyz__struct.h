// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/DataXYZ.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__DATA_XYZ__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__DATA_XYZ__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/DataXYZ in the package custom_msgs.
typedef struct custom_msgs__msg__DataXYZ
{
  double x;
  double y;
  double z;
} custom_msgs__msg__DataXYZ;

// Struct for a sequence of custom_msgs__msg__DataXYZ.
typedef struct custom_msgs__msg__DataXYZ__Sequence
{
  custom_msgs__msg__DataXYZ * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__DataXYZ__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__DATA_XYZ__STRUCT_H_
