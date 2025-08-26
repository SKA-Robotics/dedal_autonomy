// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/DroneStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__DRONE_STATUS__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__DRONE_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'ekf_position'
#include "custom_msgs/msg/detail/geo_data__struct.h"

/// Struct defined in msg/DroneStatus in the package custom_msgs.
typedef struct custom_msgs__msg__DroneStatus
{
  bool is_armed;
  bool is_autonomy_active;
  bool is_moving;
  bool is_searching;
  bool is_durning_takeoff;
  bool is_target_spotted;
  float battery_voltage;
  custom_msgs__msg__GeoData ekf_position;
} custom_msgs__msg__DroneStatus;

// Struct for a sequence of custom_msgs__msg__DroneStatus.
typedef struct custom_msgs__msg__DroneStatus__Sequence
{
  custom_msgs__msg__DroneStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__DroneStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__DRONE_STATUS__STRUCT_H_
