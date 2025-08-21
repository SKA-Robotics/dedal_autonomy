// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_msgs:msg/DroneStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_msgs/msg/detail/drone_status__rosidl_typesupport_introspection_c.h"
#include "custom_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_msgs/msg/detail/drone_status__functions.h"
#include "custom_msgs/msg/detail/drone_status__struct.h"


// Include directives for member types
// Member `ekf_position`
#include "custom_msgs/msg/geo_data.h"
// Member `ekf_position`
#include "custom_msgs/msg/detail/geo_data__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_msgs__msg__DroneStatus__rosidl_typesupport_introspection_c__DroneStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_msgs__msg__DroneStatus__init(message_memory);
}

void custom_msgs__msg__DroneStatus__rosidl_typesupport_introspection_c__DroneStatus_fini_function(void * message_memory)
{
  custom_msgs__msg__DroneStatus__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_msgs__msg__DroneStatus__rosidl_typesupport_introspection_c__DroneStatus_message_member_array[4] = {
  {
    "is_autonomy_active",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs__msg__DroneStatus, is_autonomy_active),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_moving",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs__msg__DroneStatus, is_moving),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "battery_voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs__msg__DroneStatus, battery_voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ekf_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs__msg__DroneStatus, ekf_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_msgs__msg__DroneStatus__rosidl_typesupport_introspection_c__DroneStatus_message_members = {
  "custom_msgs__msg",  // message namespace
  "DroneStatus",  // message name
  4,  // number of fields
  sizeof(custom_msgs__msg__DroneStatus),
  custom_msgs__msg__DroneStatus__rosidl_typesupport_introspection_c__DroneStatus_message_member_array,  // message members
  custom_msgs__msg__DroneStatus__rosidl_typesupport_introspection_c__DroneStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_msgs__msg__DroneStatus__rosidl_typesupport_introspection_c__DroneStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_msgs__msg__DroneStatus__rosidl_typesupport_introspection_c__DroneStatus_message_type_support_handle = {
  0,
  &custom_msgs__msg__DroneStatus__rosidl_typesupport_introspection_c__DroneStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_msgs, msg, DroneStatus)() {
  custom_msgs__msg__DroneStatus__rosidl_typesupport_introspection_c__DroneStatus_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_msgs, msg, GeoData)();
  if (!custom_msgs__msg__DroneStatus__rosidl_typesupport_introspection_c__DroneStatus_message_type_support_handle.typesupport_identifier) {
    custom_msgs__msg__DroneStatus__rosidl_typesupport_introspection_c__DroneStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_msgs__msg__DroneStatus__rosidl_typesupport_introspection_c__DroneStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
