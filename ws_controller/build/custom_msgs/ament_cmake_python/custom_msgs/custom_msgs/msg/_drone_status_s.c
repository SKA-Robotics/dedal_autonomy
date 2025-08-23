// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from custom_msgs:msg/DroneStatus.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "custom_msgs/msg/detail/drone_status__struct.h"
#include "custom_msgs/msg/detail/drone_status__functions.h"

bool custom_msgs__msg__geo_data__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * custom_msgs__msg__geo_data__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool custom_msgs__msg__drone_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[42];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("custom_msgs.msg._drone_status.DroneStatus", full_classname_dest, 41) == 0);
  }
  custom_msgs__msg__DroneStatus * ros_message = _ros_message;
  {  // is_armed
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_armed");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_armed = (Py_True == field);
    Py_DECREF(field);
  }
  {  // is_autonomy_active
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_autonomy_active");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_autonomy_active = (Py_True == field);
    Py_DECREF(field);
  }
  {  // is_moving
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_moving");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_moving = (Py_True == field);
    Py_DECREF(field);
  }
  {  // battery_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->battery_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ekf_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "ekf_position");
    if (!field) {
      return false;
    }
    if (!custom_msgs__msg__geo_data__convert_from_py(field, &ros_message->ekf_position)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * custom_msgs__msg__drone_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DroneStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("custom_msgs.msg._drone_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DroneStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  custom_msgs__msg__DroneStatus * ros_message = (custom_msgs__msg__DroneStatus *)raw_ros_message;
  {  // is_armed
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_armed ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_armed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_autonomy_active
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_autonomy_active ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_autonomy_active", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_moving
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_moving ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_moving", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->battery_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ekf_position
    PyObject * field = NULL;
    field = custom_msgs__msg__geo_data__convert_to_py(&ros_message->ekf_position);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "ekf_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
