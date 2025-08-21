// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from custom_msgs:msg/TagLocation.idl
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
#include "custom_msgs/msg/detail/tag_location__struct.h"
#include "custom_msgs/msg/detail/tag_location__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool custom_msgs__msg__tag_location__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("custom_msgs.msg._tag_location.TagLocation", full_classname_dest, 41) == 0);
  }
  custom_msgs__msg__TagLocation * ros_message = _ros_message;
  {  // x_distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_distance = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_distance = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_distance = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch_rads
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch_rads");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch_rads = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // roll_rads
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll_rads");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll_rads = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw_rads
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw_rads");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw_rads = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * custom_msgs__msg__tag_location__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TagLocation */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("custom_msgs.msg._tag_location");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TagLocation");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  custom_msgs__msg__TagLocation * ros_message = (custom_msgs__msg__TagLocation *)raw_ros_message;
  {  // x_distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch_rads
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch_rads);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch_rads", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roll_rads
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll_rads);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll_rads", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw_rads
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw_rads);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw_rads", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
