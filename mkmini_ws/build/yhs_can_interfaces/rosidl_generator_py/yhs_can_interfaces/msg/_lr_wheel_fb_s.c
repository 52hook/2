// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from yhs_can_interfaces:msg/LrWheelFb.idl
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
#include "yhs_can_interfaces/msg/detail/lr_wheel_fb__struct.h"
#include "yhs_can_interfaces/msg/detail/lr_wheel_fb__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool yhs_can_interfaces__msg__lr_wheel_fb__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
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
    assert(strncmp("yhs_can_interfaces.msg._lr_wheel_fb.LrWheelFb", full_classname_dest, 45) == 0);
  }
  yhs_can_interfaces__msg__LrWheelFb * ros_message = _ros_message;
  {  // lr_wheel_fb_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "lr_wheel_fb_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->lr_wheel_fb_velocity = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // lr_wheel_fb_pulse
    PyObject * field = PyObject_GetAttrString(_pymsg, "lr_wheel_fb_pulse");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->lr_wheel_fb_pulse = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * yhs_can_interfaces__msg__lr_wheel_fb__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of LrWheelFb */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("yhs_can_interfaces.msg._lr_wheel_fb");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "LrWheelFb");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  yhs_can_interfaces__msg__LrWheelFb * ros_message = (yhs_can_interfaces__msg__LrWheelFb *)raw_ros_message;
  {  // lr_wheel_fb_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->lr_wheel_fb_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lr_wheel_fb_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lr_wheel_fb_pulse
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->lr_wheel_fb_pulse);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lr_wheel_fb_pulse", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
