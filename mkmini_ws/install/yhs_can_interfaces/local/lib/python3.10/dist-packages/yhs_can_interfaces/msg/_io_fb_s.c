// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from yhs_can_interfaces:msg/IoFb.idl
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
#include "yhs_can_interfaces/msg/detail/io_fb__struct.h"
#include "yhs_can_interfaces/msg/detail/io_fb__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool yhs_can_interfaces__msg__io_fb__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[35];
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
    assert(strncmp("yhs_can_interfaces.msg._io_fb.IoFb", full_classname_dest, 34) == 0);
  }
  yhs_can_interfaces__msg__IoFb * ros_message = _ros_message;
  {  // io_fb_enable
    PyObject * field = PyObject_GetAttrString(_pymsg, "io_fb_enable");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->io_fb_enable = (Py_True == field);
    Py_DECREF(field);
  }
  {  // io_fb_turn_lamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "io_fb_turn_lamp");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->io_fb_turn_lamp = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // io_fb_braking_lamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "io_fb_braking_lamp");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->io_fb_braking_lamp = (Py_True == field);
    Py_DECREF(field);
  }
  {  // io_fb_fm_impact_sensor
    PyObject * field = PyObject_GetAttrString(_pymsg, "io_fb_fm_impact_sensor");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->io_fb_fm_impact_sensor = (Py_True == field);
    Py_DECREF(field);
  }
  {  // io_fb_rm_impact_sensor
    PyObject * field = PyObject_GetAttrString(_pymsg, "io_fb_rm_impact_sensor");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->io_fb_rm_impact_sensor = (Py_True == field);
    Py_DECREF(field);
  }
  {  // io_fb_dis_charge
    PyObject * field = PyObject_GetAttrString(_pymsg, "io_fb_dis_charge");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->io_fb_dis_charge = (Py_True == field);
    Py_DECREF(field);
  }
  {  // io_fb_charge_en
    PyObject * field = PyObject_GetAttrString(_pymsg, "io_fb_charge_en");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->io_fb_charge_en = (Py_True == field);
    Py_DECREF(field);
  }
  {  // io_fb_scram_st
    PyObject * field = PyObject_GetAttrString(_pymsg, "io_fb_scram_st");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->io_fb_scram_st = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * yhs_can_interfaces__msg__io_fb__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of IoFb */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("yhs_can_interfaces.msg._io_fb");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "IoFb");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  yhs_can_interfaces__msg__IoFb * ros_message = (yhs_can_interfaces__msg__IoFb *)raw_ros_message;
  {  // io_fb_enable
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->io_fb_enable ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "io_fb_enable", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // io_fb_turn_lamp
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->io_fb_turn_lamp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "io_fb_turn_lamp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // io_fb_braking_lamp
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->io_fb_braking_lamp ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "io_fb_braking_lamp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // io_fb_fm_impact_sensor
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->io_fb_fm_impact_sensor ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "io_fb_fm_impact_sensor", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // io_fb_rm_impact_sensor
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->io_fb_rm_impact_sensor ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "io_fb_rm_impact_sensor", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // io_fb_dis_charge
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->io_fb_dis_charge ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "io_fb_dis_charge", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // io_fb_charge_en
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->io_fb_charge_en ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "io_fb_charge_en", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // io_fb_scram_st
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->io_fb_scram_st ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "io_fb_scram_st", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
