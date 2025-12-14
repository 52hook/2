// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from yhs_can_interfaces:msg/ChassisInfoFb.idl
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
#include "yhs_can_interfaces/msg/detail/chassis_info_fb__struct.h"
#include "yhs_can_interfaces/msg/detail/chassis_info_fb__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool yhs_can_interfaces__msg__ctrl_fb__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * yhs_can_interfaces__msg__ctrl_fb__convert_to_py(void * raw_ros_message);
bool yhs_can_interfaces__msg__io_fb__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * yhs_can_interfaces__msg__io_fb__convert_to_py(void * raw_ros_message);
bool yhs_can_interfaces__msg__lr_wheel_fb__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * yhs_can_interfaces__msg__lr_wheel_fb__convert_to_py(void * raw_ros_message);
bool yhs_can_interfaces__msg__rr_wheel_fb__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * yhs_can_interfaces__msg__rr_wheel_fb__convert_to_py(void * raw_ros_message);
bool yhs_can_interfaces__msg__bms_flag_info_fb__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * yhs_can_interfaces__msg__bms_flag_info_fb__convert_to_py(void * raw_ros_message);
bool yhs_can_interfaces__msg__bms_info_fb__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * yhs_can_interfaces__msg__bms_info_fb__convert_to_py(void * raw_ros_message);
bool yhs_can_interfaces__msg__drive_mcu_ecode_fb__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * yhs_can_interfaces__msg__drive_mcu_ecode_fb__convert_to_py(void * raw_ros_message);
bool yhs_can_interfaces__msg__veh_diag_fb__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * yhs_can_interfaces__msg__veh_diag_fb__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool yhs_can_interfaces__msg__chassis_info_fb__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[54];
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
    assert(strncmp("yhs_can_interfaces.msg._chassis_info_fb.ChassisInfoFb", full_classname_dest, 53) == 0);
  }
  yhs_can_interfaces__msg__ChassisInfoFb * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // ctrl_fb
    PyObject * field = PyObject_GetAttrString(_pymsg, "ctrl_fb");
    if (!field) {
      return false;
    }
    if (!yhs_can_interfaces__msg__ctrl_fb__convert_from_py(field, &ros_message->ctrl_fb)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // io_fb
    PyObject * field = PyObject_GetAttrString(_pymsg, "io_fb");
    if (!field) {
      return false;
    }
    if (!yhs_can_interfaces__msg__io_fb__convert_from_py(field, &ros_message->io_fb)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // lr_wheel_fb
    PyObject * field = PyObject_GetAttrString(_pymsg, "lr_wheel_fb");
    if (!field) {
      return false;
    }
    if (!yhs_can_interfaces__msg__lr_wheel_fb__convert_from_py(field, &ros_message->lr_wheel_fb)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // rr_wheel_fb
    PyObject * field = PyObject_GetAttrString(_pymsg, "rr_wheel_fb");
    if (!field) {
      return false;
    }
    if (!yhs_can_interfaces__msg__rr_wheel_fb__convert_from_py(field, &ros_message->rr_wheel_fb)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // bms_flag_info_fb
    PyObject * field = PyObject_GetAttrString(_pymsg, "bms_flag_info_fb");
    if (!field) {
      return false;
    }
    if (!yhs_can_interfaces__msg__bms_flag_info_fb__convert_from_py(field, &ros_message->bms_flag_info_fb)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // bms_info_fb
    PyObject * field = PyObject_GetAttrString(_pymsg, "bms_info_fb");
    if (!field) {
      return false;
    }
    if (!yhs_can_interfaces__msg__bms_info_fb__convert_from_py(field, &ros_message->bms_info_fb)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // drive_mcu_ecode_fb
    PyObject * field = PyObject_GetAttrString(_pymsg, "drive_mcu_ecode_fb");
    if (!field) {
      return false;
    }
    if (!yhs_can_interfaces__msg__drive_mcu_ecode_fb__convert_from_py(field, &ros_message->drive_mcu_ecode_fb)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // veh_diag_fb
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_diag_fb");
    if (!field) {
      return false;
    }
    if (!yhs_can_interfaces__msg__veh_diag_fb__convert_from_py(field, &ros_message->veh_diag_fb)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * yhs_can_interfaces__msg__chassis_info_fb__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ChassisInfoFb */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("yhs_can_interfaces.msg._chassis_info_fb");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ChassisInfoFb");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  yhs_can_interfaces__msg__ChassisInfoFb * ros_message = (yhs_can_interfaces__msg__ChassisInfoFb *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ctrl_fb
    PyObject * field = NULL;
    field = yhs_can_interfaces__msg__ctrl_fb__convert_to_py(&ros_message->ctrl_fb);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "ctrl_fb", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // io_fb
    PyObject * field = NULL;
    field = yhs_can_interfaces__msg__io_fb__convert_to_py(&ros_message->io_fb);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "io_fb", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lr_wheel_fb
    PyObject * field = NULL;
    field = yhs_can_interfaces__msg__lr_wheel_fb__convert_to_py(&ros_message->lr_wheel_fb);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "lr_wheel_fb", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rr_wheel_fb
    PyObject * field = NULL;
    field = yhs_can_interfaces__msg__rr_wheel_fb__convert_to_py(&ros_message->rr_wheel_fb);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "rr_wheel_fb", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bms_flag_info_fb
    PyObject * field = NULL;
    field = yhs_can_interfaces__msg__bms_flag_info_fb__convert_to_py(&ros_message->bms_flag_info_fb);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "bms_flag_info_fb", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bms_info_fb
    PyObject * field = NULL;
    field = yhs_can_interfaces__msg__bms_info_fb__convert_to_py(&ros_message->bms_info_fb);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "bms_info_fb", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // drive_mcu_ecode_fb
    PyObject * field = NULL;
    field = yhs_can_interfaces__msg__drive_mcu_ecode_fb__convert_to_py(&ros_message->drive_mcu_ecode_fb);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "drive_mcu_ecode_fb", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_diag_fb
    PyObject * field = NULL;
    field = yhs_can_interfaces__msg__veh_diag_fb__convert_to_py(&ros_message->veh_diag_fb);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_diag_fb", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
