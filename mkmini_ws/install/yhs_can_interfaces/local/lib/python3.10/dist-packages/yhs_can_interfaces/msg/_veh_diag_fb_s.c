// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from yhs_can_interfaces:msg/VehDiagFb.idl
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
#include "yhs_can_interfaces/msg/detail/veh_diag_fb__struct.h"
#include "yhs_can_interfaces/msg/detail/veh_diag_fb__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool yhs_can_interfaces__msg__veh_diag_fb__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("yhs_can_interfaces.msg._veh_diag_fb.VehDiagFb", full_classname_dest, 45) == 0);
  }
  yhs_can_interfaces__msg__VehDiagFb * ros_message = _ros_message;
  {  // veh_fb_fault_level
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_fault_level");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->veh_fb_fault_level = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // veh_fb_auto_can_ctrl_cmd
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_auto_can_ctrl_cmd");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_auto_can_ctrl_cmd = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_auto_io_can_cmd
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_auto_io_can_cmd");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_auto_io_can_cmd = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_eps_dis_on_line
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_eps_dis_on_line");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_eps_dis_on_line = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_eps_fault
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_eps_fault");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_eps_fault = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_eps_mosf_et_ot
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_eps_mosf_et_ot");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_eps_mosf_et_ot = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_eps_warning
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_eps_warning");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_eps_warning = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_eps_dis_work
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_eps_dis_work");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_eps_dis_work = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_eps_over_current
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_eps_over_current");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_eps_over_current = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_st_reserve
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_st_reserve");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_st_reserve = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_ehb_ecu_fault
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_ehb_ecu_fault");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_ehb_ecu_fault = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_ehb_dis_on_line
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_ehb_dis_on_line");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_ehb_dis_on_line = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_ehb_work_model_fault
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_ehb_work_model_fault");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_ehb_work_model_fault = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_ehb_dis_en
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_ehb_dis_en");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_ehb_dis_en = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_ehb_anguler_fault
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_ehb_anguler_fault");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_ehb_anguler_fault = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_ehb_ot
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_ehb_ot");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_ehb_ot = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_ehb_power_fault
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_ehb_power_fault");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_ehb_power_fault = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_ehb_sensor_abnomal
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_ehb_sensor_abnomal");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_ehb_sensor_abnomal = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_ehb_motor_fault
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_ehb_motor_fault");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_ehb_motor_fault = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_ehb_oil_press_sensor_fault
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_ehb_oil_press_sensor_fault");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_ehb_oil_press_sensor_fault = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_ehb_oil_fault
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_ehb_oil_fault");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_ehb_oil_fault = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_ld_rv_mcu_fault
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_ld_rv_mcu_fault");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->veh_fb_ld_rv_mcu_fault = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // veh_fb_rd_rv_mcu_fault
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_rd_rv_mcu_fault");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->veh_fb_rd_rv_mcu_fault = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // veh_fb_aux_bms_dis_on_line
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_aux_bms_dis_on_line");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_aux_bms_dis_on_line = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_aux_scram
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_aux_scram");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_aux_scram = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_aux_remote_close
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_aux_remote_close");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_aux_remote_close = (Py_True == field);
    Py_DECREF(field);
  }
  {  // veh_fb_aux_remote_dis_on_line
    PyObject * field = PyObject_GetAttrString(_pymsg, "veh_fb_aux_remote_dis_on_line");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->veh_fb_aux_remote_dis_on_line = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * yhs_can_interfaces__msg__veh_diag_fb__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of VehDiagFb */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("yhs_can_interfaces.msg._veh_diag_fb");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "VehDiagFb");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  yhs_can_interfaces__msg__VehDiagFb * ros_message = (yhs_can_interfaces__msg__VehDiagFb *)raw_ros_message;
  {  // veh_fb_fault_level
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->veh_fb_fault_level);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_fault_level", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_auto_can_ctrl_cmd
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_auto_can_ctrl_cmd ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_auto_can_ctrl_cmd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_auto_io_can_cmd
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_auto_io_can_cmd ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_auto_io_can_cmd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_eps_dis_on_line
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_eps_dis_on_line ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_eps_dis_on_line", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_eps_fault
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_eps_fault ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_eps_fault", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_eps_mosf_et_ot
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_eps_mosf_et_ot ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_eps_mosf_et_ot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_eps_warning
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_eps_warning ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_eps_warning", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_eps_dis_work
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_eps_dis_work ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_eps_dis_work", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_eps_over_current
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_eps_over_current ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_eps_over_current", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_st_reserve
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_st_reserve ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_st_reserve", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_ehb_ecu_fault
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_ehb_ecu_fault ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_ehb_ecu_fault", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_ehb_dis_on_line
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_ehb_dis_on_line ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_ehb_dis_on_line", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_ehb_work_model_fault
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_ehb_work_model_fault ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_ehb_work_model_fault", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_ehb_dis_en
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_ehb_dis_en ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_ehb_dis_en", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_ehb_anguler_fault
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_ehb_anguler_fault ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_ehb_anguler_fault", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_ehb_ot
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_ehb_ot ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_ehb_ot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_ehb_power_fault
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_ehb_power_fault ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_ehb_power_fault", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_ehb_sensor_abnomal
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_ehb_sensor_abnomal ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_ehb_sensor_abnomal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_ehb_motor_fault
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_ehb_motor_fault ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_ehb_motor_fault", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_ehb_oil_press_sensor_fault
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_ehb_oil_press_sensor_fault ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_ehb_oil_press_sensor_fault", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_ehb_oil_fault
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_ehb_oil_fault ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_ehb_oil_fault", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_ld_rv_mcu_fault
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->veh_fb_ld_rv_mcu_fault);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_ld_rv_mcu_fault", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_rd_rv_mcu_fault
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->veh_fb_rd_rv_mcu_fault);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_rd_rv_mcu_fault", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_aux_bms_dis_on_line
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_aux_bms_dis_on_line ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_aux_bms_dis_on_line", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_aux_scram
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_aux_scram ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_aux_scram", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_aux_remote_close
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_aux_remote_close ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_aux_remote_close", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // veh_fb_aux_remote_dis_on_line
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->veh_fb_aux_remote_dis_on_line ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "veh_fb_aux_remote_dis_on_line", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
