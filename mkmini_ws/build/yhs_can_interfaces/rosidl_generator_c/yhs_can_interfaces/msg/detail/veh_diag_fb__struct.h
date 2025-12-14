// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yhs_can_interfaces:msg/VehDiagFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__VEH_DIAG_FB__STRUCT_H_
#define YHS_CAN_INTERFACES__MSG__DETAIL__VEH_DIAG_FB__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/VehDiagFb in the package yhs_can_interfaces.
typedef struct yhs_can_interfaces__msg__VehDiagFb
{
  uint8_t veh_fb_fault_level;
  bool veh_fb_auto_can_ctrl_cmd;
  bool veh_fb_auto_io_can_cmd;
  bool veh_fb_eps_dis_on_line;
  bool veh_fb_eps_fault;
  bool veh_fb_eps_mosf_et_ot;
  bool veh_fb_eps_warning;
  bool veh_fb_eps_dis_work;
  bool veh_fb_eps_over_current;
  bool veh_fb_st_reserve;
  bool veh_fb_ehb_ecu_fault;
  bool veh_fb_ehb_dis_on_line;
  bool veh_fb_ehb_work_model_fault;
  bool veh_fb_ehb_dis_en;
  bool veh_fb_ehb_anguler_fault;
  bool veh_fb_ehb_ot;
  bool veh_fb_ehb_power_fault;
  bool veh_fb_ehb_sensor_abnomal;
  bool veh_fb_ehb_motor_fault;
  bool veh_fb_ehb_oil_press_sensor_fault;
  bool veh_fb_ehb_oil_fault;
  uint8_t veh_fb_ld_rv_mcu_fault;
  uint8_t veh_fb_rd_rv_mcu_fault;
  bool veh_fb_aux_bms_dis_on_line;
  bool veh_fb_aux_scram;
  bool veh_fb_aux_remote_close;
  bool veh_fb_aux_remote_dis_on_line;
} yhs_can_interfaces__msg__VehDiagFb;

// Struct for a sequence of yhs_can_interfaces__msg__VehDiagFb.
typedef struct yhs_can_interfaces__msg__VehDiagFb__Sequence
{
  yhs_can_interfaces__msg__VehDiagFb * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yhs_can_interfaces__msg__VehDiagFb__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__VEH_DIAG_FB__STRUCT_H_
