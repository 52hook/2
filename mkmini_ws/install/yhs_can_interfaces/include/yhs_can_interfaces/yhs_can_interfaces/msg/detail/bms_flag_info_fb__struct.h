// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yhs_can_interfaces:msg/BmsFlagInfoFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__BMS_FLAG_INFO_FB__STRUCT_H_
#define YHS_CAN_INTERFACES__MSG__DETAIL__BMS_FLAG_INFO_FB__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/BmsFlagInfoFb in the package yhs_can_interfaces.
typedef struct yhs_can_interfaces__msg__BmsFlagInfoFb
{
  uint8_t bms_flag_info_soc;
  bool bms_flag_info_single_ov;
  bool bms_flag_info_single_uv;
  bool bms_flag_info_ov;
  bool bms_flag_info_uv;
  bool bms_flag_info_charge_ot;
  bool bms_flag_info_charge_ut;
  bool bms_flag_info_discharge_ot;
  bool bms_flag_info_discharge_ut;
  bool bms_flag_info_charge_oc;
  bool bms_flag_info_discharge_oc;
  bool bms_flag_info_short;
  bool bms_flag_info_ic_error;
  bool bms_flag_info_lock_mos;
  bool bms_flag_info_charge_flag;
  bool bms_flag_info_soc_warning;
  bool bms_flag_info_soc_low_protection;
  float bms_flag_info_hight_temperature;
  float bms_flag_info_low_temperature;
} yhs_can_interfaces__msg__BmsFlagInfoFb;

// Struct for a sequence of yhs_can_interfaces__msg__BmsFlagInfoFb.
typedef struct yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence
{
  yhs_can_interfaces__msg__BmsFlagInfoFb * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__BMS_FLAG_INFO_FB__STRUCT_H_
