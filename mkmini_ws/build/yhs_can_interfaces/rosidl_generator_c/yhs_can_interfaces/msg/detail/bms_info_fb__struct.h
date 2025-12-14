// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yhs_can_interfaces:msg/BmsInfoFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__BMS_INFO_FB__STRUCT_H_
#define YHS_CAN_INTERFACES__MSG__DETAIL__BMS_INFO_FB__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/BmsInfoFb in the package yhs_can_interfaces.
typedef struct yhs_can_interfaces__msg__BmsInfoFb
{
  float bms_info_voltage;
  float bms_info_current;
  float bms_info_remaining_capacity;
} yhs_can_interfaces__msg__BmsInfoFb;

// Struct for a sequence of yhs_can_interfaces__msg__BmsInfoFb.
typedef struct yhs_can_interfaces__msg__BmsInfoFb__Sequence
{
  yhs_can_interfaces__msg__BmsInfoFb * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yhs_can_interfaces__msg__BmsInfoFb__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__BMS_INFO_FB__STRUCT_H_
