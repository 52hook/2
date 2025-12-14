// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yhs_can_interfaces:msg/IoFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__IO_FB__STRUCT_H_
#define YHS_CAN_INTERFACES__MSG__DETAIL__IO_FB__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/IoFb in the package yhs_can_interfaces.
typedef struct yhs_can_interfaces__msg__IoFb
{
  bool io_fb_enable;
  int8_t io_fb_turn_lamp;
  bool io_fb_braking_lamp;
  bool io_fb_fm_impact_sensor;
  bool io_fb_rm_impact_sensor;
  bool io_fb_dis_charge;
  bool io_fb_charge_en;
  bool io_fb_scram_st;
} yhs_can_interfaces__msg__IoFb;

// Struct for a sequence of yhs_can_interfaces__msg__IoFb.
typedef struct yhs_can_interfaces__msg__IoFb__Sequence
{
  yhs_can_interfaces__msg__IoFb * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yhs_can_interfaces__msg__IoFb__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__IO_FB__STRUCT_H_
