// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yhs_can_interfaces:msg/DriveMcuEcodeFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__DRIVE_MCU_ECODE_FB__STRUCT_H_
#define YHS_CAN_INTERFACES__MSG__DETAIL__DRIVE_MCU_ECODE_FB__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/DriveMcuEcodeFb in the package yhs_can_interfaces.
typedef struct yhs_can_interfaces__msg__DriveMcuEcodeFb
{
  int32_t drive_fb_mcuecode;
} yhs_can_interfaces__msg__DriveMcuEcodeFb;

// Struct for a sequence of yhs_can_interfaces__msg__DriveMcuEcodeFb.
typedef struct yhs_can_interfaces__msg__DriveMcuEcodeFb__Sequence
{
  yhs_can_interfaces__msg__DriveMcuEcodeFb * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yhs_can_interfaces__msg__DriveMcuEcodeFb__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__DRIVE_MCU_ECODE_FB__STRUCT_H_
