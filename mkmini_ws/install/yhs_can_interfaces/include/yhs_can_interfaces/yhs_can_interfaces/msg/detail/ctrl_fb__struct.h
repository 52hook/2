// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yhs_can_interfaces:msg/CtrlFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__CTRL_FB__STRUCT_H_
#define YHS_CAN_INTERFACES__MSG__DETAIL__CTRL_FB__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CtrlFb in the package yhs_can_interfaces.
typedef struct yhs_can_interfaces__msg__CtrlFb
{
  uint8_t ctrl_fb_gear;
  float ctrl_fb_velocity;
  float ctrl_fb_steering;
  uint8_t ctrl_fb_mode;
} yhs_can_interfaces__msg__CtrlFb;

// Struct for a sequence of yhs_can_interfaces__msg__CtrlFb.
typedef struct yhs_can_interfaces__msg__CtrlFb__Sequence
{
  yhs_can_interfaces__msg__CtrlFb * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yhs_can_interfaces__msg__CtrlFb__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__CTRL_FB__STRUCT_H_
