// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yhs_can_interfaces:msg/LrWheelFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__LR_WHEEL_FB__STRUCT_H_
#define YHS_CAN_INTERFACES__MSG__DETAIL__LR_WHEEL_FB__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/LrWheelFb in the package yhs_can_interfaces.
typedef struct yhs_can_interfaces__msg__LrWheelFb
{
  float lr_wheel_fb_velocity;
  int32_t lr_wheel_fb_pulse;
} yhs_can_interfaces__msg__LrWheelFb;

// Struct for a sequence of yhs_can_interfaces__msg__LrWheelFb.
typedef struct yhs_can_interfaces__msg__LrWheelFb__Sequence
{
  yhs_can_interfaces__msg__LrWheelFb * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yhs_can_interfaces__msg__LrWheelFb__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__LR_WHEEL_FB__STRUCT_H_
