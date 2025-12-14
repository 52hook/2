// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yhs_can_interfaces:msg/CtrlCmd.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__CTRL_CMD__STRUCT_H_
#define YHS_CAN_INTERFACES__MSG__DETAIL__CTRL_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CtrlCmd in the package yhs_can_interfaces.
typedef struct yhs_can_interfaces__msg__CtrlCmd
{
  uint8_t ctrl_cmd_gear;
  float ctrl_cmd_velocity;
  float ctrl_cmd_steering;
} yhs_can_interfaces__msg__CtrlCmd;

// Struct for a sequence of yhs_can_interfaces__msg__CtrlCmd.
typedef struct yhs_can_interfaces__msg__CtrlCmd__Sequence
{
  yhs_can_interfaces__msg__CtrlCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yhs_can_interfaces__msg__CtrlCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__CTRL_CMD__STRUCT_H_
