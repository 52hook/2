// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yhs_can_interfaces:msg/ChassisInfoFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__CHASSIS_INFO_FB__STRUCT_H_
#define YHS_CAN_INTERFACES__MSG__DETAIL__CHASSIS_INFO_FB__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'ctrl_fb'
#include "yhs_can_interfaces/msg/detail/ctrl_fb__struct.h"
// Member 'io_fb'
#include "yhs_can_interfaces/msg/detail/io_fb__struct.h"
// Member 'lr_wheel_fb'
#include "yhs_can_interfaces/msg/detail/lr_wheel_fb__struct.h"
// Member 'rr_wheel_fb'
#include "yhs_can_interfaces/msg/detail/rr_wheel_fb__struct.h"
// Member 'bms_flag_info_fb'
#include "yhs_can_interfaces/msg/detail/bms_flag_info_fb__struct.h"
// Member 'bms_info_fb'
#include "yhs_can_interfaces/msg/detail/bms_info_fb__struct.h"
// Member 'drive_mcu_ecode_fb'
#include "yhs_can_interfaces/msg/detail/drive_mcu_ecode_fb__struct.h"
// Member 'veh_diag_fb'
#include "yhs_can_interfaces/msg/detail/veh_diag_fb__struct.h"

/// Struct defined in msg/ChassisInfoFb in the package yhs_can_interfaces.
typedef struct yhs_can_interfaces__msg__ChassisInfoFb
{
  std_msgs__msg__Header header;
  yhs_can_interfaces__msg__CtrlFb ctrl_fb;
  yhs_can_interfaces__msg__IoFb io_fb;
  yhs_can_interfaces__msg__LrWheelFb lr_wheel_fb;
  yhs_can_interfaces__msg__RrWheelFb rr_wheel_fb;
  yhs_can_interfaces__msg__BmsFlagInfoFb bms_flag_info_fb;
  yhs_can_interfaces__msg__BmsInfoFb bms_info_fb;
  yhs_can_interfaces__msg__DriveMcuEcodeFb drive_mcu_ecode_fb;
  yhs_can_interfaces__msg__VehDiagFb veh_diag_fb;
} yhs_can_interfaces__msg__ChassisInfoFb;

// Struct for a sequence of yhs_can_interfaces__msg__ChassisInfoFb.
typedef struct yhs_can_interfaces__msg__ChassisInfoFb__Sequence
{
  yhs_can_interfaces__msg__ChassisInfoFb * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yhs_can_interfaces__msg__ChassisInfoFb__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__CHASSIS_INFO_FB__STRUCT_H_
