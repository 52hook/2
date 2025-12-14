// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from yhs_can_interfaces:msg/ChassisInfoFb.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "yhs_can_interfaces/msg/detail/chassis_info_fb__rosidl_typesupport_introspection_c.h"
#include "yhs_can_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "yhs_can_interfaces/msg/detail/chassis_info_fb__functions.h"
#include "yhs_can_interfaces/msg/detail/chassis_info_fb__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `ctrl_fb`
#include "yhs_can_interfaces/msg/ctrl_fb.h"
// Member `ctrl_fb`
#include "yhs_can_interfaces/msg/detail/ctrl_fb__rosidl_typesupport_introspection_c.h"
// Member `io_fb`
#include "yhs_can_interfaces/msg/io_fb.h"
// Member `io_fb`
#include "yhs_can_interfaces/msg/detail/io_fb__rosidl_typesupport_introspection_c.h"
// Member `lr_wheel_fb`
#include "yhs_can_interfaces/msg/lr_wheel_fb.h"
// Member `lr_wheel_fb`
#include "yhs_can_interfaces/msg/detail/lr_wheel_fb__rosidl_typesupport_introspection_c.h"
// Member `rr_wheel_fb`
#include "yhs_can_interfaces/msg/rr_wheel_fb.h"
// Member `rr_wheel_fb`
#include "yhs_can_interfaces/msg/detail/rr_wheel_fb__rosidl_typesupport_introspection_c.h"
// Member `bms_flag_info_fb`
#include "yhs_can_interfaces/msg/bms_flag_info_fb.h"
// Member `bms_flag_info_fb`
#include "yhs_can_interfaces/msg/detail/bms_flag_info_fb__rosidl_typesupport_introspection_c.h"
// Member `bms_info_fb`
#include "yhs_can_interfaces/msg/bms_info_fb.h"
// Member `bms_info_fb`
#include "yhs_can_interfaces/msg/detail/bms_info_fb__rosidl_typesupport_introspection_c.h"
// Member `drive_mcu_ecode_fb`
#include "yhs_can_interfaces/msg/drive_mcu_ecode_fb.h"
// Member `drive_mcu_ecode_fb`
#include "yhs_can_interfaces/msg/detail/drive_mcu_ecode_fb__rosidl_typesupport_introspection_c.h"
// Member `veh_diag_fb`
#include "yhs_can_interfaces/msg/veh_diag_fb.h"
// Member `veh_diag_fb`
#include "yhs_can_interfaces/msg/detail/veh_diag_fb__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yhs_can_interfaces__msg__ChassisInfoFb__init(message_memory);
}

void yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_fini_function(void * message_memory)
{
  yhs_can_interfaces__msg__ChassisInfoFb__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_member_array[9] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yhs_can_interfaces__msg__ChassisInfoFb, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ctrl_fb",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yhs_can_interfaces__msg__ChassisInfoFb, ctrl_fb),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "io_fb",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yhs_can_interfaces__msg__ChassisInfoFb, io_fb),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lr_wheel_fb",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yhs_can_interfaces__msg__ChassisInfoFb, lr_wheel_fb),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rr_wheel_fb",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yhs_can_interfaces__msg__ChassisInfoFb, rr_wheel_fb),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bms_flag_info_fb",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yhs_can_interfaces__msg__ChassisInfoFb, bms_flag_info_fb),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bms_info_fb",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yhs_can_interfaces__msg__ChassisInfoFb, bms_info_fb),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "drive_mcu_ecode_fb",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yhs_can_interfaces__msg__ChassisInfoFb, drive_mcu_ecode_fb),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "veh_diag_fb",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yhs_can_interfaces__msg__ChassisInfoFb, veh_diag_fb),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_members = {
  "yhs_can_interfaces__msg",  // message namespace
  "ChassisInfoFb",  // message name
  9,  // number of fields
  sizeof(yhs_can_interfaces__msg__ChassisInfoFb),
  yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_member_array,  // message members
  yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_init_function,  // function to initialize message memory (memory has to be allocated)
  yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_type_support_handle = {
  0,
  &yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yhs_can_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yhs_can_interfaces, msg, ChassisInfoFb)() {
  yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yhs_can_interfaces, msg, CtrlFb)();
  yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yhs_can_interfaces, msg, IoFb)();
  yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yhs_can_interfaces, msg, LrWheelFb)();
  yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yhs_can_interfaces, msg, RrWheelFb)();
  yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yhs_can_interfaces, msg, BmsFlagInfoFb)();
  yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yhs_can_interfaces, msg, BmsInfoFb)();
  yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yhs_can_interfaces, msg, DriveMcuEcodeFb)();
  yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_member_array[8].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yhs_can_interfaces, msg, VehDiagFb)();
  if (!yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_type_support_handle.typesupport_identifier) {
    yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &yhs_can_interfaces__msg__ChassisInfoFb__rosidl_typesupport_introspection_c__ChassisInfoFb_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
