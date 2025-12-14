// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from yhs_can_interfaces:msg/VehDiagFb.idl
// generated code does not contain a copyright notice
#include "yhs_can_interfaces/msg/detail/veh_diag_fb__rosidl_typesupport_fastrtps_cpp.hpp"
#include "yhs_can_interfaces/msg/detail/veh_diag_fb__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace yhs_can_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yhs_can_interfaces
cdr_serialize(
  const yhs_can_interfaces::msg::VehDiagFb & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: veh_fb_fault_level
  cdr << ros_message.veh_fb_fault_level;
  // Member: veh_fb_auto_can_ctrl_cmd
  cdr << (ros_message.veh_fb_auto_can_ctrl_cmd ? true : false);
  // Member: veh_fb_auto_io_can_cmd
  cdr << (ros_message.veh_fb_auto_io_can_cmd ? true : false);
  // Member: veh_fb_eps_dis_on_line
  cdr << (ros_message.veh_fb_eps_dis_on_line ? true : false);
  // Member: veh_fb_eps_fault
  cdr << (ros_message.veh_fb_eps_fault ? true : false);
  // Member: veh_fb_eps_mosf_et_ot
  cdr << (ros_message.veh_fb_eps_mosf_et_ot ? true : false);
  // Member: veh_fb_eps_warning
  cdr << (ros_message.veh_fb_eps_warning ? true : false);
  // Member: veh_fb_eps_dis_work
  cdr << (ros_message.veh_fb_eps_dis_work ? true : false);
  // Member: veh_fb_eps_over_current
  cdr << (ros_message.veh_fb_eps_over_current ? true : false);
  // Member: veh_fb_st_reserve
  cdr << (ros_message.veh_fb_st_reserve ? true : false);
  // Member: veh_fb_ehb_ecu_fault
  cdr << (ros_message.veh_fb_ehb_ecu_fault ? true : false);
  // Member: veh_fb_ehb_dis_on_line
  cdr << (ros_message.veh_fb_ehb_dis_on_line ? true : false);
  // Member: veh_fb_ehb_work_model_fault
  cdr << (ros_message.veh_fb_ehb_work_model_fault ? true : false);
  // Member: veh_fb_ehb_dis_en
  cdr << (ros_message.veh_fb_ehb_dis_en ? true : false);
  // Member: veh_fb_ehb_anguler_fault
  cdr << (ros_message.veh_fb_ehb_anguler_fault ? true : false);
  // Member: veh_fb_ehb_ot
  cdr << (ros_message.veh_fb_ehb_ot ? true : false);
  // Member: veh_fb_ehb_power_fault
  cdr << (ros_message.veh_fb_ehb_power_fault ? true : false);
  // Member: veh_fb_ehb_sensor_abnomal
  cdr << (ros_message.veh_fb_ehb_sensor_abnomal ? true : false);
  // Member: veh_fb_ehb_motor_fault
  cdr << (ros_message.veh_fb_ehb_motor_fault ? true : false);
  // Member: veh_fb_ehb_oil_press_sensor_fault
  cdr << (ros_message.veh_fb_ehb_oil_press_sensor_fault ? true : false);
  // Member: veh_fb_ehb_oil_fault
  cdr << (ros_message.veh_fb_ehb_oil_fault ? true : false);
  // Member: veh_fb_ld_rv_mcu_fault
  cdr << ros_message.veh_fb_ld_rv_mcu_fault;
  // Member: veh_fb_rd_rv_mcu_fault
  cdr << ros_message.veh_fb_rd_rv_mcu_fault;
  // Member: veh_fb_aux_bms_dis_on_line
  cdr << (ros_message.veh_fb_aux_bms_dis_on_line ? true : false);
  // Member: veh_fb_aux_scram
  cdr << (ros_message.veh_fb_aux_scram ? true : false);
  // Member: veh_fb_aux_remote_close
  cdr << (ros_message.veh_fb_aux_remote_close ? true : false);
  // Member: veh_fb_aux_remote_dis_on_line
  cdr << (ros_message.veh_fb_aux_remote_dis_on_line ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yhs_can_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  yhs_can_interfaces::msg::VehDiagFb & ros_message)
{
  // Member: veh_fb_fault_level
  cdr >> ros_message.veh_fb_fault_level;

  // Member: veh_fb_auto_can_ctrl_cmd
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_auto_can_ctrl_cmd = tmp ? true : false;
  }

  // Member: veh_fb_auto_io_can_cmd
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_auto_io_can_cmd = tmp ? true : false;
  }

  // Member: veh_fb_eps_dis_on_line
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_eps_dis_on_line = tmp ? true : false;
  }

  // Member: veh_fb_eps_fault
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_eps_fault = tmp ? true : false;
  }

  // Member: veh_fb_eps_mosf_et_ot
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_eps_mosf_et_ot = tmp ? true : false;
  }

  // Member: veh_fb_eps_warning
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_eps_warning = tmp ? true : false;
  }

  // Member: veh_fb_eps_dis_work
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_eps_dis_work = tmp ? true : false;
  }

  // Member: veh_fb_eps_over_current
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_eps_over_current = tmp ? true : false;
  }

  // Member: veh_fb_st_reserve
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_st_reserve = tmp ? true : false;
  }

  // Member: veh_fb_ehb_ecu_fault
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_ehb_ecu_fault = tmp ? true : false;
  }

  // Member: veh_fb_ehb_dis_on_line
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_ehb_dis_on_line = tmp ? true : false;
  }

  // Member: veh_fb_ehb_work_model_fault
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_ehb_work_model_fault = tmp ? true : false;
  }

  // Member: veh_fb_ehb_dis_en
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_ehb_dis_en = tmp ? true : false;
  }

  // Member: veh_fb_ehb_anguler_fault
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_ehb_anguler_fault = tmp ? true : false;
  }

  // Member: veh_fb_ehb_ot
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_ehb_ot = tmp ? true : false;
  }

  // Member: veh_fb_ehb_power_fault
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_ehb_power_fault = tmp ? true : false;
  }

  // Member: veh_fb_ehb_sensor_abnomal
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_ehb_sensor_abnomal = tmp ? true : false;
  }

  // Member: veh_fb_ehb_motor_fault
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_ehb_motor_fault = tmp ? true : false;
  }

  // Member: veh_fb_ehb_oil_press_sensor_fault
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_ehb_oil_press_sensor_fault = tmp ? true : false;
  }

  // Member: veh_fb_ehb_oil_fault
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_ehb_oil_fault = tmp ? true : false;
  }

  // Member: veh_fb_ld_rv_mcu_fault
  cdr >> ros_message.veh_fb_ld_rv_mcu_fault;

  // Member: veh_fb_rd_rv_mcu_fault
  cdr >> ros_message.veh_fb_rd_rv_mcu_fault;

  // Member: veh_fb_aux_bms_dis_on_line
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_aux_bms_dis_on_line = tmp ? true : false;
  }

  // Member: veh_fb_aux_scram
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_aux_scram = tmp ? true : false;
  }

  // Member: veh_fb_aux_remote_close
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_aux_remote_close = tmp ? true : false;
  }

  // Member: veh_fb_aux_remote_dis_on_line
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.veh_fb_aux_remote_dis_on_line = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yhs_can_interfaces
get_serialized_size(
  const yhs_can_interfaces::msg::VehDiagFb & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: veh_fb_fault_level
  {
    size_t item_size = sizeof(ros_message.veh_fb_fault_level);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_auto_can_ctrl_cmd
  {
    size_t item_size = sizeof(ros_message.veh_fb_auto_can_ctrl_cmd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_auto_io_can_cmd
  {
    size_t item_size = sizeof(ros_message.veh_fb_auto_io_can_cmd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_eps_dis_on_line
  {
    size_t item_size = sizeof(ros_message.veh_fb_eps_dis_on_line);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_eps_fault
  {
    size_t item_size = sizeof(ros_message.veh_fb_eps_fault);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_eps_mosf_et_ot
  {
    size_t item_size = sizeof(ros_message.veh_fb_eps_mosf_et_ot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_eps_warning
  {
    size_t item_size = sizeof(ros_message.veh_fb_eps_warning);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_eps_dis_work
  {
    size_t item_size = sizeof(ros_message.veh_fb_eps_dis_work);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_eps_over_current
  {
    size_t item_size = sizeof(ros_message.veh_fb_eps_over_current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_st_reserve
  {
    size_t item_size = sizeof(ros_message.veh_fb_st_reserve);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_ehb_ecu_fault
  {
    size_t item_size = sizeof(ros_message.veh_fb_ehb_ecu_fault);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_ehb_dis_on_line
  {
    size_t item_size = sizeof(ros_message.veh_fb_ehb_dis_on_line);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_ehb_work_model_fault
  {
    size_t item_size = sizeof(ros_message.veh_fb_ehb_work_model_fault);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_ehb_dis_en
  {
    size_t item_size = sizeof(ros_message.veh_fb_ehb_dis_en);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_ehb_anguler_fault
  {
    size_t item_size = sizeof(ros_message.veh_fb_ehb_anguler_fault);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_ehb_ot
  {
    size_t item_size = sizeof(ros_message.veh_fb_ehb_ot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_ehb_power_fault
  {
    size_t item_size = sizeof(ros_message.veh_fb_ehb_power_fault);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_ehb_sensor_abnomal
  {
    size_t item_size = sizeof(ros_message.veh_fb_ehb_sensor_abnomal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_ehb_motor_fault
  {
    size_t item_size = sizeof(ros_message.veh_fb_ehb_motor_fault);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_ehb_oil_press_sensor_fault
  {
    size_t item_size = sizeof(ros_message.veh_fb_ehb_oil_press_sensor_fault);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_ehb_oil_fault
  {
    size_t item_size = sizeof(ros_message.veh_fb_ehb_oil_fault);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_ld_rv_mcu_fault
  {
    size_t item_size = sizeof(ros_message.veh_fb_ld_rv_mcu_fault);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_rd_rv_mcu_fault
  {
    size_t item_size = sizeof(ros_message.veh_fb_rd_rv_mcu_fault);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_aux_bms_dis_on_line
  {
    size_t item_size = sizeof(ros_message.veh_fb_aux_bms_dis_on_line);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_aux_scram
  {
    size_t item_size = sizeof(ros_message.veh_fb_aux_scram);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_aux_remote_close
  {
    size_t item_size = sizeof(ros_message.veh_fb_aux_remote_close);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: veh_fb_aux_remote_dis_on_line
  {
    size_t item_size = sizeof(ros_message.veh_fb_aux_remote_dis_on_line);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yhs_can_interfaces
max_serialized_size_VehDiagFb(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: veh_fb_fault_level
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_auto_can_ctrl_cmd
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_auto_io_can_cmd
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_eps_dis_on_line
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_eps_fault
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_eps_mosf_et_ot
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_eps_warning
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_eps_dis_work
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_eps_over_current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_st_reserve
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_ehb_ecu_fault
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_ehb_dis_on_line
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_ehb_work_model_fault
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_ehb_dis_en
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_ehb_anguler_fault
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_ehb_ot
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_ehb_power_fault
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_ehb_sensor_abnomal
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_ehb_motor_fault
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_ehb_oil_press_sensor_fault
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_ehb_oil_fault
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_ld_rv_mcu_fault
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_rd_rv_mcu_fault
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_aux_bms_dis_on_line
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_aux_scram
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_aux_remote_close
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: veh_fb_aux_remote_dis_on_line
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = yhs_can_interfaces::msg::VehDiagFb;
    is_plain =
      (
      offsetof(DataType, veh_fb_aux_remote_dis_on_line) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _VehDiagFb__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const yhs_can_interfaces::msg::VehDiagFb *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _VehDiagFb__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<yhs_can_interfaces::msg::VehDiagFb *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _VehDiagFb__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const yhs_can_interfaces::msg::VehDiagFb *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _VehDiagFb__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_VehDiagFb(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _VehDiagFb__callbacks = {
  "yhs_can_interfaces::msg",
  "VehDiagFb",
  _VehDiagFb__cdr_serialize,
  _VehDiagFb__cdr_deserialize,
  _VehDiagFb__get_serialized_size,
  _VehDiagFb__max_serialized_size
};

static rosidl_message_type_support_t _VehDiagFb__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_VehDiagFb__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace yhs_can_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_yhs_can_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<yhs_can_interfaces::msg::VehDiagFb>()
{
  return &yhs_can_interfaces::msg::typesupport_fastrtps_cpp::_VehDiagFb__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, yhs_can_interfaces, msg, VehDiagFb)() {
  return &yhs_can_interfaces::msg::typesupport_fastrtps_cpp::_VehDiagFb__handle;
}

#ifdef __cplusplus
}
#endif
