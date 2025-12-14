// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yhs_can_interfaces:msg/VehDiagFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__VEH_DIAG_FB__TRAITS_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__VEH_DIAG_FB__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yhs_can_interfaces/msg/detail/veh_diag_fb__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace yhs_can_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const VehDiagFb & msg,
  std::ostream & out)
{
  out << "{";
  // member: veh_fb_fault_level
  {
    out << "veh_fb_fault_level: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_fault_level, out);
    out << ", ";
  }

  // member: veh_fb_auto_can_ctrl_cmd
  {
    out << "veh_fb_auto_can_ctrl_cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_auto_can_ctrl_cmd, out);
    out << ", ";
  }

  // member: veh_fb_auto_io_can_cmd
  {
    out << "veh_fb_auto_io_can_cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_auto_io_can_cmd, out);
    out << ", ";
  }

  // member: veh_fb_eps_dis_on_line
  {
    out << "veh_fb_eps_dis_on_line: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_eps_dis_on_line, out);
    out << ", ";
  }

  // member: veh_fb_eps_fault
  {
    out << "veh_fb_eps_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_eps_fault, out);
    out << ", ";
  }

  // member: veh_fb_eps_mosf_et_ot
  {
    out << "veh_fb_eps_mosf_et_ot: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_eps_mosf_et_ot, out);
    out << ", ";
  }

  // member: veh_fb_eps_warning
  {
    out << "veh_fb_eps_warning: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_eps_warning, out);
    out << ", ";
  }

  // member: veh_fb_eps_dis_work
  {
    out << "veh_fb_eps_dis_work: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_eps_dis_work, out);
    out << ", ";
  }

  // member: veh_fb_eps_over_current
  {
    out << "veh_fb_eps_over_current: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_eps_over_current, out);
    out << ", ";
  }

  // member: veh_fb_st_reserve
  {
    out << "veh_fb_st_reserve: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_st_reserve, out);
    out << ", ";
  }

  // member: veh_fb_ehb_ecu_fault
  {
    out << "veh_fb_ehb_ecu_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_ecu_fault, out);
    out << ", ";
  }

  // member: veh_fb_ehb_dis_on_line
  {
    out << "veh_fb_ehb_dis_on_line: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_dis_on_line, out);
    out << ", ";
  }

  // member: veh_fb_ehb_work_model_fault
  {
    out << "veh_fb_ehb_work_model_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_work_model_fault, out);
    out << ", ";
  }

  // member: veh_fb_ehb_dis_en
  {
    out << "veh_fb_ehb_dis_en: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_dis_en, out);
    out << ", ";
  }

  // member: veh_fb_ehb_anguler_fault
  {
    out << "veh_fb_ehb_anguler_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_anguler_fault, out);
    out << ", ";
  }

  // member: veh_fb_ehb_ot
  {
    out << "veh_fb_ehb_ot: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_ot, out);
    out << ", ";
  }

  // member: veh_fb_ehb_power_fault
  {
    out << "veh_fb_ehb_power_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_power_fault, out);
    out << ", ";
  }

  // member: veh_fb_ehb_sensor_abnomal
  {
    out << "veh_fb_ehb_sensor_abnomal: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_sensor_abnomal, out);
    out << ", ";
  }

  // member: veh_fb_ehb_motor_fault
  {
    out << "veh_fb_ehb_motor_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_motor_fault, out);
    out << ", ";
  }

  // member: veh_fb_ehb_oil_press_sensor_fault
  {
    out << "veh_fb_ehb_oil_press_sensor_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_oil_press_sensor_fault, out);
    out << ", ";
  }

  // member: veh_fb_ehb_oil_fault
  {
    out << "veh_fb_ehb_oil_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_oil_fault, out);
    out << ", ";
  }

  // member: veh_fb_ld_rv_mcu_fault
  {
    out << "veh_fb_ld_rv_mcu_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ld_rv_mcu_fault, out);
    out << ", ";
  }

  // member: veh_fb_rd_rv_mcu_fault
  {
    out << "veh_fb_rd_rv_mcu_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_rd_rv_mcu_fault, out);
    out << ", ";
  }

  // member: veh_fb_aux_bms_dis_on_line
  {
    out << "veh_fb_aux_bms_dis_on_line: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_aux_bms_dis_on_line, out);
    out << ", ";
  }

  // member: veh_fb_aux_scram
  {
    out << "veh_fb_aux_scram: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_aux_scram, out);
    out << ", ";
  }

  // member: veh_fb_aux_remote_close
  {
    out << "veh_fb_aux_remote_close: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_aux_remote_close, out);
    out << ", ";
  }

  // member: veh_fb_aux_remote_dis_on_line
  {
    out << "veh_fb_aux_remote_dis_on_line: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_aux_remote_dis_on_line, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VehDiagFb & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: veh_fb_fault_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_fault_level: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_fault_level, out);
    out << "\n";
  }

  // member: veh_fb_auto_can_ctrl_cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_auto_can_ctrl_cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_auto_can_ctrl_cmd, out);
    out << "\n";
  }

  // member: veh_fb_auto_io_can_cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_auto_io_can_cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_auto_io_can_cmd, out);
    out << "\n";
  }

  // member: veh_fb_eps_dis_on_line
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_eps_dis_on_line: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_eps_dis_on_line, out);
    out << "\n";
  }

  // member: veh_fb_eps_fault
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_eps_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_eps_fault, out);
    out << "\n";
  }

  // member: veh_fb_eps_mosf_et_ot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_eps_mosf_et_ot: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_eps_mosf_et_ot, out);
    out << "\n";
  }

  // member: veh_fb_eps_warning
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_eps_warning: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_eps_warning, out);
    out << "\n";
  }

  // member: veh_fb_eps_dis_work
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_eps_dis_work: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_eps_dis_work, out);
    out << "\n";
  }

  // member: veh_fb_eps_over_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_eps_over_current: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_eps_over_current, out);
    out << "\n";
  }

  // member: veh_fb_st_reserve
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_st_reserve: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_st_reserve, out);
    out << "\n";
  }

  // member: veh_fb_ehb_ecu_fault
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_ehb_ecu_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_ecu_fault, out);
    out << "\n";
  }

  // member: veh_fb_ehb_dis_on_line
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_ehb_dis_on_line: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_dis_on_line, out);
    out << "\n";
  }

  // member: veh_fb_ehb_work_model_fault
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_ehb_work_model_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_work_model_fault, out);
    out << "\n";
  }

  // member: veh_fb_ehb_dis_en
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_ehb_dis_en: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_dis_en, out);
    out << "\n";
  }

  // member: veh_fb_ehb_anguler_fault
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_ehb_anguler_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_anguler_fault, out);
    out << "\n";
  }

  // member: veh_fb_ehb_ot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_ehb_ot: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_ot, out);
    out << "\n";
  }

  // member: veh_fb_ehb_power_fault
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_ehb_power_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_power_fault, out);
    out << "\n";
  }

  // member: veh_fb_ehb_sensor_abnomal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_ehb_sensor_abnomal: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_sensor_abnomal, out);
    out << "\n";
  }

  // member: veh_fb_ehb_motor_fault
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_ehb_motor_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_motor_fault, out);
    out << "\n";
  }

  // member: veh_fb_ehb_oil_press_sensor_fault
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_ehb_oil_press_sensor_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_oil_press_sensor_fault, out);
    out << "\n";
  }

  // member: veh_fb_ehb_oil_fault
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_ehb_oil_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ehb_oil_fault, out);
    out << "\n";
  }

  // member: veh_fb_ld_rv_mcu_fault
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_ld_rv_mcu_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_ld_rv_mcu_fault, out);
    out << "\n";
  }

  // member: veh_fb_rd_rv_mcu_fault
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_rd_rv_mcu_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_rd_rv_mcu_fault, out);
    out << "\n";
  }

  // member: veh_fb_aux_bms_dis_on_line
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_aux_bms_dis_on_line: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_aux_bms_dis_on_line, out);
    out << "\n";
  }

  // member: veh_fb_aux_scram
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_aux_scram: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_aux_scram, out);
    out << "\n";
  }

  // member: veh_fb_aux_remote_close
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_aux_remote_close: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_aux_remote_close, out);
    out << "\n";
  }

  // member: veh_fb_aux_remote_dis_on_line
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_fb_aux_remote_dis_on_line: ";
    rosidl_generator_traits::value_to_yaml(msg.veh_fb_aux_remote_dis_on_line, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VehDiagFb & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace yhs_can_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use yhs_can_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yhs_can_interfaces::msg::VehDiagFb & msg,
  std::ostream & out, size_t indentation = 0)
{
  yhs_can_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yhs_can_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const yhs_can_interfaces::msg::VehDiagFb & msg)
{
  return yhs_can_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yhs_can_interfaces::msg::VehDiagFb>()
{
  return "yhs_can_interfaces::msg::VehDiagFb";
}

template<>
inline const char * name<yhs_can_interfaces::msg::VehDiagFb>()
{
  return "yhs_can_interfaces/msg/VehDiagFb";
}

template<>
struct has_fixed_size<yhs_can_interfaces::msg::VehDiagFb>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<yhs_can_interfaces::msg::VehDiagFb>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<yhs_can_interfaces::msg::VehDiagFb>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__VEH_DIAG_FB__TRAITS_HPP_
