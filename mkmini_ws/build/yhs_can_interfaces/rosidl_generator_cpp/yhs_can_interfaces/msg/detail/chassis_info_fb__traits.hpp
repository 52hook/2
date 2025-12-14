// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yhs_can_interfaces:msg/ChassisInfoFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__CHASSIS_INFO_FB__TRAITS_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__CHASSIS_INFO_FB__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yhs_can_interfaces/msg/detail/chassis_info_fb__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'ctrl_fb'
#include "yhs_can_interfaces/msg/detail/ctrl_fb__traits.hpp"
// Member 'io_fb'
#include "yhs_can_interfaces/msg/detail/io_fb__traits.hpp"
// Member 'lr_wheel_fb'
#include "yhs_can_interfaces/msg/detail/lr_wheel_fb__traits.hpp"
// Member 'rr_wheel_fb'
#include "yhs_can_interfaces/msg/detail/rr_wheel_fb__traits.hpp"
// Member 'bms_flag_info_fb'
#include "yhs_can_interfaces/msg/detail/bms_flag_info_fb__traits.hpp"
// Member 'bms_info_fb'
#include "yhs_can_interfaces/msg/detail/bms_info_fb__traits.hpp"
// Member 'drive_mcu_ecode_fb'
#include "yhs_can_interfaces/msg/detail/drive_mcu_ecode_fb__traits.hpp"
// Member 'veh_diag_fb'
#include "yhs_can_interfaces/msg/detail/veh_diag_fb__traits.hpp"

namespace yhs_can_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ChassisInfoFb & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: ctrl_fb
  {
    out << "ctrl_fb: ";
    to_flow_style_yaml(msg.ctrl_fb, out);
    out << ", ";
  }

  // member: io_fb
  {
    out << "io_fb: ";
    to_flow_style_yaml(msg.io_fb, out);
    out << ", ";
  }

  // member: lr_wheel_fb
  {
    out << "lr_wheel_fb: ";
    to_flow_style_yaml(msg.lr_wheel_fb, out);
    out << ", ";
  }

  // member: rr_wheel_fb
  {
    out << "rr_wheel_fb: ";
    to_flow_style_yaml(msg.rr_wheel_fb, out);
    out << ", ";
  }

  // member: bms_flag_info_fb
  {
    out << "bms_flag_info_fb: ";
    to_flow_style_yaml(msg.bms_flag_info_fb, out);
    out << ", ";
  }

  // member: bms_info_fb
  {
    out << "bms_info_fb: ";
    to_flow_style_yaml(msg.bms_info_fb, out);
    out << ", ";
  }

  // member: drive_mcu_ecode_fb
  {
    out << "drive_mcu_ecode_fb: ";
    to_flow_style_yaml(msg.drive_mcu_ecode_fb, out);
    out << ", ";
  }

  // member: veh_diag_fb
  {
    out << "veh_diag_fb: ";
    to_flow_style_yaml(msg.veh_diag_fb, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChassisInfoFb & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: ctrl_fb
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ctrl_fb:\n";
    to_block_style_yaml(msg.ctrl_fb, out, indentation + 2);
  }

  // member: io_fb
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_fb:\n";
    to_block_style_yaml(msg.io_fb, out, indentation + 2);
  }

  // member: lr_wheel_fb
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lr_wheel_fb:\n";
    to_block_style_yaml(msg.lr_wheel_fb, out, indentation + 2);
  }

  // member: rr_wheel_fb
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rr_wheel_fb:\n";
    to_block_style_yaml(msg.rr_wheel_fb, out, indentation + 2);
  }

  // member: bms_flag_info_fb
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_fb:\n";
    to_block_style_yaml(msg.bms_flag_info_fb, out, indentation + 2);
  }

  // member: bms_info_fb
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_info_fb:\n";
    to_block_style_yaml(msg.bms_info_fb, out, indentation + 2);
  }

  // member: drive_mcu_ecode_fb
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drive_mcu_ecode_fb:\n";
    to_block_style_yaml(msg.drive_mcu_ecode_fb, out, indentation + 2);
  }

  // member: veh_diag_fb
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "veh_diag_fb:\n";
    to_block_style_yaml(msg.veh_diag_fb, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChassisInfoFb & msg, bool use_flow_style = false)
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
  const yhs_can_interfaces::msg::ChassisInfoFb & msg,
  std::ostream & out, size_t indentation = 0)
{
  yhs_can_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yhs_can_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const yhs_can_interfaces::msg::ChassisInfoFb & msg)
{
  return yhs_can_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yhs_can_interfaces::msg::ChassisInfoFb>()
{
  return "yhs_can_interfaces::msg::ChassisInfoFb";
}

template<>
inline const char * name<yhs_can_interfaces::msg::ChassisInfoFb>()
{
  return "yhs_can_interfaces/msg/ChassisInfoFb";
}

template<>
struct has_fixed_size<yhs_can_interfaces::msg::ChassisInfoFb>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value && has_fixed_size<yhs_can_interfaces::msg::BmsFlagInfoFb>::value && has_fixed_size<yhs_can_interfaces::msg::BmsInfoFb>::value && has_fixed_size<yhs_can_interfaces::msg::CtrlFb>::value && has_fixed_size<yhs_can_interfaces::msg::DriveMcuEcodeFb>::value && has_fixed_size<yhs_can_interfaces::msg::IoFb>::value && has_fixed_size<yhs_can_interfaces::msg::LrWheelFb>::value && has_fixed_size<yhs_can_interfaces::msg::RrWheelFb>::value && has_fixed_size<yhs_can_interfaces::msg::VehDiagFb>::value> {};

template<>
struct has_bounded_size<yhs_can_interfaces::msg::ChassisInfoFb>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value && has_bounded_size<yhs_can_interfaces::msg::BmsFlagInfoFb>::value && has_bounded_size<yhs_can_interfaces::msg::BmsInfoFb>::value && has_bounded_size<yhs_can_interfaces::msg::CtrlFb>::value && has_bounded_size<yhs_can_interfaces::msg::DriveMcuEcodeFb>::value && has_bounded_size<yhs_can_interfaces::msg::IoFb>::value && has_bounded_size<yhs_can_interfaces::msg::LrWheelFb>::value && has_bounded_size<yhs_can_interfaces::msg::RrWheelFb>::value && has_bounded_size<yhs_can_interfaces::msg::VehDiagFb>::value> {};

template<>
struct is_message<yhs_can_interfaces::msg::ChassisInfoFb>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__CHASSIS_INFO_FB__TRAITS_HPP_
