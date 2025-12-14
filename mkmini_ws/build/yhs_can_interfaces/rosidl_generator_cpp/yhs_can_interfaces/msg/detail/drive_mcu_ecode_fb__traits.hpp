// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yhs_can_interfaces:msg/DriveMcuEcodeFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__DRIVE_MCU_ECODE_FB__TRAITS_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__DRIVE_MCU_ECODE_FB__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yhs_can_interfaces/msg/detail/drive_mcu_ecode_fb__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace yhs_can_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const DriveMcuEcodeFb & msg,
  std::ostream & out)
{
  out << "{";
  // member: drive_fb_mcuecode
  {
    out << "drive_fb_mcuecode: ";
    rosidl_generator_traits::value_to_yaml(msg.drive_fb_mcuecode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DriveMcuEcodeFb & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: drive_fb_mcuecode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drive_fb_mcuecode: ";
    rosidl_generator_traits::value_to_yaml(msg.drive_fb_mcuecode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DriveMcuEcodeFb & msg, bool use_flow_style = false)
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
  const yhs_can_interfaces::msg::DriveMcuEcodeFb & msg,
  std::ostream & out, size_t indentation = 0)
{
  yhs_can_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yhs_can_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const yhs_can_interfaces::msg::DriveMcuEcodeFb & msg)
{
  return yhs_can_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yhs_can_interfaces::msg::DriveMcuEcodeFb>()
{
  return "yhs_can_interfaces::msg::DriveMcuEcodeFb";
}

template<>
inline const char * name<yhs_can_interfaces::msg::DriveMcuEcodeFb>()
{
  return "yhs_can_interfaces/msg/DriveMcuEcodeFb";
}

template<>
struct has_fixed_size<yhs_can_interfaces::msg::DriveMcuEcodeFb>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<yhs_can_interfaces::msg::DriveMcuEcodeFb>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<yhs_can_interfaces::msg::DriveMcuEcodeFb>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__DRIVE_MCU_ECODE_FB__TRAITS_HPP_
