// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yhs_can_interfaces:msg/BmsInfoFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__BMS_INFO_FB__TRAITS_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__BMS_INFO_FB__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yhs_can_interfaces/msg/detail/bms_info_fb__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace yhs_can_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const BmsInfoFb & msg,
  std::ostream & out)
{
  out << "{";
  // member: bms_info_voltage
  {
    out << "bms_info_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_info_voltage, out);
    out << ", ";
  }

  // member: bms_info_current
  {
    out << "bms_info_current: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_info_current, out);
    out << ", ";
  }

  // member: bms_info_remaining_capacity
  {
    out << "bms_info_remaining_capacity: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_info_remaining_capacity, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BmsInfoFb & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: bms_info_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_info_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_info_voltage, out);
    out << "\n";
  }

  // member: bms_info_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_info_current: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_info_current, out);
    out << "\n";
  }

  // member: bms_info_remaining_capacity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_info_remaining_capacity: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_info_remaining_capacity, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BmsInfoFb & msg, bool use_flow_style = false)
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
  const yhs_can_interfaces::msg::BmsInfoFb & msg,
  std::ostream & out, size_t indentation = 0)
{
  yhs_can_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yhs_can_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const yhs_can_interfaces::msg::BmsInfoFb & msg)
{
  return yhs_can_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yhs_can_interfaces::msg::BmsInfoFb>()
{
  return "yhs_can_interfaces::msg::BmsInfoFb";
}

template<>
inline const char * name<yhs_can_interfaces::msg::BmsInfoFb>()
{
  return "yhs_can_interfaces/msg/BmsInfoFb";
}

template<>
struct has_fixed_size<yhs_can_interfaces::msg::BmsInfoFb>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<yhs_can_interfaces::msg::BmsInfoFb>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<yhs_can_interfaces::msg::BmsInfoFb>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__BMS_INFO_FB__TRAITS_HPP_
