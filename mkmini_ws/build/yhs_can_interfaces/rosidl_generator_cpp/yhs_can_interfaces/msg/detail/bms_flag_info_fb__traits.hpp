// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yhs_can_interfaces:msg/BmsFlagInfoFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__BMS_FLAG_INFO_FB__TRAITS_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__BMS_FLAG_INFO_FB__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yhs_can_interfaces/msg/detail/bms_flag_info_fb__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace yhs_can_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const BmsFlagInfoFb & msg,
  std::ostream & out)
{
  out << "{";
  // member: bms_flag_info_soc
  {
    out << "bms_flag_info_soc: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_soc, out);
    out << ", ";
  }

  // member: bms_flag_info_single_ov
  {
    out << "bms_flag_info_single_ov: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_single_ov, out);
    out << ", ";
  }

  // member: bms_flag_info_single_uv
  {
    out << "bms_flag_info_single_uv: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_single_uv, out);
    out << ", ";
  }

  // member: bms_flag_info_ov
  {
    out << "bms_flag_info_ov: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_ov, out);
    out << ", ";
  }

  // member: bms_flag_info_uv
  {
    out << "bms_flag_info_uv: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_uv, out);
    out << ", ";
  }

  // member: bms_flag_info_charge_ot
  {
    out << "bms_flag_info_charge_ot: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_charge_ot, out);
    out << ", ";
  }

  // member: bms_flag_info_charge_ut
  {
    out << "bms_flag_info_charge_ut: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_charge_ut, out);
    out << ", ";
  }

  // member: bms_flag_info_discharge_ot
  {
    out << "bms_flag_info_discharge_ot: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_discharge_ot, out);
    out << ", ";
  }

  // member: bms_flag_info_discharge_ut
  {
    out << "bms_flag_info_discharge_ut: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_discharge_ut, out);
    out << ", ";
  }

  // member: bms_flag_info_charge_oc
  {
    out << "bms_flag_info_charge_oc: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_charge_oc, out);
    out << ", ";
  }

  // member: bms_flag_info_discharge_oc
  {
    out << "bms_flag_info_discharge_oc: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_discharge_oc, out);
    out << ", ";
  }

  // member: bms_flag_info_short
  {
    out << "bms_flag_info_short: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_short, out);
    out << ", ";
  }

  // member: bms_flag_info_ic_error
  {
    out << "bms_flag_info_ic_error: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_ic_error, out);
    out << ", ";
  }

  // member: bms_flag_info_lock_mos
  {
    out << "bms_flag_info_lock_mos: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_lock_mos, out);
    out << ", ";
  }

  // member: bms_flag_info_charge_flag
  {
    out << "bms_flag_info_charge_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_charge_flag, out);
    out << ", ";
  }

  // member: bms_flag_info_soc_warning
  {
    out << "bms_flag_info_soc_warning: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_soc_warning, out);
    out << ", ";
  }

  // member: bms_flag_info_soc_low_protection
  {
    out << "bms_flag_info_soc_low_protection: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_soc_low_protection, out);
    out << ", ";
  }

  // member: bms_flag_info_hight_temperature
  {
    out << "bms_flag_info_hight_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_hight_temperature, out);
    out << ", ";
  }

  // member: bms_flag_info_low_temperature
  {
    out << "bms_flag_info_low_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_low_temperature, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BmsFlagInfoFb & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: bms_flag_info_soc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_soc: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_soc, out);
    out << "\n";
  }

  // member: bms_flag_info_single_ov
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_single_ov: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_single_ov, out);
    out << "\n";
  }

  // member: bms_flag_info_single_uv
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_single_uv: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_single_uv, out);
    out << "\n";
  }

  // member: bms_flag_info_ov
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_ov: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_ov, out);
    out << "\n";
  }

  // member: bms_flag_info_uv
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_uv: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_uv, out);
    out << "\n";
  }

  // member: bms_flag_info_charge_ot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_charge_ot: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_charge_ot, out);
    out << "\n";
  }

  // member: bms_flag_info_charge_ut
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_charge_ut: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_charge_ut, out);
    out << "\n";
  }

  // member: bms_flag_info_discharge_ot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_discharge_ot: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_discharge_ot, out);
    out << "\n";
  }

  // member: bms_flag_info_discharge_ut
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_discharge_ut: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_discharge_ut, out);
    out << "\n";
  }

  // member: bms_flag_info_charge_oc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_charge_oc: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_charge_oc, out);
    out << "\n";
  }

  // member: bms_flag_info_discharge_oc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_discharge_oc: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_discharge_oc, out);
    out << "\n";
  }

  // member: bms_flag_info_short
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_short: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_short, out);
    out << "\n";
  }

  // member: bms_flag_info_ic_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_ic_error: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_ic_error, out);
    out << "\n";
  }

  // member: bms_flag_info_lock_mos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_lock_mos: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_lock_mos, out);
    out << "\n";
  }

  // member: bms_flag_info_charge_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_charge_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_charge_flag, out);
    out << "\n";
  }

  // member: bms_flag_info_soc_warning
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_soc_warning: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_soc_warning, out);
    out << "\n";
  }

  // member: bms_flag_info_soc_low_protection
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_soc_low_protection: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_soc_low_protection, out);
    out << "\n";
  }

  // member: bms_flag_info_hight_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_hight_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_hight_temperature, out);
    out << "\n";
  }

  // member: bms_flag_info_low_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bms_flag_info_low_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.bms_flag_info_low_temperature, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BmsFlagInfoFb & msg, bool use_flow_style = false)
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
  const yhs_can_interfaces::msg::BmsFlagInfoFb & msg,
  std::ostream & out, size_t indentation = 0)
{
  yhs_can_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yhs_can_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const yhs_can_interfaces::msg::BmsFlagInfoFb & msg)
{
  return yhs_can_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yhs_can_interfaces::msg::BmsFlagInfoFb>()
{
  return "yhs_can_interfaces::msg::BmsFlagInfoFb";
}

template<>
inline const char * name<yhs_can_interfaces::msg::BmsFlagInfoFb>()
{
  return "yhs_can_interfaces/msg/BmsFlagInfoFb";
}

template<>
struct has_fixed_size<yhs_can_interfaces::msg::BmsFlagInfoFb>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<yhs_can_interfaces::msg::BmsFlagInfoFb>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<yhs_can_interfaces::msg::BmsFlagInfoFb>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__BMS_FLAG_INFO_FB__TRAITS_HPP_
