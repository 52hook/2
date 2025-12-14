// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yhs_can_interfaces:msg/IoFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__IO_FB__TRAITS_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__IO_FB__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yhs_can_interfaces/msg/detail/io_fb__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace yhs_can_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const IoFb & msg,
  std::ostream & out)
{
  out << "{";
  // member: io_fb_enable
  {
    out << "io_fb_enable: ";
    rosidl_generator_traits::value_to_yaml(msg.io_fb_enable, out);
    out << ", ";
  }

  // member: io_fb_turn_lamp
  {
    out << "io_fb_turn_lamp: ";
    rosidl_generator_traits::value_to_yaml(msg.io_fb_turn_lamp, out);
    out << ", ";
  }

  // member: io_fb_braking_lamp
  {
    out << "io_fb_braking_lamp: ";
    rosidl_generator_traits::value_to_yaml(msg.io_fb_braking_lamp, out);
    out << ", ";
  }

  // member: io_fb_fm_impact_sensor
  {
    out << "io_fb_fm_impact_sensor: ";
    rosidl_generator_traits::value_to_yaml(msg.io_fb_fm_impact_sensor, out);
    out << ", ";
  }

  // member: io_fb_rm_impact_sensor
  {
    out << "io_fb_rm_impact_sensor: ";
    rosidl_generator_traits::value_to_yaml(msg.io_fb_rm_impact_sensor, out);
    out << ", ";
  }

  // member: io_fb_dis_charge
  {
    out << "io_fb_dis_charge: ";
    rosidl_generator_traits::value_to_yaml(msg.io_fb_dis_charge, out);
    out << ", ";
  }

  // member: io_fb_charge_en
  {
    out << "io_fb_charge_en: ";
    rosidl_generator_traits::value_to_yaml(msg.io_fb_charge_en, out);
    out << ", ";
  }

  // member: io_fb_scram_st
  {
    out << "io_fb_scram_st: ";
    rosidl_generator_traits::value_to_yaml(msg.io_fb_scram_st, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IoFb & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: io_fb_enable
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_fb_enable: ";
    rosidl_generator_traits::value_to_yaml(msg.io_fb_enable, out);
    out << "\n";
  }

  // member: io_fb_turn_lamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_fb_turn_lamp: ";
    rosidl_generator_traits::value_to_yaml(msg.io_fb_turn_lamp, out);
    out << "\n";
  }

  // member: io_fb_braking_lamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_fb_braking_lamp: ";
    rosidl_generator_traits::value_to_yaml(msg.io_fb_braking_lamp, out);
    out << "\n";
  }

  // member: io_fb_fm_impact_sensor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_fb_fm_impact_sensor: ";
    rosidl_generator_traits::value_to_yaml(msg.io_fb_fm_impact_sensor, out);
    out << "\n";
  }

  // member: io_fb_rm_impact_sensor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_fb_rm_impact_sensor: ";
    rosidl_generator_traits::value_to_yaml(msg.io_fb_rm_impact_sensor, out);
    out << "\n";
  }

  // member: io_fb_dis_charge
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_fb_dis_charge: ";
    rosidl_generator_traits::value_to_yaml(msg.io_fb_dis_charge, out);
    out << "\n";
  }

  // member: io_fb_charge_en
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_fb_charge_en: ";
    rosidl_generator_traits::value_to_yaml(msg.io_fb_charge_en, out);
    out << "\n";
  }

  // member: io_fb_scram_st
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_fb_scram_st: ";
    rosidl_generator_traits::value_to_yaml(msg.io_fb_scram_st, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IoFb & msg, bool use_flow_style = false)
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
  const yhs_can_interfaces::msg::IoFb & msg,
  std::ostream & out, size_t indentation = 0)
{
  yhs_can_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yhs_can_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const yhs_can_interfaces::msg::IoFb & msg)
{
  return yhs_can_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yhs_can_interfaces::msg::IoFb>()
{
  return "yhs_can_interfaces::msg::IoFb";
}

template<>
inline const char * name<yhs_can_interfaces::msg::IoFb>()
{
  return "yhs_can_interfaces/msg/IoFb";
}

template<>
struct has_fixed_size<yhs_can_interfaces::msg::IoFb>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<yhs_can_interfaces::msg::IoFb>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<yhs_can_interfaces::msg::IoFb>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__IO_FB__TRAITS_HPP_
