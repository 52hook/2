// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yhs_can_interfaces:msg/IoCmd.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__IO_CMD__TRAITS_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__IO_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yhs_can_interfaces/msg/detail/io_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace yhs_can_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const IoCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: io_cmd_enable
  {
    out << "io_cmd_enable: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_enable, out);
    out << ", ";
  }

  // member: io_cmd_lower_beam_headlamp
  {
    out << "io_cmd_lower_beam_headlamp: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_lower_beam_headlamp, out);
    out << ", ";
  }

  // member: io_cmd_upper_beam_headlamp
  {
    out << "io_cmd_upper_beam_headlamp: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_upper_beam_headlamp, out);
    out << ", ";
  }

  // member: io_cmd_turn_lamp
  {
    out << "io_cmd_turn_lamp: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_turn_lamp, out);
    out << ", ";
  }

  // member: io_cmd_braking_lamp
  {
    out << "io_cmd_braking_lamp: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_braking_lamp, out);
    out << ", ";
  }

  // member: io_cmd_clearance_lamp
  {
    out << "io_cmd_clearance_lamp: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_clearance_lamp, out);
    out << ", ";
  }

  // member: io_cmd_fog_lamp
  {
    out << "io_cmd_fog_lamp: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_fog_lamp, out);
    out << ", ";
  }

  // member: io_cmd_speaker
  {
    out << "io_cmd_speaker: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_speaker, out);
    out << ", ";
  }

  // member: io_cmd_dis_charge
  {
    out << "io_cmd_dis_charge: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_dis_charge, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IoCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: io_cmd_enable
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_cmd_enable: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_enable, out);
    out << "\n";
  }

  // member: io_cmd_lower_beam_headlamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_cmd_lower_beam_headlamp: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_lower_beam_headlamp, out);
    out << "\n";
  }

  // member: io_cmd_upper_beam_headlamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_cmd_upper_beam_headlamp: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_upper_beam_headlamp, out);
    out << "\n";
  }

  // member: io_cmd_turn_lamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_cmd_turn_lamp: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_turn_lamp, out);
    out << "\n";
  }

  // member: io_cmd_braking_lamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_cmd_braking_lamp: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_braking_lamp, out);
    out << "\n";
  }

  // member: io_cmd_clearance_lamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_cmd_clearance_lamp: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_clearance_lamp, out);
    out << "\n";
  }

  // member: io_cmd_fog_lamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_cmd_fog_lamp: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_fog_lamp, out);
    out << "\n";
  }

  // member: io_cmd_speaker
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_cmd_speaker: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_speaker, out);
    out << "\n";
  }

  // member: io_cmd_dis_charge
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_cmd_dis_charge: ";
    rosidl_generator_traits::value_to_yaml(msg.io_cmd_dis_charge, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IoCmd & msg, bool use_flow_style = false)
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
  const yhs_can_interfaces::msg::IoCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  yhs_can_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yhs_can_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const yhs_can_interfaces::msg::IoCmd & msg)
{
  return yhs_can_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yhs_can_interfaces::msg::IoCmd>()
{
  return "yhs_can_interfaces::msg::IoCmd";
}

template<>
inline const char * name<yhs_can_interfaces::msg::IoCmd>()
{
  return "yhs_can_interfaces/msg/IoCmd";
}

template<>
struct has_fixed_size<yhs_can_interfaces::msg::IoCmd>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<yhs_can_interfaces::msg::IoCmd>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<yhs_can_interfaces::msg::IoCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__IO_CMD__TRAITS_HPP_
