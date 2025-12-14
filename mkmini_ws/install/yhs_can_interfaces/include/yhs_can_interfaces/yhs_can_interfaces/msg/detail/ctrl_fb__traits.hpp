// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yhs_can_interfaces:msg/CtrlFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__CTRL_FB__TRAITS_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__CTRL_FB__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yhs_can_interfaces/msg/detail/ctrl_fb__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace yhs_can_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const CtrlFb & msg,
  std::ostream & out)
{
  out << "{";
  // member: ctrl_fb_gear
  {
    out << "ctrl_fb_gear: ";
    rosidl_generator_traits::value_to_yaml(msg.ctrl_fb_gear, out);
    out << ", ";
  }

  // member: ctrl_fb_velocity
  {
    out << "ctrl_fb_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.ctrl_fb_velocity, out);
    out << ", ";
  }

  // member: ctrl_fb_steering
  {
    out << "ctrl_fb_steering: ";
    rosidl_generator_traits::value_to_yaml(msg.ctrl_fb_steering, out);
    out << ", ";
  }

  // member: ctrl_fb_mode
  {
    out << "ctrl_fb_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.ctrl_fb_mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CtrlFb & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ctrl_fb_gear
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ctrl_fb_gear: ";
    rosidl_generator_traits::value_to_yaml(msg.ctrl_fb_gear, out);
    out << "\n";
  }

  // member: ctrl_fb_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ctrl_fb_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.ctrl_fb_velocity, out);
    out << "\n";
  }

  // member: ctrl_fb_steering
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ctrl_fb_steering: ";
    rosidl_generator_traits::value_to_yaml(msg.ctrl_fb_steering, out);
    out << "\n";
  }

  // member: ctrl_fb_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ctrl_fb_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.ctrl_fb_mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CtrlFb & msg, bool use_flow_style = false)
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
  const yhs_can_interfaces::msg::CtrlFb & msg,
  std::ostream & out, size_t indentation = 0)
{
  yhs_can_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yhs_can_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const yhs_can_interfaces::msg::CtrlFb & msg)
{
  return yhs_can_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yhs_can_interfaces::msg::CtrlFb>()
{
  return "yhs_can_interfaces::msg::CtrlFb";
}

template<>
inline const char * name<yhs_can_interfaces::msg::CtrlFb>()
{
  return "yhs_can_interfaces/msg/CtrlFb";
}

template<>
struct has_fixed_size<yhs_can_interfaces::msg::CtrlFb>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<yhs_can_interfaces::msg::CtrlFb>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<yhs_can_interfaces::msg::CtrlFb>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__CTRL_FB__TRAITS_HPP_
