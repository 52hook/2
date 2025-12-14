// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yhs_can_interfaces:msg/LrWheelFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__LR_WHEEL_FB__TRAITS_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__LR_WHEEL_FB__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yhs_can_interfaces/msg/detail/lr_wheel_fb__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace yhs_can_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const LrWheelFb & msg,
  std::ostream & out)
{
  out << "{";
  // member: lr_wheel_fb_velocity
  {
    out << "lr_wheel_fb_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.lr_wheel_fb_velocity, out);
    out << ", ";
  }

  // member: lr_wheel_fb_pulse
  {
    out << "lr_wheel_fb_pulse: ";
    rosidl_generator_traits::value_to_yaml(msg.lr_wheel_fb_pulse, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LrWheelFb & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: lr_wheel_fb_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lr_wheel_fb_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.lr_wheel_fb_velocity, out);
    out << "\n";
  }

  // member: lr_wheel_fb_pulse
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lr_wheel_fb_pulse: ";
    rosidl_generator_traits::value_to_yaml(msg.lr_wheel_fb_pulse, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LrWheelFb & msg, bool use_flow_style = false)
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
  const yhs_can_interfaces::msg::LrWheelFb & msg,
  std::ostream & out, size_t indentation = 0)
{
  yhs_can_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yhs_can_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const yhs_can_interfaces::msg::LrWheelFb & msg)
{
  return yhs_can_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yhs_can_interfaces::msg::LrWheelFb>()
{
  return "yhs_can_interfaces::msg::LrWheelFb";
}

template<>
inline const char * name<yhs_can_interfaces::msg::LrWheelFb>()
{
  return "yhs_can_interfaces/msg/LrWheelFb";
}

template<>
struct has_fixed_size<yhs_can_interfaces::msg::LrWheelFb>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<yhs_can_interfaces::msg::LrWheelFb>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<yhs_can_interfaces::msg::LrWheelFb>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__LR_WHEEL_FB__TRAITS_HPP_
