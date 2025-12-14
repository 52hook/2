// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yhs_can_interfaces:msg/LrWheelFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__LR_WHEEL_FB__BUILDER_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__LR_WHEEL_FB__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yhs_can_interfaces/msg/detail/lr_wheel_fb__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yhs_can_interfaces
{

namespace msg
{

namespace builder
{

class Init_LrWheelFb_lr_wheel_fb_pulse
{
public:
  explicit Init_LrWheelFb_lr_wheel_fb_pulse(::yhs_can_interfaces::msg::LrWheelFb & msg)
  : msg_(msg)
  {}
  ::yhs_can_interfaces::msg::LrWheelFb lr_wheel_fb_pulse(::yhs_can_interfaces::msg::LrWheelFb::_lr_wheel_fb_pulse_type arg)
  {
    msg_.lr_wheel_fb_pulse = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yhs_can_interfaces::msg::LrWheelFb msg_;
};

class Init_LrWheelFb_lr_wheel_fb_velocity
{
public:
  Init_LrWheelFb_lr_wheel_fb_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LrWheelFb_lr_wheel_fb_pulse lr_wheel_fb_velocity(::yhs_can_interfaces::msg::LrWheelFb::_lr_wheel_fb_velocity_type arg)
  {
    msg_.lr_wheel_fb_velocity = std::move(arg);
    return Init_LrWheelFb_lr_wheel_fb_pulse(msg_);
  }

private:
  ::yhs_can_interfaces::msg::LrWheelFb msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yhs_can_interfaces::msg::LrWheelFb>()
{
  return yhs_can_interfaces::msg::builder::Init_LrWheelFb_lr_wheel_fb_velocity();
}

}  // namespace yhs_can_interfaces

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__LR_WHEEL_FB__BUILDER_HPP_
