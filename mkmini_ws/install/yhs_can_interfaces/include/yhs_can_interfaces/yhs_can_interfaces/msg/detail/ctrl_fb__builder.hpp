// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yhs_can_interfaces:msg/CtrlFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__CTRL_FB__BUILDER_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__CTRL_FB__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yhs_can_interfaces/msg/detail/ctrl_fb__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yhs_can_interfaces
{

namespace msg
{

namespace builder
{

class Init_CtrlFb_ctrl_fb_mode
{
public:
  explicit Init_CtrlFb_ctrl_fb_mode(::yhs_can_interfaces::msg::CtrlFb & msg)
  : msg_(msg)
  {}
  ::yhs_can_interfaces::msg::CtrlFb ctrl_fb_mode(::yhs_can_interfaces::msg::CtrlFb::_ctrl_fb_mode_type arg)
  {
    msg_.ctrl_fb_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yhs_can_interfaces::msg::CtrlFb msg_;
};

class Init_CtrlFb_ctrl_fb_steering
{
public:
  explicit Init_CtrlFb_ctrl_fb_steering(::yhs_can_interfaces::msg::CtrlFb & msg)
  : msg_(msg)
  {}
  Init_CtrlFb_ctrl_fb_mode ctrl_fb_steering(::yhs_can_interfaces::msg::CtrlFb::_ctrl_fb_steering_type arg)
  {
    msg_.ctrl_fb_steering = std::move(arg);
    return Init_CtrlFb_ctrl_fb_mode(msg_);
  }

private:
  ::yhs_can_interfaces::msg::CtrlFb msg_;
};

class Init_CtrlFb_ctrl_fb_velocity
{
public:
  explicit Init_CtrlFb_ctrl_fb_velocity(::yhs_can_interfaces::msg::CtrlFb & msg)
  : msg_(msg)
  {}
  Init_CtrlFb_ctrl_fb_steering ctrl_fb_velocity(::yhs_can_interfaces::msg::CtrlFb::_ctrl_fb_velocity_type arg)
  {
    msg_.ctrl_fb_velocity = std::move(arg);
    return Init_CtrlFb_ctrl_fb_steering(msg_);
  }

private:
  ::yhs_can_interfaces::msg::CtrlFb msg_;
};

class Init_CtrlFb_ctrl_fb_gear
{
public:
  Init_CtrlFb_ctrl_fb_gear()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CtrlFb_ctrl_fb_velocity ctrl_fb_gear(::yhs_can_interfaces::msg::CtrlFb::_ctrl_fb_gear_type arg)
  {
    msg_.ctrl_fb_gear = std::move(arg);
    return Init_CtrlFb_ctrl_fb_velocity(msg_);
  }

private:
  ::yhs_can_interfaces::msg::CtrlFb msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yhs_can_interfaces::msg::CtrlFb>()
{
  return yhs_can_interfaces::msg::builder::Init_CtrlFb_ctrl_fb_gear();
}

}  // namespace yhs_can_interfaces

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__CTRL_FB__BUILDER_HPP_
