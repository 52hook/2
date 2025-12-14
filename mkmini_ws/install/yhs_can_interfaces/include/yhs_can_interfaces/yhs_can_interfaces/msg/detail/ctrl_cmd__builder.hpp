// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yhs_can_interfaces:msg/CtrlCmd.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__CTRL_CMD__BUILDER_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__CTRL_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yhs_can_interfaces/msg/detail/ctrl_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yhs_can_interfaces
{

namespace msg
{

namespace builder
{

class Init_CtrlCmd_ctrl_cmd_steering
{
public:
  explicit Init_CtrlCmd_ctrl_cmd_steering(::yhs_can_interfaces::msg::CtrlCmd & msg)
  : msg_(msg)
  {}
  ::yhs_can_interfaces::msg::CtrlCmd ctrl_cmd_steering(::yhs_can_interfaces::msg::CtrlCmd::_ctrl_cmd_steering_type arg)
  {
    msg_.ctrl_cmd_steering = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yhs_can_interfaces::msg::CtrlCmd msg_;
};

class Init_CtrlCmd_ctrl_cmd_velocity
{
public:
  explicit Init_CtrlCmd_ctrl_cmd_velocity(::yhs_can_interfaces::msg::CtrlCmd & msg)
  : msg_(msg)
  {}
  Init_CtrlCmd_ctrl_cmd_steering ctrl_cmd_velocity(::yhs_can_interfaces::msg::CtrlCmd::_ctrl_cmd_velocity_type arg)
  {
    msg_.ctrl_cmd_velocity = std::move(arg);
    return Init_CtrlCmd_ctrl_cmd_steering(msg_);
  }

private:
  ::yhs_can_interfaces::msg::CtrlCmd msg_;
};

class Init_CtrlCmd_ctrl_cmd_gear
{
public:
  Init_CtrlCmd_ctrl_cmd_gear()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CtrlCmd_ctrl_cmd_velocity ctrl_cmd_gear(::yhs_can_interfaces::msg::CtrlCmd::_ctrl_cmd_gear_type arg)
  {
    msg_.ctrl_cmd_gear = std::move(arg);
    return Init_CtrlCmd_ctrl_cmd_velocity(msg_);
  }

private:
  ::yhs_can_interfaces::msg::CtrlCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yhs_can_interfaces::msg::CtrlCmd>()
{
  return yhs_can_interfaces::msg::builder::Init_CtrlCmd_ctrl_cmd_gear();
}

}  // namespace yhs_can_interfaces

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__CTRL_CMD__BUILDER_HPP_
