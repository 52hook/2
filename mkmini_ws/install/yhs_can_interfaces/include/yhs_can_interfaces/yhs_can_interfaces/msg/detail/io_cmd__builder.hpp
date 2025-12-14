// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yhs_can_interfaces:msg/IoCmd.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__IO_CMD__BUILDER_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__IO_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yhs_can_interfaces/msg/detail/io_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yhs_can_interfaces
{

namespace msg
{

namespace builder
{

class Init_IoCmd_io_cmd_dis_charge
{
public:
  explicit Init_IoCmd_io_cmd_dis_charge(::yhs_can_interfaces::msg::IoCmd & msg)
  : msg_(msg)
  {}
  ::yhs_can_interfaces::msg::IoCmd io_cmd_dis_charge(::yhs_can_interfaces::msg::IoCmd::_io_cmd_dis_charge_type arg)
  {
    msg_.io_cmd_dis_charge = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoCmd msg_;
};

class Init_IoCmd_io_cmd_speaker
{
public:
  explicit Init_IoCmd_io_cmd_speaker(::yhs_can_interfaces::msg::IoCmd & msg)
  : msg_(msg)
  {}
  Init_IoCmd_io_cmd_dis_charge io_cmd_speaker(::yhs_can_interfaces::msg::IoCmd::_io_cmd_speaker_type arg)
  {
    msg_.io_cmd_speaker = std::move(arg);
    return Init_IoCmd_io_cmd_dis_charge(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoCmd msg_;
};

class Init_IoCmd_io_cmd_fog_lamp
{
public:
  explicit Init_IoCmd_io_cmd_fog_lamp(::yhs_can_interfaces::msg::IoCmd & msg)
  : msg_(msg)
  {}
  Init_IoCmd_io_cmd_speaker io_cmd_fog_lamp(::yhs_can_interfaces::msg::IoCmd::_io_cmd_fog_lamp_type arg)
  {
    msg_.io_cmd_fog_lamp = std::move(arg);
    return Init_IoCmd_io_cmd_speaker(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoCmd msg_;
};

class Init_IoCmd_io_cmd_clearance_lamp
{
public:
  explicit Init_IoCmd_io_cmd_clearance_lamp(::yhs_can_interfaces::msg::IoCmd & msg)
  : msg_(msg)
  {}
  Init_IoCmd_io_cmd_fog_lamp io_cmd_clearance_lamp(::yhs_can_interfaces::msg::IoCmd::_io_cmd_clearance_lamp_type arg)
  {
    msg_.io_cmd_clearance_lamp = std::move(arg);
    return Init_IoCmd_io_cmd_fog_lamp(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoCmd msg_;
};

class Init_IoCmd_io_cmd_braking_lamp
{
public:
  explicit Init_IoCmd_io_cmd_braking_lamp(::yhs_can_interfaces::msg::IoCmd & msg)
  : msg_(msg)
  {}
  Init_IoCmd_io_cmd_clearance_lamp io_cmd_braking_lamp(::yhs_can_interfaces::msg::IoCmd::_io_cmd_braking_lamp_type arg)
  {
    msg_.io_cmd_braking_lamp = std::move(arg);
    return Init_IoCmd_io_cmd_clearance_lamp(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoCmd msg_;
};

class Init_IoCmd_io_cmd_turn_lamp
{
public:
  explicit Init_IoCmd_io_cmd_turn_lamp(::yhs_can_interfaces::msg::IoCmd & msg)
  : msg_(msg)
  {}
  Init_IoCmd_io_cmd_braking_lamp io_cmd_turn_lamp(::yhs_can_interfaces::msg::IoCmd::_io_cmd_turn_lamp_type arg)
  {
    msg_.io_cmd_turn_lamp = std::move(arg);
    return Init_IoCmd_io_cmd_braking_lamp(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoCmd msg_;
};

class Init_IoCmd_io_cmd_upper_beam_headlamp
{
public:
  explicit Init_IoCmd_io_cmd_upper_beam_headlamp(::yhs_can_interfaces::msg::IoCmd & msg)
  : msg_(msg)
  {}
  Init_IoCmd_io_cmd_turn_lamp io_cmd_upper_beam_headlamp(::yhs_can_interfaces::msg::IoCmd::_io_cmd_upper_beam_headlamp_type arg)
  {
    msg_.io_cmd_upper_beam_headlamp = std::move(arg);
    return Init_IoCmd_io_cmd_turn_lamp(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoCmd msg_;
};

class Init_IoCmd_io_cmd_lower_beam_headlamp
{
public:
  explicit Init_IoCmd_io_cmd_lower_beam_headlamp(::yhs_can_interfaces::msg::IoCmd & msg)
  : msg_(msg)
  {}
  Init_IoCmd_io_cmd_upper_beam_headlamp io_cmd_lower_beam_headlamp(::yhs_can_interfaces::msg::IoCmd::_io_cmd_lower_beam_headlamp_type arg)
  {
    msg_.io_cmd_lower_beam_headlamp = std::move(arg);
    return Init_IoCmd_io_cmd_upper_beam_headlamp(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoCmd msg_;
};

class Init_IoCmd_io_cmd_enable
{
public:
  Init_IoCmd_io_cmd_enable()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IoCmd_io_cmd_lower_beam_headlamp io_cmd_enable(::yhs_can_interfaces::msg::IoCmd::_io_cmd_enable_type arg)
  {
    msg_.io_cmd_enable = std::move(arg);
    return Init_IoCmd_io_cmd_lower_beam_headlamp(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yhs_can_interfaces::msg::IoCmd>()
{
  return yhs_can_interfaces::msg::builder::Init_IoCmd_io_cmd_enable();
}

}  // namespace yhs_can_interfaces

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__IO_CMD__BUILDER_HPP_
