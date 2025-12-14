// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yhs_can_interfaces:msg/ChassisInfoFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__CHASSIS_INFO_FB__BUILDER_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__CHASSIS_INFO_FB__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yhs_can_interfaces/msg/detail/chassis_info_fb__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yhs_can_interfaces
{

namespace msg
{

namespace builder
{

class Init_ChassisInfoFb_veh_diag_fb
{
public:
  explicit Init_ChassisInfoFb_veh_diag_fb(::yhs_can_interfaces::msg::ChassisInfoFb & msg)
  : msg_(msg)
  {}
  ::yhs_can_interfaces::msg::ChassisInfoFb veh_diag_fb(::yhs_can_interfaces::msg::ChassisInfoFb::_veh_diag_fb_type arg)
  {
    msg_.veh_diag_fb = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yhs_can_interfaces::msg::ChassisInfoFb msg_;
};

class Init_ChassisInfoFb_drive_mcu_ecode_fb
{
public:
  explicit Init_ChassisInfoFb_drive_mcu_ecode_fb(::yhs_can_interfaces::msg::ChassisInfoFb & msg)
  : msg_(msg)
  {}
  Init_ChassisInfoFb_veh_diag_fb drive_mcu_ecode_fb(::yhs_can_interfaces::msg::ChassisInfoFb::_drive_mcu_ecode_fb_type arg)
  {
    msg_.drive_mcu_ecode_fb = std::move(arg);
    return Init_ChassisInfoFb_veh_diag_fb(msg_);
  }

private:
  ::yhs_can_interfaces::msg::ChassisInfoFb msg_;
};

class Init_ChassisInfoFb_bms_info_fb
{
public:
  explicit Init_ChassisInfoFb_bms_info_fb(::yhs_can_interfaces::msg::ChassisInfoFb & msg)
  : msg_(msg)
  {}
  Init_ChassisInfoFb_drive_mcu_ecode_fb bms_info_fb(::yhs_can_interfaces::msg::ChassisInfoFb::_bms_info_fb_type arg)
  {
    msg_.bms_info_fb = std::move(arg);
    return Init_ChassisInfoFb_drive_mcu_ecode_fb(msg_);
  }

private:
  ::yhs_can_interfaces::msg::ChassisInfoFb msg_;
};

class Init_ChassisInfoFb_bms_flag_info_fb
{
public:
  explicit Init_ChassisInfoFb_bms_flag_info_fb(::yhs_can_interfaces::msg::ChassisInfoFb & msg)
  : msg_(msg)
  {}
  Init_ChassisInfoFb_bms_info_fb bms_flag_info_fb(::yhs_can_interfaces::msg::ChassisInfoFb::_bms_flag_info_fb_type arg)
  {
    msg_.bms_flag_info_fb = std::move(arg);
    return Init_ChassisInfoFb_bms_info_fb(msg_);
  }

private:
  ::yhs_can_interfaces::msg::ChassisInfoFb msg_;
};

class Init_ChassisInfoFb_rr_wheel_fb
{
public:
  explicit Init_ChassisInfoFb_rr_wheel_fb(::yhs_can_interfaces::msg::ChassisInfoFb & msg)
  : msg_(msg)
  {}
  Init_ChassisInfoFb_bms_flag_info_fb rr_wheel_fb(::yhs_can_interfaces::msg::ChassisInfoFb::_rr_wheel_fb_type arg)
  {
    msg_.rr_wheel_fb = std::move(arg);
    return Init_ChassisInfoFb_bms_flag_info_fb(msg_);
  }

private:
  ::yhs_can_interfaces::msg::ChassisInfoFb msg_;
};

class Init_ChassisInfoFb_lr_wheel_fb
{
public:
  explicit Init_ChassisInfoFb_lr_wheel_fb(::yhs_can_interfaces::msg::ChassisInfoFb & msg)
  : msg_(msg)
  {}
  Init_ChassisInfoFb_rr_wheel_fb lr_wheel_fb(::yhs_can_interfaces::msg::ChassisInfoFb::_lr_wheel_fb_type arg)
  {
    msg_.lr_wheel_fb = std::move(arg);
    return Init_ChassisInfoFb_rr_wheel_fb(msg_);
  }

private:
  ::yhs_can_interfaces::msg::ChassisInfoFb msg_;
};

class Init_ChassisInfoFb_io_fb
{
public:
  explicit Init_ChassisInfoFb_io_fb(::yhs_can_interfaces::msg::ChassisInfoFb & msg)
  : msg_(msg)
  {}
  Init_ChassisInfoFb_lr_wheel_fb io_fb(::yhs_can_interfaces::msg::ChassisInfoFb::_io_fb_type arg)
  {
    msg_.io_fb = std::move(arg);
    return Init_ChassisInfoFb_lr_wheel_fb(msg_);
  }

private:
  ::yhs_can_interfaces::msg::ChassisInfoFb msg_;
};

class Init_ChassisInfoFb_ctrl_fb
{
public:
  explicit Init_ChassisInfoFb_ctrl_fb(::yhs_can_interfaces::msg::ChassisInfoFb & msg)
  : msg_(msg)
  {}
  Init_ChassisInfoFb_io_fb ctrl_fb(::yhs_can_interfaces::msg::ChassisInfoFb::_ctrl_fb_type arg)
  {
    msg_.ctrl_fb = std::move(arg);
    return Init_ChassisInfoFb_io_fb(msg_);
  }

private:
  ::yhs_can_interfaces::msg::ChassisInfoFb msg_;
};

class Init_ChassisInfoFb_header
{
public:
  Init_ChassisInfoFb_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChassisInfoFb_ctrl_fb header(::yhs_can_interfaces::msg::ChassisInfoFb::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ChassisInfoFb_ctrl_fb(msg_);
  }

private:
  ::yhs_can_interfaces::msg::ChassisInfoFb msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yhs_can_interfaces::msg::ChassisInfoFb>()
{
  return yhs_can_interfaces::msg::builder::Init_ChassisInfoFb_header();
}

}  // namespace yhs_can_interfaces

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__CHASSIS_INFO_FB__BUILDER_HPP_
