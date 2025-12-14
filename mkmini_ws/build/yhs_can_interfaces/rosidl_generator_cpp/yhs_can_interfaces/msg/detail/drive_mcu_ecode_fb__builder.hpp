// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yhs_can_interfaces:msg/DriveMcuEcodeFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__DRIVE_MCU_ECODE_FB__BUILDER_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__DRIVE_MCU_ECODE_FB__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yhs_can_interfaces/msg/detail/drive_mcu_ecode_fb__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yhs_can_interfaces
{

namespace msg
{

namespace builder
{

class Init_DriveMcuEcodeFb_drive_fb_mcuecode
{
public:
  Init_DriveMcuEcodeFb_drive_fb_mcuecode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::yhs_can_interfaces::msg::DriveMcuEcodeFb drive_fb_mcuecode(::yhs_can_interfaces::msg::DriveMcuEcodeFb::_drive_fb_mcuecode_type arg)
  {
    msg_.drive_fb_mcuecode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yhs_can_interfaces::msg::DriveMcuEcodeFb msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yhs_can_interfaces::msg::DriveMcuEcodeFb>()
{
  return yhs_can_interfaces::msg::builder::Init_DriveMcuEcodeFb_drive_fb_mcuecode();
}

}  // namespace yhs_can_interfaces

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__DRIVE_MCU_ECODE_FB__BUILDER_HPP_
