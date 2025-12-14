// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yhs_can_interfaces:msg/BmsInfoFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__BMS_INFO_FB__BUILDER_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__BMS_INFO_FB__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yhs_can_interfaces/msg/detail/bms_info_fb__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yhs_can_interfaces
{

namespace msg
{

namespace builder
{

class Init_BmsInfoFb_bms_info_remaining_capacity
{
public:
  explicit Init_BmsInfoFb_bms_info_remaining_capacity(::yhs_can_interfaces::msg::BmsInfoFb & msg)
  : msg_(msg)
  {}
  ::yhs_can_interfaces::msg::BmsInfoFb bms_info_remaining_capacity(::yhs_can_interfaces::msg::BmsInfoFb::_bms_info_remaining_capacity_type arg)
  {
    msg_.bms_info_remaining_capacity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yhs_can_interfaces::msg::BmsInfoFb msg_;
};

class Init_BmsInfoFb_bms_info_current
{
public:
  explicit Init_BmsInfoFb_bms_info_current(::yhs_can_interfaces::msg::BmsInfoFb & msg)
  : msg_(msg)
  {}
  Init_BmsInfoFb_bms_info_remaining_capacity bms_info_current(::yhs_can_interfaces::msg::BmsInfoFb::_bms_info_current_type arg)
  {
    msg_.bms_info_current = std::move(arg);
    return Init_BmsInfoFb_bms_info_remaining_capacity(msg_);
  }

private:
  ::yhs_can_interfaces::msg::BmsInfoFb msg_;
};

class Init_BmsInfoFb_bms_info_voltage
{
public:
  Init_BmsInfoFb_bms_info_voltage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BmsInfoFb_bms_info_current bms_info_voltage(::yhs_can_interfaces::msg::BmsInfoFb::_bms_info_voltage_type arg)
  {
    msg_.bms_info_voltage = std::move(arg);
    return Init_BmsInfoFb_bms_info_current(msg_);
  }

private:
  ::yhs_can_interfaces::msg::BmsInfoFb msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yhs_can_interfaces::msg::BmsInfoFb>()
{
  return yhs_can_interfaces::msg::builder::Init_BmsInfoFb_bms_info_voltage();
}

}  // namespace yhs_can_interfaces

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__BMS_INFO_FB__BUILDER_HPP_
