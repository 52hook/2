// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yhs_can_interfaces:msg/IoFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__IO_FB__BUILDER_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__IO_FB__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yhs_can_interfaces/msg/detail/io_fb__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yhs_can_interfaces
{

namespace msg
{

namespace builder
{

class Init_IoFb_io_fb_scram_st
{
public:
  explicit Init_IoFb_io_fb_scram_st(::yhs_can_interfaces::msg::IoFb & msg)
  : msg_(msg)
  {}
  ::yhs_can_interfaces::msg::IoFb io_fb_scram_st(::yhs_can_interfaces::msg::IoFb::_io_fb_scram_st_type arg)
  {
    msg_.io_fb_scram_st = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoFb msg_;
};

class Init_IoFb_io_fb_charge_en
{
public:
  explicit Init_IoFb_io_fb_charge_en(::yhs_can_interfaces::msg::IoFb & msg)
  : msg_(msg)
  {}
  Init_IoFb_io_fb_scram_st io_fb_charge_en(::yhs_can_interfaces::msg::IoFb::_io_fb_charge_en_type arg)
  {
    msg_.io_fb_charge_en = std::move(arg);
    return Init_IoFb_io_fb_scram_st(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoFb msg_;
};

class Init_IoFb_io_fb_dis_charge
{
public:
  explicit Init_IoFb_io_fb_dis_charge(::yhs_can_interfaces::msg::IoFb & msg)
  : msg_(msg)
  {}
  Init_IoFb_io_fb_charge_en io_fb_dis_charge(::yhs_can_interfaces::msg::IoFb::_io_fb_dis_charge_type arg)
  {
    msg_.io_fb_dis_charge = std::move(arg);
    return Init_IoFb_io_fb_charge_en(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoFb msg_;
};

class Init_IoFb_io_fb_rm_impact_sensor
{
public:
  explicit Init_IoFb_io_fb_rm_impact_sensor(::yhs_can_interfaces::msg::IoFb & msg)
  : msg_(msg)
  {}
  Init_IoFb_io_fb_dis_charge io_fb_rm_impact_sensor(::yhs_can_interfaces::msg::IoFb::_io_fb_rm_impact_sensor_type arg)
  {
    msg_.io_fb_rm_impact_sensor = std::move(arg);
    return Init_IoFb_io_fb_dis_charge(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoFb msg_;
};

class Init_IoFb_io_fb_fm_impact_sensor
{
public:
  explicit Init_IoFb_io_fb_fm_impact_sensor(::yhs_can_interfaces::msg::IoFb & msg)
  : msg_(msg)
  {}
  Init_IoFb_io_fb_rm_impact_sensor io_fb_fm_impact_sensor(::yhs_can_interfaces::msg::IoFb::_io_fb_fm_impact_sensor_type arg)
  {
    msg_.io_fb_fm_impact_sensor = std::move(arg);
    return Init_IoFb_io_fb_rm_impact_sensor(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoFb msg_;
};

class Init_IoFb_io_fb_braking_lamp
{
public:
  explicit Init_IoFb_io_fb_braking_lamp(::yhs_can_interfaces::msg::IoFb & msg)
  : msg_(msg)
  {}
  Init_IoFb_io_fb_fm_impact_sensor io_fb_braking_lamp(::yhs_can_interfaces::msg::IoFb::_io_fb_braking_lamp_type arg)
  {
    msg_.io_fb_braking_lamp = std::move(arg);
    return Init_IoFb_io_fb_fm_impact_sensor(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoFb msg_;
};

class Init_IoFb_io_fb_turn_lamp
{
public:
  explicit Init_IoFb_io_fb_turn_lamp(::yhs_can_interfaces::msg::IoFb & msg)
  : msg_(msg)
  {}
  Init_IoFb_io_fb_braking_lamp io_fb_turn_lamp(::yhs_can_interfaces::msg::IoFb::_io_fb_turn_lamp_type arg)
  {
    msg_.io_fb_turn_lamp = std::move(arg);
    return Init_IoFb_io_fb_braking_lamp(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoFb msg_;
};

class Init_IoFb_io_fb_enable
{
public:
  Init_IoFb_io_fb_enable()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IoFb_io_fb_turn_lamp io_fb_enable(::yhs_can_interfaces::msg::IoFb::_io_fb_enable_type arg)
  {
    msg_.io_fb_enable = std::move(arg);
    return Init_IoFb_io_fb_turn_lamp(msg_);
  }

private:
  ::yhs_can_interfaces::msg::IoFb msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yhs_can_interfaces::msg::IoFb>()
{
  return yhs_can_interfaces::msg::builder::Init_IoFb_io_fb_enable();
}

}  // namespace yhs_can_interfaces

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__IO_FB__BUILDER_HPP_
