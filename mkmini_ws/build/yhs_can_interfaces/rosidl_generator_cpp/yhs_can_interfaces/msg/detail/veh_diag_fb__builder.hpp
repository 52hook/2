// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yhs_can_interfaces:msg/VehDiagFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__VEH_DIAG_FB__BUILDER_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__VEH_DIAG_FB__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yhs_can_interfaces/msg/detail/veh_diag_fb__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yhs_can_interfaces
{

namespace msg
{

namespace builder
{

class Init_VehDiagFb_veh_fb_aux_remote_dis_on_line
{
public:
  explicit Init_VehDiagFb_veh_fb_aux_remote_dis_on_line(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  ::yhs_can_interfaces::msg::VehDiagFb veh_fb_aux_remote_dis_on_line(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_aux_remote_dis_on_line_type arg)
  {
    msg_.veh_fb_aux_remote_dis_on_line = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_aux_remote_close
{
public:
  explicit Init_VehDiagFb_veh_fb_aux_remote_close(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_aux_remote_dis_on_line veh_fb_aux_remote_close(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_aux_remote_close_type arg)
  {
    msg_.veh_fb_aux_remote_close = std::move(arg);
    return Init_VehDiagFb_veh_fb_aux_remote_dis_on_line(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_aux_scram
{
public:
  explicit Init_VehDiagFb_veh_fb_aux_scram(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_aux_remote_close veh_fb_aux_scram(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_aux_scram_type arg)
  {
    msg_.veh_fb_aux_scram = std::move(arg);
    return Init_VehDiagFb_veh_fb_aux_remote_close(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_aux_bms_dis_on_line
{
public:
  explicit Init_VehDiagFb_veh_fb_aux_bms_dis_on_line(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_aux_scram veh_fb_aux_bms_dis_on_line(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_aux_bms_dis_on_line_type arg)
  {
    msg_.veh_fb_aux_bms_dis_on_line = std::move(arg);
    return Init_VehDiagFb_veh_fb_aux_scram(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_rd_rv_mcu_fault
{
public:
  explicit Init_VehDiagFb_veh_fb_rd_rv_mcu_fault(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_aux_bms_dis_on_line veh_fb_rd_rv_mcu_fault(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_rd_rv_mcu_fault_type arg)
  {
    msg_.veh_fb_rd_rv_mcu_fault = std::move(arg);
    return Init_VehDiagFb_veh_fb_aux_bms_dis_on_line(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_ld_rv_mcu_fault
{
public:
  explicit Init_VehDiagFb_veh_fb_ld_rv_mcu_fault(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_rd_rv_mcu_fault veh_fb_ld_rv_mcu_fault(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_ld_rv_mcu_fault_type arg)
  {
    msg_.veh_fb_ld_rv_mcu_fault = std::move(arg);
    return Init_VehDiagFb_veh_fb_rd_rv_mcu_fault(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_ehb_oil_fault
{
public:
  explicit Init_VehDiagFb_veh_fb_ehb_oil_fault(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_ld_rv_mcu_fault veh_fb_ehb_oil_fault(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_ehb_oil_fault_type arg)
  {
    msg_.veh_fb_ehb_oil_fault = std::move(arg);
    return Init_VehDiagFb_veh_fb_ld_rv_mcu_fault(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_ehb_oil_press_sensor_fault
{
public:
  explicit Init_VehDiagFb_veh_fb_ehb_oil_press_sensor_fault(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_ehb_oil_fault veh_fb_ehb_oil_press_sensor_fault(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_ehb_oil_press_sensor_fault_type arg)
  {
    msg_.veh_fb_ehb_oil_press_sensor_fault = std::move(arg);
    return Init_VehDiagFb_veh_fb_ehb_oil_fault(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_ehb_motor_fault
{
public:
  explicit Init_VehDiagFb_veh_fb_ehb_motor_fault(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_ehb_oil_press_sensor_fault veh_fb_ehb_motor_fault(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_ehb_motor_fault_type arg)
  {
    msg_.veh_fb_ehb_motor_fault = std::move(arg);
    return Init_VehDiagFb_veh_fb_ehb_oil_press_sensor_fault(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_ehb_sensor_abnomal
{
public:
  explicit Init_VehDiagFb_veh_fb_ehb_sensor_abnomal(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_ehb_motor_fault veh_fb_ehb_sensor_abnomal(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_ehb_sensor_abnomal_type arg)
  {
    msg_.veh_fb_ehb_sensor_abnomal = std::move(arg);
    return Init_VehDiagFb_veh_fb_ehb_motor_fault(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_ehb_power_fault
{
public:
  explicit Init_VehDiagFb_veh_fb_ehb_power_fault(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_ehb_sensor_abnomal veh_fb_ehb_power_fault(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_ehb_power_fault_type arg)
  {
    msg_.veh_fb_ehb_power_fault = std::move(arg);
    return Init_VehDiagFb_veh_fb_ehb_sensor_abnomal(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_ehb_ot
{
public:
  explicit Init_VehDiagFb_veh_fb_ehb_ot(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_ehb_power_fault veh_fb_ehb_ot(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_ehb_ot_type arg)
  {
    msg_.veh_fb_ehb_ot = std::move(arg);
    return Init_VehDiagFb_veh_fb_ehb_power_fault(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_ehb_anguler_fault
{
public:
  explicit Init_VehDiagFb_veh_fb_ehb_anguler_fault(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_ehb_ot veh_fb_ehb_anguler_fault(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_ehb_anguler_fault_type arg)
  {
    msg_.veh_fb_ehb_anguler_fault = std::move(arg);
    return Init_VehDiagFb_veh_fb_ehb_ot(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_ehb_dis_en
{
public:
  explicit Init_VehDiagFb_veh_fb_ehb_dis_en(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_ehb_anguler_fault veh_fb_ehb_dis_en(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_ehb_dis_en_type arg)
  {
    msg_.veh_fb_ehb_dis_en = std::move(arg);
    return Init_VehDiagFb_veh_fb_ehb_anguler_fault(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_ehb_work_model_fault
{
public:
  explicit Init_VehDiagFb_veh_fb_ehb_work_model_fault(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_ehb_dis_en veh_fb_ehb_work_model_fault(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_ehb_work_model_fault_type arg)
  {
    msg_.veh_fb_ehb_work_model_fault = std::move(arg);
    return Init_VehDiagFb_veh_fb_ehb_dis_en(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_ehb_dis_on_line
{
public:
  explicit Init_VehDiagFb_veh_fb_ehb_dis_on_line(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_ehb_work_model_fault veh_fb_ehb_dis_on_line(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_ehb_dis_on_line_type arg)
  {
    msg_.veh_fb_ehb_dis_on_line = std::move(arg);
    return Init_VehDiagFb_veh_fb_ehb_work_model_fault(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_ehb_ecu_fault
{
public:
  explicit Init_VehDiagFb_veh_fb_ehb_ecu_fault(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_ehb_dis_on_line veh_fb_ehb_ecu_fault(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_ehb_ecu_fault_type arg)
  {
    msg_.veh_fb_ehb_ecu_fault = std::move(arg);
    return Init_VehDiagFb_veh_fb_ehb_dis_on_line(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_st_reserve
{
public:
  explicit Init_VehDiagFb_veh_fb_st_reserve(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_ehb_ecu_fault veh_fb_st_reserve(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_st_reserve_type arg)
  {
    msg_.veh_fb_st_reserve = std::move(arg);
    return Init_VehDiagFb_veh_fb_ehb_ecu_fault(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_eps_over_current
{
public:
  explicit Init_VehDiagFb_veh_fb_eps_over_current(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_st_reserve veh_fb_eps_over_current(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_eps_over_current_type arg)
  {
    msg_.veh_fb_eps_over_current = std::move(arg);
    return Init_VehDiagFb_veh_fb_st_reserve(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_eps_dis_work
{
public:
  explicit Init_VehDiagFb_veh_fb_eps_dis_work(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_eps_over_current veh_fb_eps_dis_work(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_eps_dis_work_type arg)
  {
    msg_.veh_fb_eps_dis_work = std::move(arg);
    return Init_VehDiagFb_veh_fb_eps_over_current(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_eps_warning
{
public:
  explicit Init_VehDiagFb_veh_fb_eps_warning(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_eps_dis_work veh_fb_eps_warning(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_eps_warning_type arg)
  {
    msg_.veh_fb_eps_warning = std::move(arg);
    return Init_VehDiagFb_veh_fb_eps_dis_work(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_eps_mosf_et_ot
{
public:
  explicit Init_VehDiagFb_veh_fb_eps_mosf_et_ot(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_eps_warning veh_fb_eps_mosf_et_ot(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_eps_mosf_et_ot_type arg)
  {
    msg_.veh_fb_eps_mosf_et_ot = std::move(arg);
    return Init_VehDiagFb_veh_fb_eps_warning(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_eps_fault
{
public:
  explicit Init_VehDiagFb_veh_fb_eps_fault(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_eps_mosf_et_ot veh_fb_eps_fault(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_eps_fault_type arg)
  {
    msg_.veh_fb_eps_fault = std::move(arg);
    return Init_VehDiagFb_veh_fb_eps_mosf_et_ot(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_eps_dis_on_line
{
public:
  explicit Init_VehDiagFb_veh_fb_eps_dis_on_line(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_eps_fault veh_fb_eps_dis_on_line(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_eps_dis_on_line_type arg)
  {
    msg_.veh_fb_eps_dis_on_line = std::move(arg);
    return Init_VehDiagFb_veh_fb_eps_fault(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_auto_io_can_cmd
{
public:
  explicit Init_VehDiagFb_veh_fb_auto_io_can_cmd(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_eps_dis_on_line veh_fb_auto_io_can_cmd(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_auto_io_can_cmd_type arg)
  {
    msg_.veh_fb_auto_io_can_cmd = std::move(arg);
    return Init_VehDiagFb_veh_fb_eps_dis_on_line(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_auto_can_ctrl_cmd
{
public:
  explicit Init_VehDiagFb_veh_fb_auto_can_ctrl_cmd(::yhs_can_interfaces::msg::VehDiagFb & msg)
  : msg_(msg)
  {}
  Init_VehDiagFb_veh_fb_auto_io_can_cmd veh_fb_auto_can_ctrl_cmd(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_auto_can_ctrl_cmd_type arg)
  {
    msg_.veh_fb_auto_can_ctrl_cmd = std::move(arg);
    return Init_VehDiagFb_veh_fb_auto_io_can_cmd(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

class Init_VehDiagFb_veh_fb_fault_level
{
public:
  Init_VehDiagFb_veh_fb_fault_level()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VehDiagFb_veh_fb_auto_can_ctrl_cmd veh_fb_fault_level(::yhs_can_interfaces::msg::VehDiagFb::_veh_fb_fault_level_type arg)
  {
    msg_.veh_fb_fault_level = std::move(arg);
    return Init_VehDiagFb_veh_fb_auto_can_ctrl_cmd(msg_);
  }

private:
  ::yhs_can_interfaces::msg::VehDiagFb msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yhs_can_interfaces::msg::VehDiagFb>()
{
  return yhs_can_interfaces::msg::builder::Init_VehDiagFb_veh_fb_fault_level();
}

}  // namespace yhs_can_interfaces

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__VEH_DIAG_FB__BUILDER_HPP_
