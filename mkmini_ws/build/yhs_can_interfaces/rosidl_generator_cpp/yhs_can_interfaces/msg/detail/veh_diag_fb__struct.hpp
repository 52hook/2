// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yhs_can_interfaces:msg/VehDiagFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__VEH_DIAG_FB__STRUCT_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__VEH_DIAG_FB__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__yhs_can_interfaces__msg__VehDiagFb __attribute__((deprecated))
#else
# define DEPRECATED__yhs_can_interfaces__msg__VehDiagFb __declspec(deprecated)
#endif

namespace yhs_can_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VehDiagFb_
{
  using Type = VehDiagFb_<ContainerAllocator>;

  explicit VehDiagFb_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->veh_fb_fault_level = 0;
      this->veh_fb_auto_can_ctrl_cmd = false;
      this->veh_fb_auto_io_can_cmd = false;
      this->veh_fb_eps_dis_on_line = false;
      this->veh_fb_eps_fault = false;
      this->veh_fb_eps_mosf_et_ot = false;
      this->veh_fb_eps_warning = false;
      this->veh_fb_eps_dis_work = false;
      this->veh_fb_eps_over_current = false;
      this->veh_fb_st_reserve = false;
      this->veh_fb_ehb_ecu_fault = false;
      this->veh_fb_ehb_dis_on_line = false;
      this->veh_fb_ehb_work_model_fault = false;
      this->veh_fb_ehb_dis_en = false;
      this->veh_fb_ehb_anguler_fault = false;
      this->veh_fb_ehb_ot = false;
      this->veh_fb_ehb_power_fault = false;
      this->veh_fb_ehb_sensor_abnomal = false;
      this->veh_fb_ehb_motor_fault = false;
      this->veh_fb_ehb_oil_press_sensor_fault = false;
      this->veh_fb_ehb_oil_fault = false;
      this->veh_fb_ld_rv_mcu_fault = 0;
      this->veh_fb_rd_rv_mcu_fault = 0;
      this->veh_fb_aux_bms_dis_on_line = false;
      this->veh_fb_aux_scram = false;
      this->veh_fb_aux_remote_close = false;
      this->veh_fb_aux_remote_dis_on_line = false;
    }
  }

  explicit VehDiagFb_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->veh_fb_fault_level = 0;
      this->veh_fb_auto_can_ctrl_cmd = false;
      this->veh_fb_auto_io_can_cmd = false;
      this->veh_fb_eps_dis_on_line = false;
      this->veh_fb_eps_fault = false;
      this->veh_fb_eps_mosf_et_ot = false;
      this->veh_fb_eps_warning = false;
      this->veh_fb_eps_dis_work = false;
      this->veh_fb_eps_over_current = false;
      this->veh_fb_st_reserve = false;
      this->veh_fb_ehb_ecu_fault = false;
      this->veh_fb_ehb_dis_on_line = false;
      this->veh_fb_ehb_work_model_fault = false;
      this->veh_fb_ehb_dis_en = false;
      this->veh_fb_ehb_anguler_fault = false;
      this->veh_fb_ehb_ot = false;
      this->veh_fb_ehb_power_fault = false;
      this->veh_fb_ehb_sensor_abnomal = false;
      this->veh_fb_ehb_motor_fault = false;
      this->veh_fb_ehb_oil_press_sensor_fault = false;
      this->veh_fb_ehb_oil_fault = false;
      this->veh_fb_ld_rv_mcu_fault = 0;
      this->veh_fb_rd_rv_mcu_fault = 0;
      this->veh_fb_aux_bms_dis_on_line = false;
      this->veh_fb_aux_scram = false;
      this->veh_fb_aux_remote_close = false;
      this->veh_fb_aux_remote_dis_on_line = false;
    }
  }

  // field types and members
  using _veh_fb_fault_level_type =
    uint8_t;
  _veh_fb_fault_level_type veh_fb_fault_level;
  using _veh_fb_auto_can_ctrl_cmd_type =
    bool;
  _veh_fb_auto_can_ctrl_cmd_type veh_fb_auto_can_ctrl_cmd;
  using _veh_fb_auto_io_can_cmd_type =
    bool;
  _veh_fb_auto_io_can_cmd_type veh_fb_auto_io_can_cmd;
  using _veh_fb_eps_dis_on_line_type =
    bool;
  _veh_fb_eps_dis_on_line_type veh_fb_eps_dis_on_line;
  using _veh_fb_eps_fault_type =
    bool;
  _veh_fb_eps_fault_type veh_fb_eps_fault;
  using _veh_fb_eps_mosf_et_ot_type =
    bool;
  _veh_fb_eps_mosf_et_ot_type veh_fb_eps_mosf_et_ot;
  using _veh_fb_eps_warning_type =
    bool;
  _veh_fb_eps_warning_type veh_fb_eps_warning;
  using _veh_fb_eps_dis_work_type =
    bool;
  _veh_fb_eps_dis_work_type veh_fb_eps_dis_work;
  using _veh_fb_eps_over_current_type =
    bool;
  _veh_fb_eps_over_current_type veh_fb_eps_over_current;
  using _veh_fb_st_reserve_type =
    bool;
  _veh_fb_st_reserve_type veh_fb_st_reserve;
  using _veh_fb_ehb_ecu_fault_type =
    bool;
  _veh_fb_ehb_ecu_fault_type veh_fb_ehb_ecu_fault;
  using _veh_fb_ehb_dis_on_line_type =
    bool;
  _veh_fb_ehb_dis_on_line_type veh_fb_ehb_dis_on_line;
  using _veh_fb_ehb_work_model_fault_type =
    bool;
  _veh_fb_ehb_work_model_fault_type veh_fb_ehb_work_model_fault;
  using _veh_fb_ehb_dis_en_type =
    bool;
  _veh_fb_ehb_dis_en_type veh_fb_ehb_dis_en;
  using _veh_fb_ehb_anguler_fault_type =
    bool;
  _veh_fb_ehb_anguler_fault_type veh_fb_ehb_anguler_fault;
  using _veh_fb_ehb_ot_type =
    bool;
  _veh_fb_ehb_ot_type veh_fb_ehb_ot;
  using _veh_fb_ehb_power_fault_type =
    bool;
  _veh_fb_ehb_power_fault_type veh_fb_ehb_power_fault;
  using _veh_fb_ehb_sensor_abnomal_type =
    bool;
  _veh_fb_ehb_sensor_abnomal_type veh_fb_ehb_sensor_abnomal;
  using _veh_fb_ehb_motor_fault_type =
    bool;
  _veh_fb_ehb_motor_fault_type veh_fb_ehb_motor_fault;
  using _veh_fb_ehb_oil_press_sensor_fault_type =
    bool;
  _veh_fb_ehb_oil_press_sensor_fault_type veh_fb_ehb_oil_press_sensor_fault;
  using _veh_fb_ehb_oil_fault_type =
    bool;
  _veh_fb_ehb_oil_fault_type veh_fb_ehb_oil_fault;
  using _veh_fb_ld_rv_mcu_fault_type =
    uint8_t;
  _veh_fb_ld_rv_mcu_fault_type veh_fb_ld_rv_mcu_fault;
  using _veh_fb_rd_rv_mcu_fault_type =
    uint8_t;
  _veh_fb_rd_rv_mcu_fault_type veh_fb_rd_rv_mcu_fault;
  using _veh_fb_aux_bms_dis_on_line_type =
    bool;
  _veh_fb_aux_bms_dis_on_line_type veh_fb_aux_bms_dis_on_line;
  using _veh_fb_aux_scram_type =
    bool;
  _veh_fb_aux_scram_type veh_fb_aux_scram;
  using _veh_fb_aux_remote_close_type =
    bool;
  _veh_fb_aux_remote_close_type veh_fb_aux_remote_close;
  using _veh_fb_aux_remote_dis_on_line_type =
    bool;
  _veh_fb_aux_remote_dis_on_line_type veh_fb_aux_remote_dis_on_line;

  // setters for named parameter idiom
  Type & set__veh_fb_fault_level(
    const uint8_t & _arg)
  {
    this->veh_fb_fault_level = _arg;
    return *this;
  }
  Type & set__veh_fb_auto_can_ctrl_cmd(
    const bool & _arg)
  {
    this->veh_fb_auto_can_ctrl_cmd = _arg;
    return *this;
  }
  Type & set__veh_fb_auto_io_can_cmd(
    const bool & _arg)
  {
    this->veh_fb_auto_io_can_cmd = _arg;
    return *this;
  }
  Type & set__veh_fb_eps_dis_on_line(
    const bool & _arg)
  {
    this->veh_fb_eps_dis_on_line = _arg;
    return *this;
  }
  Type & set__veh_fb_eps_fault(
    const bool & _arg)
  {
    this->veh_fb_eps_fault = _arg;
    return *this;
  }
  Type & set__veh_fb_eps_mosf_et_ot(
    const bool & _arg)
  {
    this->veh_fb_eps_mosf_et_ot = _arg;
    return *this;
  }
  Type & set__veh_fb_eps_warning(
    const bool & _arg)
  {
    this->veh_fb_eps_warning = _arg;
    return *this;
  }
  Type & set__veh_fb_eps_dis_work(
    const bool & _arg)
  {
    this->veh_fb_eps_dis_work = _arg;
    return *this;
  }
  Type & set__veh_fb_eps_over_current(
    const bool & _arg)
  {
    this->veh_fb_eps_over_current = _arg;
    return *this;
  }
  Type & set__veh_fb_st_reserve(
    const bool & _arg)
  {
    this->veh_fb_st_reserve = _arg;
    return *this;
  }
  Type & set__veh_fb_ehb_ecu_fault(
    const bool & _arg)
  {
    this->veh_fb_ehb_ecu_fault = _arg;
    return *this;
  }
  Type & set__veh_fb_ehb_dis_on_line(
    const bool & _arg)
  {
    this->veh_fb_ehb_dis_on_line = _arg;
    return *this;
  }
  Type & set__veh_fb_ehb_work_model_fault(
    const bool & _arg)
  {
    this->veh_fb_ehb_work_model_fault = _arg;
    return *this;
  }
  Type & set__veh_fb_ehb_dis_en(
    const bool & _arg)
  {
    this->veh_fb_ehb_dis_en = _arg;
    return *this;
  }
  Type & set__veh_fb_ehb_anguler_fault(
    const bool & _arg)
  {
    this->veh_fb_ehb_anguler_fault = _arg;
    return *this;
  }
  Type & set__veh_fb_ehb_ot(
    const bool & _arg)
  {
    this->veh_fb_ehb_ot = _arg;
    return *this;
  }
  Type & set__veh_fb_ehb_power_fault(
    const bool & _arg)
  {
    this->veh_fb_ehb_power_fault = _arg;
    return *this;
  }
  Type & set__veh_fb_ehb_sensor_abnomal(
    const bool & _arg)
  {
    this->veh_fb_ehb_sensor_abnomal = _arg;
    return *this;
  }
  Type & set__veh_fb_ehb_motor_fault(
    const bool & _arg)
  {
    this->veh_fb_ehb_motor_fault = _arg;
    return *this;
  }
  Type & set__veh_fb_ehb_oil_press_sensor_fault(
    const bool & _arg)
  {
    this->veh_fb_ehb_oil_press_sensor_fault = _arg;
    return *this;
  }
  Type & set__veh_fb_ehb_oil_fault(
    const bool & _arg)
  {
    this->veh_fb_ehb_oil_fault = _arg;
    return *this;
  }
  Type & set__veh_fb_ld_rv_mcu_fault(
    const uint8_t & _arg)
  {
    this->veh_fb_ld_rv_mcu_fault = _arg;
    return *this;
  }
  Type & set__veh_fb_rd_rv_mcu_fault(
    const uint8_t & _arg)
  {
    this->veh_fb_rd_rv_mcu_fault = _arg;
    return *this;
  }
  Type & set__veh_fb_aux_bms_dis_on_line(
    const bool & _arg)
  {
    this->veh_fb_aux_bms_dis_on_line = _arg;
    return *this;
  }
  Type & set__veh_fb_aux_scram(
    const bool & _arg)
  {
    this->veh_fb_aux_scram = _arg;
    return *this;
  }
  Type & set__veh_fb_aux_remote_close(
    const bool & _arg)
  {
    this->veh_fb_aux_remote_close = _arg;
    return *this;
  }
  Type & set__veh_fb_aux_remote_dis_on_line(
    const bool & _arg)
  {
    this->veh_fb_aux_remote_dis_on_line = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yhs_can_interfaces::msg::VehDiagFb_<ContainerAllocator> *;
  using ConstRawPtr =
    const yhs_can_interfaces::msg::VehDiagFb_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yhs_can_interfaces::msg::VehDiagFb_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yhs_can_interfaces::msg::VehDiagFb_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yhs_can_interfaces::msg::VehDiagFb_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yhs_can_interfaces::msg::VehDiagFb_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yhs_can_interfaces::msg::VehDiagFb_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yhs_can_interfaces::msg::VehDiagFb_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yhs_can_interfaces::msg::VehDiagFb_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yhs_can_interfaces::msg::VehDiagFb_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yhs_can_interfaces__msg__VehDiagFb
    std::shared_ptr<yhs_can_interfaces::msg::VehDiagFb_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yhs_can_interfaces__msg__VehDiagFb
    std::shared_ptr<yhs_can_interfaces::msg::VehDiagFb_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VehDiagFb_ & other) const
  {
    if (this->veh_fb_fault_level != other.veh_fb_fault_level) {
      return false;
    }
    if (this->veh_fb_auto_can_ctrl_cmd != other.veh_fb_auto_can_ctrl_cmd) {
      return false;
    }
    if (this->veh_fb_auto_io_can_cmd != other.veh_fb_auto_io_can_cmd) {
      return false;
    }
    if (this->veh_fb_eps_dis_on_line != other.veh_fb_eps_dis_on_line) {
      return false;
    }
    if (this->veh_fb_eps_fault != other.veh_fb_eps_fault) {
      return false;
    }
    if (this->veh_fb_eps_mosf_et_ot != other.veh_fb_eps_mosf_et_ot) {
      return false;
    }
    if (this->veh_fb_eps_warning != other.veh_fb_eps_warning) {
      return false;
    }
    if (this->veh_fb_eps_dis_work != other.veh_fb_eps_dis_work) {
      return false;
    }
    if (this->veh_fb_eps_over_current != other.veh_fb_eps_over_current) {
      return false;
    }
    if (this->veh_fb_st_reserve != other.veh_fb_st_reserve) {
      return false;
    }
    if (this->veh_fb_ehb_ecu_fault != other.veh_fb_ehb_ecu_fault) {
      return false;
    }
    if (this->veh_fb_ehb_dis_on_line != other.veh_fb_ehb_dis_on_line) {
      return false;
    }
    if (this->veh_fb_ehb_work_model_fault != other.veh_fb_ehb_work_model_fault) {
      return false;
    }
    if (this->veh_fb_ehb_dis_en != other.veh_fb_ehb_dis_en) {
      return false;
    }
    if (this->veh_fb_ehb_anguler_fault != other.veh_fb_ehb_anguler_fault) {
      return false;
    }
    if (this->veh_fb_ehb_ot != other.veh_fb_ehb_ot) {
      return false;
    }
    if (this->veh_fb_ehb_power_fault != other.veh_fb_ehb_power_fault) {
      return false;
    }
    if (this->veh_fb_ehb_sensor_abnomal != other.veh_fb_ehb_sensor_abnomal) {
      return false;
    }
    if (this->veh_fb_ehb_motor_fault != other.veh_fb_ehb_motor_fault) {
      return false;
    }
    if (this->veh_fb_ehb_oil_press_sensor_fault != other.veh_fb_ehb_oil_press_sensor_fault) {
      return false;
    }
    if (this->veh_fb_ehb_oil_fault != other.veh_fb_ehb_oil_fault) {
      return false;
    }
    if (this->veh_fb_ld_rv_mcu_fault != other.veh_fb_ld_rv_mcu_fault) {
      return false;
    }
    if (this->veh_fb_rd_rv_mcu_fault != other.veh_fb_rd_rv_mcu_fault) {
      return false;
    }
    if (this->veh_fb_aux_bms_dis_on_line != other.veh_fb_aux_bms_dis_on_line) {
      return false;
    }
    if (this->veh_fb_aux_scram != other.veh_fb_aux_scram) {
      return false;
    }
    if (this->veh_fb_aux_remote_close != other.veh_fb_aux_remote_close) {
      return false;
    }
    if (this->veh_fb_aux_remote_dis_on_line != other.veh_fb_aux_remote_dis_on_line) {
      return false;
    }
    return true;
  }
  bool operator!=(const VehDiagFb_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VehDiagFb_

// alias to use template instance with default allocator
using VehDiagFb =
  yhs_can_interfaces::msg::VehDiagFb_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yhs_can_interfaces

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__VEH_DIAG_FB__STRUCT_HPP_
