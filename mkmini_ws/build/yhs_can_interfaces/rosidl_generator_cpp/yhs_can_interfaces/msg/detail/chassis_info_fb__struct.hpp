// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yhs_can_interfaces:msg/ChassisInfoFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__CHASSIS_INFO_FB__STRUCT_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__CHASSIS_INFO_FB__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'ctrl_fb'
#include "yhs_can_interfaces/msg/detail/ctrl_fb__struct.hpp"
// Member 'io_fb'
#include "yhs_can_interfaces/msg/detail/io_fb__struct.hpp"
// Member 'lr_wheel_fb'
#include "yhs_can_interfaces/msg/detail/lr_wheel_fb__struct.hpp"
// Member 'rr_wheel_fb'
#include "yhs_can_interfaces/msg/detail/rr_wheel_fb__struct.hpp"
// Member 'bms_flag_info_fb'
#include "yhs_can_interfaces/msg/detail/bms_flag_info_fb__struct.hpp"
// Member 'bms_info_fb'
#include "yhs_can_interfaces/msg/detail/bms_info_fb__struct.hpp"
// Member 'drive_mcu_ecode_fb'
#include "yhs_can_interfaces/msg/detail/drive_mcu_ecode_fb__struct.hpp"
// Member 'veh_diag_fb'
#include "yhs_can_interfaces/msg/detail/veh_diag_fb__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__yhs_can_interfaces__msg__ChassisInfoFb __attribute__((deprecated))
#else
# define DEPRECATED__yhs_can_interfaces__msg__ChassisInfoFb __declspec(deprecated)
#endif

namespace yhs_can_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ChassisInfoFb_
{
  using Type = ChassisInfoFb_<ContainerAllocator>;

  explicit ChassisInfoFb_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    ctrl_fb(_init),
    io_fb(_init),
    lr_wheel_fb(_init),
    rr_wheel_fb(_init),
    bms_flag_info_fb(_init),
    bms_info_fb(_init),
    drive_mcu_ecode_fb(_init),
    veh_diag_fb(_init)
  {
    (void)_init;
  }

  explicit ChassisInfoFb_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    ctrl_fb(_alloc, _init),
    io_fb(_alloc, _init),
    lr_wheel_fb(_alloc, _init),
    rr_wheel_fb(_alloc, _init),
    bms_flag_info_fb(_alloc, _init),
    bms_info_fb(_alloc, _init),
    drive_mcu_ecode_fb(_alloc, _init),
    veh_diag_fb(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _ctrl_fb_type =
    yhs_can_interfaces::msg::CtrlFb_<ContainerAllocator>;
  _ctrl_fb_type ctrl_fb;
  using _io_fb_type =
    yhs_can_interfaces::msg::IoFb_<ContainerAllocator>;
  _io_fb_type io_fb;
  using _lr_wheel_fb_type =
    yhs_can_interfaces::msg::LrWheelFb_<ContainerAllocator>;
  _lr_wheel_fb_type lr_wheel_fb;
  using _rr_wheel_fb_type =
    yhs_can_interfaces::msg::RrWheelFb_<ContainerAllocator>;
  _rr_wheel_fb_type rr_wheel_fb;
  using _bms_flag_info_fb_type =
    yhs_can_interfaces::msg::BmsFlagInfoFb_<ContainerAllocator>;
  _bms_flag_info_fb_type bms_flag_info_fb;
  using _bms_info_fb_type =
    yhs_can_interfaces::msg::BmsInfoFb_<ContainerAllocator>;
  _bms_info_fb_type bms_info_fb;
  using _drive_mcu_ecode_fb_type =
    yhs_can_interfaces::msg::DriveMcuEcodeFb_<ContainerAllocator>;
  _drive_mcu_ecode_fb_type drive_mcu_ecode_fb;
  using _veh_diag_fb_type =
    yhs_can_interfaces::msg::VehDiagFb_<ContainerAllocator>;
  _veh_diag_fb_type veh_diag_fb;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__ctrl_fb(
    const yhs_can_interfaces::msg::CtrlFb_<ContainerAllocator> & _arg)
  {
    this->ctrl_fb = _arg;
    return *this;
  }
  Type & set__io_fb(
    const yhs_can_interfaces::msg::IoFb_<ContainerAllocator> & _arg)
  {
    this->io_fb = _arg;
    return *this;
  }
  Type & set__lr_wheel_fb(
    const yhs_can_interfaces::msg::LrWheelFb_<ContainerAllocator> & _arg)
  {
    this->lr_wheel_fb = _arg;
    return *this;
  }
  Type & set__rr_wheel_fb(
    const yhs_can_interfaces::msg::RrWheelFb_<ContainerAllocator> & _arg)
  {
    this->rr_wheel_fb = _arg;
    return *this;
  }
  Type & set__bms_flag_info_fb(
    const yhs_can_interfaces::msg::BmsFlagInfoFb_<ContainerAllocator> & _arg)
  {
    this->bms_flag_info_fb = _arg;
    return *this;
  }
  Type & set__bms_info_fb(
    const yhs_can_interfaces::msg::BmsInfoFb_<ContainerAllocator> & _arg)
  {
    this->bms_info_fb = _arg;
    return *this;
  }
  Type & set__drive_mcu_ecode_fb(
    const yhs_can_interfaces::msg::DriveMcuEcodeFb_<ContainerAllocator> & _arg)
  {
    this->drive_mcu_ecode_fb = _arg;
    return *this;
  }
  Type & set__veh_diag_fb(
    const yhs_can_interfaces::msg::VehDiagFb_<ContainerAllocator> & _arg)
  {
    this->veh_diag_fb = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yhs_can_interfaces::msg::ChassisInfoFb_<ContainerAllocator> *;
  using ConstRawPtr =
    const yhs_can_interfaces::msg::ChassisInfoFb_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yhs_can_interfaces::msg::ChassisInfoFb_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yhs_can_interfaces::msg::ChassisInfoFb_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yhs_can_interfaces::msg::ChassisInfoFb_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yhs_can_interfaces::msg::ChassisInfoFb_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yhs_can_interfaces::msg::ChassisInfoFb_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yhs_can_interfaces::msg::ChassisInfoFb_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yhs_can_interfaces::msg::ChassisInfoFb_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yhs_can_interfaces::msg::ChassisInfoFb_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yhs_can_interfaces__msg__ChassisInfoFb
    std::shared_ptr<yhs_can_interfaces::msg::ChassisInfoFb_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yhs_can_interfaces__msg__ChassisInfoFb
    std::shared_ptr<yhs_can_interfaces::msg::ChassisInfoFb_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChassisInfoFb_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->ctrl_fb != other.ctrl_fb) {
      return false;
    }
    if (this->io_fb != other.io_fb) {
      return false;
    }
    if (this->lr_wheel_fb != other.lr_wheel_fb) {
      return false;
    }
    if (this->rr_wheel_fb != other.rr_wheel_fb) {
      return false;
    }
    if (this->bms_flag_info_fb != other.bms_flag_info_fb) {
      return false;
    }
    if (this->bms_info_fb != other.bms_info_fb) {
      return false;
    }
    if (this->drive_mcu_ecode_fb != other.drive_mcu_ecode_fb) {
      return false;
    }
    if (this->veh_diag_fb != other.veh_diag_fb) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChassisInfoFb_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChassisInfoFb_

// alias to use template instance with default allocator
using ChassisInfoFb =
  yhs_can_interfaces::msg::ChassisInfoFb_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yhs_can_interfaces

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__CHASSIS_INFO_FB__STRUCT_HPP_
