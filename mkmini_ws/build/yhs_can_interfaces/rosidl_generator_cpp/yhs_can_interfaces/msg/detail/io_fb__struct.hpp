// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yhs_can_interfaces:msg/IoFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__IO_FB__STRUCT_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__IO_FB__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__yhs_can_interfaces__msg__IoFb __attribute__((deprecated))
#else
# define DEPRECATED__yhs_can_interfaces__msg__IoFb __declspec(deprecated)
#endif

namespace yhs_can_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct IoFb_
{
  using Type = IoFb_<ContainerAllocator>;

  explicit IoFb_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->io_fb_enable = false;
      this->io_fb_turn_lamp = 0;
      this->io_fb_braking_lamp = false;
      this->io_fb_fm_impact_sensor = false;
      this->io_fb_rm_impact_sensor = false;
      this->io_fb_dis_charge = false;
      this->io_fb_charge_en = false;
      this->io_fb_scram_st = false;
    }
  }

  explicit IoFb_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->io_fb_enable = false;
      this->io_fb_turn_lamp = 0;
      this->io_fb_braking_lamp = false;
      this->io_fb_fm_impact_sensor = false;
      this->io_fb_rm_impact_sensor = false;
      this->io_fb_dis_charge = false;
      this->io_fb_charge_en = false;
      this->io_fb_scram_st = false;
    }
  }

  // field types and members
  using _io_fb_enable_type =
    bool;
  _io_fb_enable_type io_fb_enable;
  using _io_fb_turn_lamp_type =
    int8_t;
  _io_fb_turn_lamp_type io_fb_turn_lamp;
  using _io_fb_braking_lamp_type =
    bool;
  _io_fb_braking_lamp_type io_fb_braking_lamp;
  using _io_fb_fm_impact_sensor_type =
    bool;
  _io_fb_fm_impact_sensor_type io_fb_fm_impact_sensor;
  using _io_fb_rm_impact_sensor_type =
    bool;
  _io_fb_rm_impact_sensor_type io_fb_rm_impact_sensor;
  using _io_fb_dis_charge_type =
    bool;
  _io_fb_dis_charge_type io_fb_dis_charge;
  using _io_fb_charge_en_type =
    bool;
  _io_fb_charge_en_type io_fb_charge_en;
  using _io_fb_scram_st_type =
    bool;
  _io_fb_scram_st_type io_fb_scram_st;

  // setters for named parameter idiom
  Type & set__io_fb_enable(
    const bool & _arg)
  {
    this->io_fb_enable = _arg;
    return *this;
  }
  Type & set__io_fb_turn_lamp(
    const int8_t & _arg)
  {
    this->io_fb_turn_lamp = _arg;
    return *this;
  }
  Type & set__io_fb_braking_lamp(
    const bool & _arg)
  {
    this->io_fb_braking_lamp = _arg;
    return *this;
  }
  Type & set__io_fb_fm_impact_sensor(
    const bool & _arg)
  {
    this->io_fb_fm_impact_sensor = _arg;
    return *this;
  }
  Type & set__io_fb_rm_impact_sensor(
    const bool & _arg)
  {
    this->io_fb_rm_impact_sensor = _arg;
    return *this;
  }
  Type & set__io_fb_dis_charge(
    const bool & _arg)
  {
    this->io_fb_dis_charge = _arg;
    return *this;
  }
  Type & set__io_fb_charge_en(
    const bool & _arg)
  {
    this->io_fb_charge_en = _arg;
    return *this;
  }
  Type & set__io_fb_scram_st(
    const bool & _arg)
  {
    this->io_fb_scram_st = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yhs_can_interfaces::msg::IoFb_<ContainerAllocator> *;
  using ConstRawPtr =
    const yhs_can_interfaces::msg::IoFb_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yhs_can_interfaces::msg::IoFb_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yhs_can_interfaces::msg::IoFb_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yhs_can_interfaces::msg::IoFb_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yhs_can_interfaces::msg::IoFb_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yhs_can_interfaces::msg::IoFb_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yhs_can_interfaces::msg::IoFb_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yhs_can_interfaces::msg::IoFb_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yhs_can_interfaces::msg::IoFb_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yhs_can_interfaces__msg__IoFb
    std::shared_ptr<yhs_can_interfaces::msg::IoFb_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yhs_can_interfaces__msg__IoFb
    std::shared_ptr<yhs_can_interfaces::msg::IoFb_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IoFb_ & other) const
  {
    if (this->io_fb_enable != other.io_fb_enable) {
      return false;
    }
    if (this->io_fb_turn_lamp != other.io_fb_turn_lamp) {
      return false;
    }
    if (this->io_fb_braking_lamp != other.io_fb_braking_lamp) {
      return false;
    }
    if (this->io_fb_fm_impact_sensor != other.io_fb_fm_impact_sensor) {
      return false;
    }
    if (this->io_fb_rm_impact_sensor != other.io_fb_rm_impact_sensor) {
      return false;
    }
    if (this->io_fb_dis_charge != other.io_fb_dis_charge) {
      return false;
    }
    if (this->io_fb_charge_en != other.io_fb_charge_en) {
      return false;
    }
    if (this->io_fb_scram_st != other.io_fb_scram_st) {
      return false;
    }
    return true;
  }
  bool operator!=(const IoFb_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IoFb_

// alias to use template instance with default allocator
using IoFb =
  yhs_can_interfaces::msg::IoFb_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yhs_can_interfaces

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__IO_FB__STRUCT_HPP_
