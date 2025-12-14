// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yhs_can_interfaces:msg/BmsInfoFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__BMS_INFO_FB__STRUCT_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__BMS_INFO_FB__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__yhs_can_interfaces__msg__BmsInfoFb __attribute__((deprecated))
#else
# define DEPRECATED__yhs_can_interfaces__msg__BmsInfoFb __declspec(deprecated)
#endif

namespace yhs_can_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BmsInfoFb_
{
  using Type = BmsInfoFb_<ContainerAllocator>;

  explicit BmsInfoFb_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bms_info_voltage = 0.0f;
      this->bms_info_current = 0.0f;
      this->bms_info_remaining_capacity = 0.0f;
    }
  }

  explicit BmsInfoFb_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bms_info_voltage = 0.0f;
      this->bms_info_current = 0.0f;
      this->bms_info_remaining_capacity = 0.0f;
    }
  }

  // field types and members
  using _bms_info_voltage_type =
    float;
  _bms_info_voltage_type bms_info_voltage;
  using _bms_info_current_type =
    float;
  _bms_info_current_type bms_info_current;
  using _bms_info_remaining_capacity_type =
    float;
  _bms_info_remaining_capacity_type bms_info_remaining_capacity;

  // setters for named parameter idiom
  Type & set__bms_info_voltage(
    const float & _arg)
  {
    this->bms_info_voltage = _arg;
    return *this;
  }
  Type & set__bms_info_current(
    const float & _arg)
  {
    this->bms_info_current = _arg;
    return *this;
  }
  Type & set__bms_info_remaining_capacity(
    const float & _arg)
  {
    this->bms_info_remaining_capacity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yhs_can_interfaces::msg::BmsInfoFb_<ContainerAllocator> *;
  using ConstRawPtr =
    const yhs_can_interfaces::msg::BmsInfoFb_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yhs_can_interfaces::msg::BmsInfoFb_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yhs_can_interfaces::msg::BmsInfoFb_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yhs_can_interfaces::msg::BmsInfoFb_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yhs_can_interfaces::msg::BmsInfoFb_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yhs_can_interfaces::msg::BmsInfoFb_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yhs_can_interfaces::msg::BmsInfoFb_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yhs_can_interfaces::msg::BmsInfoFb_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yhs_can_interfaces::msg::BmsInfoFb_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yhs_can_interfaces__msg__BmsInfoFb
    std::shared_ptr<yhs_can_interfaces::msg::BmsInfoFb_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yhs_can_interfaces__msg__BmsInfoFb
    std::shared_ptr<yhs_can_interfaces::msg::BmsInfoFb_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BmsInfoFb_ & other) const
  {
    if (this->bms_info_voltage != other.bms_info_voltage) {
      return false;
    }
    if (this->bms_info_current != other.bms_info_current) {
      return false;
    }
    if (this->bms_info_remaining_capacity != other.bms_info_remaining_capacity) {
      return false;
    }
    return true;
  }
  bool operator!=(const BmsInfoFb_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BmsInfoFb_

// alias to use template instance with default allocator
using BmsInfoFb =
  yhs_can_interfaces::msg::BmsInfoFb_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yhs_can_interfaces

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__BMS_INFO_FB__STRUCT_HPP_
