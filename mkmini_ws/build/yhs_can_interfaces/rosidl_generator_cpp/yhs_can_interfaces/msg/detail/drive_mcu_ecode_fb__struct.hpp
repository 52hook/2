// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yhs_can_interfaces:msg/DriveMcuEcodeFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__DRIVE_MCU_ECODE_FB__STRUCT_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__DRIVE_MCU_ECODE_FB__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__yhs_can_interfaces__msg__DriveMcuEcodeFb __attribute__((deprecated))
#else
# define DEPRECATED__yhs_can_interfaces__msg__DriveMcuEcodeFb __declspec(deprecated)
#endif

namespace yhs_can_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DriveMcuEcodeFb_
{
  using Type = DriveMcuEcodeFb_<ContainerAllocator>;

  explicit DriveMcuEcodeFb_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drive_fb_mcuecode = 0l;
    }
  }

  explicit DriveMcuEcodeFb_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drive_fb_mcuecode = 0l;
    }
  }

  // field types and members
  using _drive_fb_mcuecode_type =
    int32_t;
  _drive_fb_mcuecode_type drive_fb_mcuecode;

  // setters for named parameter idiom
  Type & set__drive_fb_mcuecode(
    const int32_t & _arg)
  {
    this->drive_fb_mcuecode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yhs_can_interfaces::msg::DriveMcuEcodeFb_<ContainerAllocator> *;
  using ConstRawPtr =
    const yhs_can_interfaces::msg::DriveMcuEcodeFb_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yhs_can_interfaces::msg::DriveMcuEcodeFb_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yhs_can_interfaces::msg::DriveMcuEcodeFb_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yhs_can_interfaces::msg::DriveMcuEcodeFb_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yhs_can_interfaces::msg::DriveMcuEcodeFb_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yhs_can_interfaces::msg::DriveMcuEcodeFb_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yhs_can_interfaces::msg::DriveMcuEcodeFb_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yhs_can_interfaces::msg::DriveMcuEcodeFb_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yhs_can_interfaces::msg::DriveMcuEcodeFb_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yhs_can_interfaces__msg__DriveMcuEcodeFb
    std::shared_ptr<yhs_can_interfaces::msg::DriveMcuEcodeFb_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yhs_can_interfaces__msg__DriveMcuEcodeFb
    std::shared_ptr<yhs_can_interfaces::msg::DriveMcuEcodeFb_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DriveMcuEcodeFb_ & other) const
  {
    if (this->drive_fb_mcuecode != other.drive_fb_mcuecode) {
      return false;
    }
    return true;
  }
  bool operator!=(const DriveMcuEcodeFb_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DriveMcuEcodeFb_

// alias to use template instance with default allocator
using DriveMcuEcodeFb =
  yhs_can_interfaces::msg::DriveMcuEcodeFb_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yhs_can_interfaces

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__DRIVE_MCU_ECODE_FB__STRUCT_HPP_
