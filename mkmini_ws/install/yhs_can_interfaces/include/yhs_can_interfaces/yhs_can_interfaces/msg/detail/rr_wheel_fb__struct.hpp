// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yhs_can_interfaces:msg/RrWheelFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__RR_WHEEL_FB__STRUCT_HPP_
#define YHS_CAN_INTERFACES__MSG__DETAIL__RR_WHEEL_FB__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__yhs_can_interfaces__msg__RrWheelFb __attribute__((deprecated))
#else
# define DEPRECATED__yhs_can_interfaces__msg__RrWheelFb __declspec(deprecated)
#endif

namespace yhs_can_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RrWheelFb_
{
  using Type = RrWheelFb_<ContainerAllocator>;

  explicit RrWheelFb_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rr_wheel_fb_velocity = 0.0f;
      this->rr_wheel_fb_pulse = 0l;
    }
  }

  explicit RrWheelFb_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rr_wheel_fb_velocity = 0.0f;
      this->rr_wheel_fb_pulse = 0l;
    }
  }

  // field types and members
  using _rr_wheel_fb_velocity_type =
    float;
  _rr_wheel_fb_velocity_type rr_wheel_fb_velocity;
  using _rr_wheel_fb_pulse_type =
    int32_t;
  _rr_wheel_fb_pulse_type rr_wheel_fb_pulse;

  // setters for named parameter idiom
  Type & set__rr_wheel_fb_velocity(
    const float & _arg)
  {
    this->rr_wheel_fb_velocity = _arg;
    return *this;
  }
  Type & set__rr_wheel_fb_pulse(
    const int32_t & _arg)
  {
    this->rr_wheel_fb_pulse = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yhs_can_interfaces::msg::RrWheelFb_<ContainerAllocator> *;
  using ConstRawPtr =
    const yhs_can_interfaces::msg::RrWheelFb_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yhs_can_interfaces::msg::RrWheelFb_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yhs_can_interfaces::msg::RrWheelFb_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yhs_can_interfaces::msg::RrWheelFb_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yhs_can_interfaces::msg::RrWheelFb_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yhs_can_interfaces::msg::RrWheelFb_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yhs_can_interfaces::msg::RrWheelFb_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yhs_can_interfaces::msg::RrWheelFb_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yhs_can_interfaces::msg::RrWheelFb_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yhs_can_interfaces__msg__RrWheelFb
    std::shared_ptr<yhs_can_interfaces::msg::RrWheelFb_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yhs_can_interfaces__msg__RrWheelFb
    std::shared_ptr<yhs_can_interfaces::msg::RrWheelFb_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RrWheelFb_ & other) const
  {
    if (this->rr_wheel_fb_velocity != other.rr_wheel_fb_velocity) {
      return false;
    }
    if (this->rr_wheel_fb_pulse != other.rr_wheel_fb_pulse) {
      return false;
    }
    return true;
  }
  bool operator!=(const RrWheelFb_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RrWheelFb_

// alias to use template instance with default allocator
using RrWheelFb =
  yhs_can_interfaces::msg::RrWheelFb_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yhs_can_interfaces

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__RR_WHEEL_FB__STRUCT_HPP_
