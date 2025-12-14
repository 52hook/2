// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from yhs_can_interfaces:msg/BmsInfoFb.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "yhs_can_interfaces/msg/detail/bms_info_fb__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace yhs_can_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void BmsInfoFb_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) yhs_can_interfaces::msg::BmsInfoFb(_init);
}

void BmsInfoFb_fini_function(void * message_memory)
{
  auto typed_message = static_cast<yhs_can_interfaces::msg::BmsInfoFb *>(message_memory);
  typed_message->~BmsInfoFb();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BmsInfoFb_message_member_array[3] = {
  {
    "bms_info_voltage",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yhs_can_interfaces::msg::BmsInfoFb, bms_info_voltage),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "bms_info_current",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yhs_can_interfaces::msg::BmsInfoFb, bms_info_current),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "bms_info_remaining_capacity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yhs_can_interfaces::msg::BmsInfoFb, bms_info_remaining_capacity),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BmsInfoFb_message_members = {
  "yhs_can_interfaces::msg",  // message namespace
  "BmsInfoFb",  // message name
  3,  // number of fields
  sizeof(yhs_can_interfaces::msg::BmsInfoFb),
  BmsInfoFb_message_member_array,  // message members
  BmsInfoFb_init_function,  // function to initialize message memory (memory has to be allocated)
  BmsInfoFb_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BmsInfoFb_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BmsInfoFb_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace yhs_can_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<yhs_can_interfaces::msg::BmsInfoFb>()
{
  return &::yhs_can_interfaces::msg::rosidl_typesupport_introspection_cpp::BmsInfoFb_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, yhs_can_interfaces, msg, BmsInfoFb)() {
  return &::yhs_can_interfaces::msg::rosidl_typesupport_introspection_cpp::BmsInfoFb_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
