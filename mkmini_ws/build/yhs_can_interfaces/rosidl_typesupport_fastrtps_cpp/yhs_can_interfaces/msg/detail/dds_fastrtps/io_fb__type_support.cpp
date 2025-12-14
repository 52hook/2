// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from yhs_can_interfaces:msg/IoFb.idl
// generated code does not contain a copyright notice
#include "yhs_can_interfaces/msg/detail/io_fb__rosidl_typesupport_fastrtps_cpp.hpp"
#include "yhs_can_interfaces/msg/detail/io_fb__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace yhs_can_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yhs_can_interfaces
cdr_serialize(
  const yhs_can_interfaces::msg::IoFb & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: io_fb_enable
  cdr << (ros_message.io_fb_enable ? true : false);
  // Member: io_fb_turn_lamp
  cdr << ros_message.io_fb_turn_lamp;
  // Member: io_fb_braking_lamp
  cdr << (ros_message.io_fb_braking_lamp ? true : false);
  // Member: io_fb_fm_impact_sensor
  cdr << (ros_message.io_fb_fm_impact_sensor ? true : false);
  // Member: io_fb_rm_impact_sensor
  cdr << (ros_message.io_fb_rm_impact_sensor ? true : false);
  // Member: io_fb_dis_charge
  cdr << (ros_message.io_fb_dis_charge ? true : false);
  // Member: io_fb_charge_en
  cdr << (ros_message.io_fb_charge_en ? true : false);
  // Member: io_fb_scram_st
  cdr << (ros_message.io_fb_scram_st ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yhs_can_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  yhs_can_interfaces::msg::IoFb & ros_message)
{
  // Member: io_fb_enable
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.io_fb_enable = tmp ? true : false;
  }

  // Member: io_fb_turn_lamp
  cdr >> ros_message.io_fb_turn_lamp;

  // Member: io_fb_braking_lamp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.io_fb_braking_lamp = tmp ? true : false;
  }

  // Member: io_fb_fm_impact_sensor
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.io_fb_fm_impact_sensor = tmp ? true : false;
  }

  // Member: io_fb_rm_impact_sensor
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.io_fb_rm_impact_sensor = tmp ? true : false;
  }

  // Member: io_fb_dis_charge
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.io_fb_dis_charge = tmp ? true : false;
  }

  // Member: io_fb_charge_en
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.io_fb_charge_en = tmp ? true : false;
  }

  // Member: io_fb_scram_st
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.io_fb_scram_st = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yhs_can_interfaces
get_serialized_size(
  const yhs_can_interfaces::msg::IoFb & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: io_fb_enable
  {
    size_t item_size = sizeof(ros_message.io_fb_enable);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: io_fb_turn_lamp
  {
    size_t item_size = sizeof(ros_message.io_fb_turn_lamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: io_fb_braking_lamp
  {
    size_t item_size = sizeof(ros_message.io_fb_braking_lamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: io_fb_fm_impact_sensor
  {
    size_t item_size = sizeof(ros_message.io_fb_fm_impact_sensor);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: io_fb_rm_impact_sensor
  {
    size_t item_size = sizeof(ros_message.io_fb_rm_impact_sensor);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: io_fb_dis_charge
  {
    size_t item_size = sizeof(ros_message.io_fb_dis_charge);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: io_fb_charge_en
  {
    size_t item_size = sizeof(ros_message.io_fb_charge_en);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: io_fb_scram_st
  {
    size_t item_size = sizeof(ros_message.io_fb_scram_st);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yhs_can_interfaces
max_serialized_size_IoFb(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: io_fb_enable
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: io_fb_turn_lamp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: io_fb_braking_lamp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: io_fb_fm_impact_sensor
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: io_fb_rm_impact_sensor
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: io_fb_dis_charge
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: io_fb_charge_en
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: io_fb_scram_st
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = yhs_can_interfaces::msg::IoFb;
    is_plain =
      (
      offsetof(DataType, io_fb_scram_st) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _IoFb__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const yhs_can_interfaces::msg::IoFb *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _IoFb__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<yhs_can_interfaces::msg::IoFb *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _IoFb__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const yhs_can_interfaces::msg::IoFb *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _IoFb__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_IoFb(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _IoFb__callbacks = {
  "yhs_can_interfaces::msg",
  "IoFb",
  _IoFb__cdr_serialize,
  _IoFb__cdr_deserialize,
  _IoFb__get_serialized_size,
  _IoFb__max_serialized_size
};

static rosidl_message_type_support_t _IoFb__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_IoFb__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace yhs_can_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_yhs_can_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<yhs_can_interfaces::msg::IoFb>()
{
  return &yhs_can_interfaces::msg::typesupport_fastrtps_cpp::_IoFb__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, yhs_can_interfaces, msg, IoFb)() {
  return &yhs_can_interfaces::msg::typesupport_fastrtps_cpp::_IoFb__handle;
}

#ifdef __cplusplus
}
#endif
