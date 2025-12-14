// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from yhs_can_interfaces:msg/IoCmd.idl
// generated code does not contain a copyright notice
#include "yhs_can_interfaces/msg/detail/io_cmd__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "yhs_can_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "yhs_can_interfaces/msg/detail/io_cmd__struct.h"
#include "yhs_can_interfaces/msg/detail/io_cmd__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _IoCmd__ros_msg_type = yhs_can_interfaces__msg__IoCmd;

static bool _IoCmd__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _IoCmd__ros_msg_type * ros_message = static_cast<const _IoCmd__ros_msg_type *>(untyped_ros_message);
  // Field name: io_cmd_enable
  {
    cdr << (ros_message->io_cmd_enable ? true : false);
  }

  // Field name: io_cmd_lower_beam_headlamp
  {
    cdr << (ros_message->io_cmd_lower_beam_headlamp ? true : false);
  }

  // Field name: io_cmd_upper_beam_headlamp
  {
    cdr << (ros_message->io_cmd_upper_beam_headlamp ? true : false);
  }

  // Field name: io_cmd_turn_lamp
  {
    cdr << ros_message->io_cmd_turn_lamp;
  }

  // Field name: io_cmd_braking_lamp
  {
    cdr << (ros_message->io_cmd_braking_lamp ? true : false);
  }

  // Field name: io_cmd_clearance_lamp
  {
    cdr << (ros_message->io_cmd_clearance_lamp ? true : false);
  }

  // Field name: io_cmd_fog_lamp
  {
    cdr << (ros_message->io_cmd_fog_lamp ? true : false);
  }

  // Field name: io_cmd_speaker
  {
    cdr << (ros_message->io_cmd_speaker ? true : false);
  }

  // Field name: io_cmd_dis_charge
  {
    cdr << (ros_message->io_cmd_dis_charge ? true : false);
  }

  return true;
}

static bool _IoCmd__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _IoCmd__ros_msg_type * ros_message = static_cast<_IoCmd__ros_msg_type *>(untyped_ros_message);
  // Field name: io_cmd_enable
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->io_cmd_enable = tmp ? true : false;
  }

  // Field name: io_cmd_lower_beam_headlamp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->io_cmd_lower_beam_headlamp = tmp ? true : false;
  }

  // Field name: io_cmd_upper_beam_headlamp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->io_cmd_upper_beam_headlamp = tmp ? true : false;
  }

  // Field name: io_cmd_turn_lamp
  {
    cdr >> ros_message->io_cmd_turn_lamp;
  }

  // Field name: io_cmd_braking_lamp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->io_cmd_braking_lamp = tmp ? true : false;
  }

  // Field name: io_cmd_clearance_lamp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->io_cmd_clearance_lamp = tmp ? true : false;
  }

  // Field name: io_cmd_fog_lamp
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->io_cmd_fog_lamp = tmp ? true : false;
  }

  // Field name: io_cmd_speaker
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->io_cmd_speaker = tmp ? true : false;
  }

  // Field name: io_cmd_dis_charge
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->io_cmd_dis_charge = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_yhs_can_interfaces
size_t get_serialized_size_yhs_can_interfaces__msg__IoCmd(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _IoCmd__ros_msg_type * ros_message = static_cast<const _IoCmd__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name io_cmd_enable
  {
    size_t item_size = sizeof(ros_message->io_cmd_enable);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name io_cmd_lower_beam_headlamp
  {
    size_t item_size = sizeof(ros_message->io_cmd_lower_beam_headlamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name io_cmd_upper_beam_headlamp
  {
    size_t item_size = sizeof(ros_message->io_cmd_upper_beam_headlamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name io_cmd_turn_lamp
  {
    size_t item_size = sizeof(ros_message->io_cmd_turn_lamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name io_cmd_braking_lamp
  {
    size_t item_size = sizeof(ros_message->io_cmd_braking_lamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name io_cmd_clearance_lamp
  {
    size_t item_size = sizeof(ros_message->io_cmd_clearance_lamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name io_cmd_fog_lamp
  {
    size_t item_size = sizeof(ros_message->io_cmd_fog_lamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name io_cmd_speaker
  {
    size_t item_size = sizeof(ros_message->io_cmd_speaker);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name io_cmd_dis_charge
  {
    size_t item_size = sizeof(ros_message->io_cmd_dis_charge);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _IoCmd__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_yhs_can_interfaces__msg__IoCmd(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_yhs_can_interfaces
size_t max_serialized_size_yhs_can_interfaces__msg__IoCmd(
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

  // member: io_cmd_enable
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: io_cmd_lower_beam_headlamp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: io_cmd_upper_beam_headlamp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: io_cmd_turn_lamp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: io_cmd_braking_lamp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: io_cmd_clearance_lamp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: io_cmd_fog_lamp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: io_cmd_speaker
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: io_cmd_dis_charge
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
    using DataType = yhs_can_interfaces__msg__IoCmd;
    is_plain =
      (
      offsetof(DataType, io_cmd_dis_charge) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _IoCmd__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_yhs_can_interfaces__msg__IoCmd(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_IoCmd = {
  "yhs_can_interfaces::msg",
  "IoCmd",
  _IoCmd__cdr_serialize,
  _IoCmd__cdr_deserialize,
  _IoCmd__get_serialized_size,
  _IoCmd__max_serialized_size
};

static rosidl_message_type_support_t _IoCmd__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_IoCmd,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yhs_can_interfaces, msg, IoCmd)() {
  return &_IoCmd__type_support;
}

#if defined(__cplusplus)
}
#endif
