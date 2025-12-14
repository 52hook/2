// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from yhs_can_interfaces:msg/BmsFlagInfoFb.idl
// generated code does not contain a copyright notice
#include "yhs_can_interfaces/msg/detail/bms_flag_info_fb__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "yhs_can_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "yhs_can_interfaces/msg/detail/bms_flag_info_fb__struct.h"
#include "yhs_can_interfaces/msg/detail/bms_flag_info_fb__functions.h"
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


using _BmsFlagInfoFb__ros_msg_type = yhs_can_interfaces__msg__BmsFlagInfoFb;

static bool _BmsFlagInfoFb__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _BmsFlagInfoFb__ros_msg_type * ros_message = static_cast<const _BmsFlagInfoFb__ros_msg_type *>(untyped_ros_message);
  // Field name: bms_flag_info_soc
  {
    cdr << ros_message->bms_flag_info_soc;
  }

  // Field name: bms_flag_info_single_ov
  {
    cdr << (ros_message->bms_flag_info_single_ov ? true : false);
  }

  // Field name: bms_flag_info_single_uv
  {
    cdr << (ros_message->bms_flag_info_single_uv ? true : false);
  }

  // Field name: bms_flag_info_ov
  {
    cdr << (ros_message->bms_flag_info_ov ? true : false);
  }

  // Field name: bms_flag_info_uv
  {
    cdr << (ros_message->bms_flag_info_uv ? true : false);
  }

  // Field name: bms_flag_info_charge_ot
  {
    cdr << (ros_message->bms_flag_info_charge_ot ? true : false);
  }

  // Field name: bms_flag_info_charge_ut
  {
    cdr << (ros_message->bms_flag_info_charge_ut ? true : false);
  }

  // Field name: bms_flag_info_discharge_ot
  {
    cdr << (ros_message->bms_flag_info_discharge_ot ? true : false);
  }

  // Field name: bms_flag_info_discharge_ut
  {
    cdr << (ros_message->bms_flag_info_discharge_ut ? true : false);
  }

  // Field name: bms_flag_info_charge_oc
  {
    cdr << (ros_message->bms_flag_info_charge_oc ? true : false);
  }

  // Field name: bms_flag_info_discharge_oc
  {
    cdr << (ros_message->bms_flag_info_discharge_oc ? true : false);
  }

  // Field name: bms_flag_info_short
  {
    cdr << (ros_message->bms_flag_info_short ? true : false);
  }

  // Field name: bms_flag_info_ic_error
  {
    cdr << (ros_message->bms_flag_info_ic_error ? true : false);
  }

  // Field name: bms_flag_info_lock_mos
  {
    cdr << (ros_message->bms_flag_info_lock_mos ? true : false);
  }

  // Field name: bms_flag_info_charge_flag
  {
    cdr << (ros_message->bms_flag_info_charge_flag ? true : false);
  }

  // Field name: bms_flag_info_soc_warning
  {
    cdr << (ros_message->bms_flag_info_soc_warning ? true : false);
  }

  // Field name: bms_flag_info_soc_low_protection
  {
    cdr << (ros_message->bms_flag_info_soc_low_protection ? true : false);
  }

  // Field name: bms_flag_info_hight_temperature
  {
    cdr << ros_message->bms_flag_info_hight_temperature;
  }

  // Field name: bms_flag_info_low_temperature
  {
    cdr << ros_message->bms_flag_info_low_temperature;
  }

  return true;
}

static bool _BmsFlagInfoFb__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _BmsFlagInfoFb__ros_msg_type * ros_message = static_cast<_BmsFlagInfoFb__ros_msg_type *>(untyped_ros_message);
  // Field name: bms_flag_info_soc
  {
    cdr >> ros_message->bms_flag_info_soc;
  }

  // Field name: bms_flag_info_single_ov
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bms_flag_info_single_ov = tmp ? true : false;
  }

  // Field name: bms_flag_info_single_uv
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bms_flag_info_single_uv = tmp ? true : false;
  }

  // Field name: bms_flag_info_ov
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bms_flag_info_ov = tmp ? true : false;
  }

  // Field name: bms_flag_info_uv
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bms_flag_info_uv = tmp ? true : false;
  }

  // Field name: bms_flag_info_charge_ot
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bms_flag_info_charge_ot = tmp ? true : false;
  }

  // Field name: bms_flag_info_charge_ut
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bms_flag_info_charge_ut = tmp ? true : false;
  }

  // Field name: bms_flag_info_discharge_ot
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bms_flag_info_discharge_ot = tmp ? true : false;
  }

  // Field name: bms_flag_info_discharge_ut
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bms_flag_info_discharge_ut = tmp ? true : false;
  }

  // Field name: bms_flag_info_charge_oc
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bms_flag_info_charge_oc = tmp ? true : false;
  }

  // Field name: bms_flag_info_discharge_oc
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bms_flag_info_discharge_oc = tmp ? true : false;
  }

  // Field name: bms_flag_info_short
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bms_flag_info_short = tmp ? true : false;
  }

  // Field name: bms_flag_info_ic_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bms_flag_info_ic_error = tmp ? true : false;
  }

  // Field name: bms_flag_info_lock_mos
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bms_flag_info_lock_mos = tmp ? true : false;
  }

  // Field name: bms_flag_info_charge_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bms_flag_info_charge_flag = tmp ? true : false;
  }

  // Field name: bms_flag_info_soc_warning
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bms_flag_info_soc_warning = tmp ? true : false;
  }

  // Field name: bms_flag_info_soc_low_protection
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bms_flag_info_soc_low_protection = tmp ? true : false;
  }

  // Field name: bms_flag_info_hight_temperature
  {
    cdr >> ros_message->bms_flag_info_hight_temperature;
  }

  // Field name: bms_flag_info_low_temperature
  {
    cdr >> ros_message->bms_flag_info_low_temperature;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_yhs_can_interfaces
size_t get_serialized_size_yhs_can_interfaces__msg__BmsFlagInfoFb(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BmsFlagInfoFb__ros_msg_type * ros_message = static_cast<const _BmsFlagInfoFb__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name bms_flag_info_soc
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_soc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_single_ov
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_single_ov);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_single_uv
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_single_uv);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_ov
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_ov);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_uv
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_uv);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_charge_ot
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_charge_ot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_charge_ut
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_charge_ut);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_discharge_ot
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_discharge_ot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_discharge_ut
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_discharge_ut);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_charge_oc
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_charge_oc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_discharge_oc
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_discharge_oc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_short
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_short);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_ic_error
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_ic_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_lock_mos
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_lock_mos);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_charge_flag
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_charge_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_soc_warning
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_soc_warning);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_soc_low_protection
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_soc_low_protection);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_hight_temperature
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_hight_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bms_flag_info_low_temperature
  {
    size_t item_size = sizeof(ros_message->bms_flag_info_low_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _BmsFlagInfoFb__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_yhs_can_interfaces__msg__BmsFlagInfoFb(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_yhs_can_interfaces
size_t max_serialized_size_yhs_can_interfaces__msg__BmsFlagInfoFb(
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

  // member: bms_flag_info_soc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_single_ov
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_single_uv
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_ov
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_uv
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_charge_ot
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_charge_ut
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_discharge_ot
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_discharge_ut
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_charge_oc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_discharge_oc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_short
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_ic_error
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_lock_mos
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_charge_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_soc_warning
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_soc_low_protection
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bms_flag_info_hight_temperature
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: bms_flag_info_low_temperature
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = yhs_can_interfaces__msg__BmsFlagInfoFb;
    is_plain =
      (
      offsetof(DataType, bms_flag_info_low_temperature) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _BmsFlagInfoFb__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_yhs_can_interfaces__msg__BmsFlagInfoFb(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_BmsFlagInfoFb = {
  "yhs_can_interfaces::msg",
  "BmsFlagInfoFb",
  _BmsFlagInfoFb__cdr_serialize,
  _BmsFlagInfoFb__cdr_deserialize,
  _BmsFlagInfoFb__get_serialized_size,
  _BmsFlagInfoFb__max_serialized_size
};

static rosidl_message_type_support_t _BmsFlagInfoFb__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BmsFlagInfoFb,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, yhs_can_interfaces, msg, BmsFlagInfoFb)() {
  return &_BmsFlagInfoFb__type_support;
}

#if defined(__cplusplus)
}
#endif
