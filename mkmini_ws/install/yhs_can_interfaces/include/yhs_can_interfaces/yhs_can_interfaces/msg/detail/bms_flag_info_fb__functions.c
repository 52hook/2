// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yhs_can_interfaces:msg/BmsFlagInfoFb.idl
// generated code does not contain a copyright notice
#include "yhs_can_interfaces/msg/detail/bms_flag_info_fb__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
yhs_can_interfaces__msg__BmsFlagInfoFb__init(yhs_can_interfaces__msg__BmsFlagInfoFb * msg)
{
  if (!msg) {
    return false;
  }
  // bms_flag_info_soc
  // bms_flag_info_single_ov
  // bms_flag_info_single_uv
  // bms_flag_info_ov
  // bms_flag_info_uv
  // bms_flag_info_charge_ot
  // bms_flag_info_charge_ut
  // bms_flag_info_discharge_ot
  // bms_flag_info_discharge_ut
  // bms_flag_info_charge_oc
  // bms_flag_info_discharge_oc
  // bms_flag_info_short
  // bms_flag_info_ic_error
  // bms_flag_info_lock_mos
  // bms_flag_info_charge_flag
  // bms_flag_info_soc_warning
  // bms_flag_info_soc_low_protection
  // bms_flag_info_hight_temperature
  // bms_flag_info_low_temperature
  return true;
}

void
yhs_can_interfaces__msg__BmsFlagInfoFb__fini(yhs_can_interfaces__msg__BmsFlagInfoFb * msg)
{
  if (!msg) {
    return;
  }
  // bms_flag_info_soc
  // bms_flag_info_single_ov
  // bms_flag_info_single_uv
  // bms_flag_info_ov
  // bms_flag_info_uv
  // bms_flag_info_charge_ot
  // bms_flag_info_charge_ut
  // bms_flag_info_discharge_ot
  // bms_flag_info_discharge_ut
  // bms_flag_info_charge_oc
  // bms_flag_info_discharge_oc
  // bms_flag_info_short
  // bms_flag_info_ic_error
  // bms_flag_info_lock_mos
  // bms_flag_info_charge_flag
  // bms_flag_info_soc_warning
  // bms_flag_info_soc_low_protection
  // bms_flag_info_hight_temperature
  // bms_flag_info_low_temperature
}

bool
yhs_can_interfaces__msg__BmsFlagInfoFb__are_equal(const yhs_can_interfaces__msg__BmsFlagInfoFb * lhs, const yhs_can_interfaces__msg__BmsFlagInfoFb * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // bms_flag_info_soc
  if (lhs->bms_flag_info_soc != rhs->bms_flag_info_soc) {
    return false;
  }
  // bms_flag_info_single_ov
  if (lhs->bms_flag_info_single_ov != rhs->bms_flag_info_single_ov) {
    return false;
  }
  // bms_flag_info_single_uv
  if (lhs->bms_flag_info_single_uv != rhs->bms_flag_info_single_uv) {
    return false;
  }
  // bms_flag_info_ov
  if (lhs->bms_flag_info_ov != rhs->bms_flag_info_ov) {
    return false;
  }
  // bms_flag_info_uv
  if (lhs->bms_flag_info_uv != rhs->bms_flag_info_uv) {
    return false;
  }
  // bms_flag_info_charge_ot
  if (lhs->bms_flag_info_charge_ot != rhs->bms_flag_info_charge_ot) {
    return false;
  }
  // bms_flag_info_charge_ut
  if (lhs->bms_flag_info_charge_ut != rhs->bms_flag_info_charge_ut) {
    return false;
  }
  // bms_flag_info_discharge_ot
  if (lhs->bms_flag_info_discharge_ot != rhs->bms_flag_info_discharge_ot) {
    return false;
  }
  // bms_flag_info_discharge_ut
  if (lhs->bms_flag_info_discharge_ut != rhs->bms_flag_info_discharge_ut) {
    return false;
  }
  // bms_flag_info_charge_oc
  if (lhs->bms_flag_info_charge_oc != rhs->bms_flag_info_charge_oc) {
    return false;
  }
  // bms_flag_info_discharge_oc
  if (lhs->bms_flag_info_discharge_oc != rhs->bms_flag_info_discharge_oc) {
    return false;
  }
  // bms_flag_info_short
  if (lhs->bms_flag_info_short != rhs->bms_flag_info_short) {
    return false;
  }
  // bms_flag_info_ic_error
  if (lhs->bms_flag_info_ic_error != rhs->bms_flag_info_ic_error) {
    return false;
  }
  // bms_flag_info_lock_mos
  if (lhs->bms_flag_info_lock_mos != rhs->bms_flag_info_lock_mos) {
    return false;
  }
  // bms_flag_info_charge_flag
  if (lhs->bms_flag_info_charge_flag != rhs->bms_flag_info_charge_flag) {
    return false;
  }
  // bms_flag_info_soc_warning
  if (lhs->bms_flag_info_soc_warning != rhs->bms_flag_info_soc_warning) {
    return false;
  }
  // bms_flag_info_soc_low_protection
  if (lhs->bms_flag_info_soc_low_protection != rhs->bms_flag_info_soc_low_protection) {
    return false;
  }
  // bms_flag_info_hight_temperature
  if (lhs->bms_flag_info_hight_temperature != rhs->bms_flag_info_hight_temperature) {
    return false;
  }
  // bms_flag_info_low_temperature
  if (lhs->bms_flag_info_low_temperature != rhs->bms_flag_info_low_temperature) {
    return false;
  }
  return true;
}

bool
yhs_can_interfaces__msg__BmsFlagInfoFb__copy(
  const yhs_can_interfaces__msg__BmsFlagInfoFb * input,
  yhs_can_interfaces__msg__BmsFlagInfoFb * output)
{
  if (!input || !output) {
    return false;
  }
  // bms_flag_info_soc
  output->bms_flag_info_soc = input->bms_flag_info_soc;
  // bms_flag_info_single_ov
  output->bms_flag_info_single_ov = input->bms_flag_info_single_ov;
  // bms_flag_info_single_uv
  output->bms_flag_info_single_uv = input->bms_flag_info_single_uv;
  // bms_flag_info_ov
  output->bms_flag_info_ov = input->bms_flag_info_ov;
  // bms_flag_info_uv
  output->bms_flag_info_uv = input->bms_flag_info_uv;
  // bms_flag_info_charge_ot
  output->bms_flag_info_charge_ot = input->bms_flag_info_charge_ot;
  // bms_flag_info_charge_ut
  output->bms_flag_info_charge_ut = input->bms_flag_info_charge_ut;
  // bms_flag_info_discharge_ot
  output->bms_flag_info_discharge_ot = input->bms_flag_info_discharge_ot;
  // bms_flag_info_discharge_ut
  output->bms_flag_info_discharge_ut = input->bms_flag_info_discharge_ut;
  // bms_flag_info_charge_oc
  output->bms_flag_info_charge_oc = input->bms_flag_info_charge_oc;
  // bms_flag_info_discharge_oc
  output->bms_flag_info_discharge_oc = input->bms_flag_info_discharge_oc;
  // bms_flag_info_short
  output->bms_flag_info_short = input->bms_flag_info_short;
  // bms_flag_info_ic_error
  output->bms_flag_info_ic_error = input->bms_flag_info_ic_error;
  // bms_flag_info_lock_mos
  output->bms_flag_info_lock_mos = input->bms_flag_info_lock_mos;
  // bms_flag_info_charge_flag
  output->bms_flag_info_charge_flag = input->bms_flag_info_charge_flag;
  // bms_flag_info_soc_warning
  output->bms_flag_info_soc_warning = input->bms_flag_info_soc_warning;
  // bms_flag_info_soc_low_protection
  output->bms_flag_info_soc_low_protection = input->bms_flag_info_soc_low_protection;
  // bms_flag_info_hight_temperature
  output->bms_flag_info_hight_temperature = input->bms_flag_info_hight_temperature;
  // bms_flag_info_low_temperature
  output->bms_flag_info_low_temperature = input->bms_flag_info_low_temperature;
  return true;
}

yhs_can_interfaces__msg__BmsFlagInfoFb *
yhs_can_interfaces__msg__BmsFlagInfoFb__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__BmsFlagInfoFb * msg = (yhs_can_interfaces__msg__BmsFlagInfoFb *)allocator.allocate(sizeof(yhs_can_interfaces__msg__BmsFlagInfoFb), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yhs_can_interfaces__msg__BmsFlagInfoFb));
  bool success = yhs_can_interfaces__msg__BmsFlagInfoFb__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yhs_can_interfaces__msg__BmsFlagInfoFb__destroy(yhs_can_interfaces__msg__BmsFlagInfoFb * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yhs_can_interfaces__msg__BmsFlagInfoFb__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence__init(yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__BmsFlagInfoFb * data = NULL;

  if (size) {
    data = (yhs_can_interfaces__msg__BmsFlagInfoFb *)allocator.zero_allocate(size, sizeof(yhs_can_interfaces__msg__BmsFlagInfoFb), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yhs_can_interfaces__msg__BmsFlagInfoFb__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yhs_can_interfaces__msg__BmsFlagInfoFb__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence__fini(yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      yhs_can_interfaces__msg__BmsFlagInfoFb__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence *
yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence * array = (yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence *)allocator.allocate(sizeof(yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence__destroy(yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence__are_equal(const yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence * lhs, const yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yhs_can_interfaces__msg__BmsFlagInfoFb__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence__copy(
  const yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence * input,
  yhs_can_interfaces__msg__BmsFlagInfoFb__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yhs_can_interfaces__msg__BmsFlagInfoFb);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yhs_can_interfaces__msg__BmsFlagInfoFb * data =
      (yhs_can_interfaces__msg__BmsFlagInfoFb *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yhs_can_interfaces__msg__BmsFlagInfoFb__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yhs_can_interfaces__msg__BmsFlagInfoFb__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yhs_can_interfaces__msg__BmsFlagInfoFb__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
