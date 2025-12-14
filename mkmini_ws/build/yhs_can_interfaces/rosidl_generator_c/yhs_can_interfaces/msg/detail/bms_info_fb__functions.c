// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yhs_can_interfaces:msg/BmsInfoFb.idl
// generated code does not contain a copyright notice
#include "yhs_can_interfaces/msg/detail/bms_info_fb__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
yhs_can_interfaces__msg__BmsInfoFb__init(yhs_can_interfaces__msg__BmsInfoFb * msg)
{
  if (!msg) {
    return false;
  }
  // bms_info_voltage
  // bms_info_current
  // bms_info_remaining_capacity
  return true;
}

void
yhs_can_interfaces__msg__BmsInfoFb__fini(yhs_can_interfaces__msg__BmsInfoFb * msg)
{
  if (!msg) {
    return;
  }
  // bms_info_voltage
  // bms_info_current
  // bms_info_remaining_capacity
}

bool
yhs_can_interfaces__msg__BmsInfoFb__are_equal(const yhs_can_interfaces__msg__BmsInfoFb * lhs, const yhs_can_interfaces__msg__BmsInfoFb * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // bms_info_voltage
  if (lhs->bms_info_voltage != rhs->bms_info_voltage) {
    return false;
  }
  // bms_info_current
  if (lhs->bms_info_current != rhs->bms_info_current) {
    return false;
  }
  // bms_info_remaining_capacity
  if (lhs->bms_info_remaining_capacity != rhs->bms_info_remaining_capacity) {
    return false;
  }
  return true;
}

bool
yhs_can_interfaces__msg__BmsInfoFb__copy(
  const yhs_can_interfaces__msg__BmsInfoFb * input,
  yhs_can_interfaces__msg__BmsInfoFb * output)
{
  if (!input || !output) {
    return false;
  }
  // bms_info_voltage
  output->bms_info_voltage = input->bms_info_voltage;
  // bms_info_current
  output->bms_info_current = input->bms_info_current;
  // bms_info_remaining_capacity
  output->bms_info_remaining_capacity = input->bms_info_remaining_capacity;
  return true;
}

yhs_can_interfaces__msg__BmsInfoFb *
yhs_can_interfaces__msg__BmsInfoFb__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__BmsInfoFb * msg = (yhs_can_interfaces__msg__BmsInfoFb *)allocator.allocate(sizeof(yhs_can_interfaces__msg__BmsInfoFb), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yhs_can_interfaces__msg__BmsInfoFb));
  bool success = yhs_can_interfaces__msg__BmsInfoFb__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yhs_can_interfaces__msg__BmsInfoFb__destroy(yhs_can_interfaces__msg__BmsInfoFb * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yhs_can_interfaces__msg__BmsInfoFb__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yhs_can_interfaces__msg__BmsInfoFb__Sequence__init(yhs_can_interfaces__msg__BmsInfoFb__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__BmsInfoFb * data = NULL;

  if (size) {
    data = (yhs_can_interfaces__msg__BmsInfoFb *)allocator.zero_allocate(size, sizeof(yhs_can_interfaces__msg__BmsInfoFb), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yhs_can_interfaces__msg__BmsInfoFb__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yhs_can_interfaces__msg__BmsInfoFb__fini(&data[i - 1]);
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
yhs_can_interfaces__msg__BmsInfoFb__Sequence__fini(yhs_can_interfaces__msg__BmsInfoFb__Sequence * array)
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
      yhs_can_interfaces__msg__BmsInfoFb__fini(&array->data[i]);
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

yhs_can_interfaces__msg__BmsInfoFb__Sequence *
yhs_can_interfaces__msg__BmsInfoFb__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__BmsInfoFb__Sequence * array = (yhs_can_interfaces__msg__BmsInfoFb__Sequence *)allocator.allocate(sizeof(yhs_can_interfaces__msg__BmsInfoFb__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yhs_can_interfaces__msg__BmsInfoFb__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yhs_can_interfaces__msg__BmsInfoFb__Sequence__destroy(yhs_can_interfaces__msg__BmsInfoFb__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yhs_can_interfaces__msg__BmsInfoFb__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yhs_can_interfaces__msg__BmsInfoFb__Sequence__are_equal(const yhs_can_interfaces__msg__BmsInfoFb__Sequence * lhs, const yhs_can_interfaces__msg__BmsInfoFb__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yhs_can_interfaces__msg__BmsInfoFb__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yhs_can_interfaces__msg__BmsInfoFb__Sequence__copy(
  const yhs_can_interfaces__msg__BmsInfoFb__Sequence * input,
  yhs_can_interfaces__msg__BmsInfoFb__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yhs_can_interfaces__msg__BmsInfoFb);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yhs_can_interfaces__msg__BmsInfoFb * data =
      (yhs_can_interfaces__msg__BmsInfoFb *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yhs_can_interfaces__msg__BmsInfoFb__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yhs_can_interfaces__msg__BmsInfoFb__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yhs_can_interfaces__msg__BmsInfoFb__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
