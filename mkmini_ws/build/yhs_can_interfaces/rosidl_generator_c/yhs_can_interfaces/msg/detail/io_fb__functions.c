// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yhs_can_interfaces:msg/IoFb.idl
// generated code does not contain a copyright notice
#include "yhs_can_interfaces/msg/detail/io_fb__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
yhs_can_interfaces__msg__IoFb__init(yhs_can_interfaces__msg__IoFb * msg)
{
  if (!msg) {
    return false;
  }
  // io_fb_enable
  // io_fb_turn_lamp
  // io_fb_braking_lamp
  // io_fb_fm_impact_sensor
  // io_fb_rm_impact_sensor
  // io_fb_dis_charge
  // io_fb_charge_en
  // io_fb_scram_st
  return true;
}

void
yhs_can_interfaces__msg__IoFb__fini(yhs_can_interfaces__msg__IoFb * msg)
{
  if (!msg) {
    return;
  }
  // io_fb_enable
  // io_fb_turn_lamp
  // io_fb_braking_lamp
  // io_fb_fm_impact_sensor
  // io_fb_rm_impact_sensor
  // io_fb_dis_charge
  // io_fb_charge_en
  // io_fb_scram_st
}

bool
yhs_can_interfaces__msg__IoFb__are_equal(const yhs_can_interfaces__msg__IoFb * lhs, const yhs_can_interfaces__msg__IoFb * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // io_fb_enable
  if (lhs->io_fb_enable != rhs->io_fb_enable) {
    return false;
  }
  // io_fb_turn_lamp
  if (lhs->io_fb_turn_lamp != rhs->io_fb_turn_lamp) {
    return false;
  }
  // io_fb_braking_lamp
  if (lhs->io_fb_braking_lamp != rhs->io_fb_braking_lamp) {
    return false;
  }
  // io_fb_fm_impact_sensor
  if (lhs->io_fb_fm_impact_sensor != rhs->io_fb_fm_impact_sensor) {
    return false;
  }
  // io_fb_rm_impact_sensor
  if (lhs->io_fb_rm_impact_sensor != rhs->io_fb_rm_impact_sensor) {
    return false;
  }
  // io_fb_dis_charge
  if (lhs->io_fb_dis_charge != rhs->io_fb_dis_charge) {
    return false;
  }
  // io_fb_charge_en
  if (lhs->io_fb_charge_en != rhs->io_fb_charge_en) {
    return false;
  }
  // io_fb_scram_st
  if (lhs->io_fb_scram_st != rhs->io_fb_scram_st) {
    return false;
  }
  return true;
}

bool
yhs_can_interfaces__msg__IoFb__copy(
  const yhs_can_interfaces__msg__IoFb * input,
  yhs_can_interfaces__msg__IoFb * output)
{
  if (!input || !output) {
    return false;
  }
  // io_fb_enable
  output->io_fb_enable = input->io_fb_enable;
  // io_fb_turn_lamp
  output->io_fb_turn_lamp = input->io_fb_turn_lamp;
  // io_fb_braking_lamp
  output->io_fb_braking_lamp = input->io_fb_braking_lamp;
  // io_fb_fm_impact_sensor
  output->io_fb_fm_impact_sensor = input->io_fb_fm_impact_sensor;
  // io_fb_rm_impact_sensor
  output->io_fb_rm_impact_sensor = input->io_fb_rm_impact_sensor;
  // io_fb_dis_charge
  output->io_fb_dis_charge = input->io_fb_dis_charge;
  // io_fb_charge_en
  output->io_fb_charge_en = input->io_fb_charge_en;
  // io_fb_scram_st
  output->io_fb_scram_st = input->io_fb_scram_st;
  return true;
}

yhs_can_interfaces__msg__IoFb *
yhs_can_interfaces__msg__IoFb__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__IoFb * msg = (yhs_can_interfaces__msg__IoFb *)allocator.allocate(sizeof(yhs_can_interfaces__msg__IoFb), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yhs_can_interfaces__msg__IoFb));
  bool success = yhs_can_interfaces__msg__IoFb__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yhs_can_interfaces__msg__IoFb__destroy(yhs_can_interfaces__msg__IoFb * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yhs_can_interfaces__msg__IoFb__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yhs_can_interfaces__msg__IoFb__Sequence__init(yhs_can_interfaces__msg__IoFb__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__IoFb * data = NULL;

  if (size) {
    data = (yhs_can_interfaces__msg__IoFb *)allocator.zero_allocate(size, sizeof(yhs_can_interfaces__msg__IoFb), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yhs_can_interfaces__msg__IoFb__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yhs_can_interfaces__msg__IoFb__fini(&data[i - 1]);
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
yhs_can_interfaces__msg__IoFb__Sequence__fini(yhs_can_interfaces__msg__IoFb__Sequence * array)
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
      yhs_can_interfaces__msg__IoFb__fini(&array->data[i]);
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

yhs_can_interfaces__msg__IoFb__Sequence *
yhs_can_interfaces__msg__IoFb__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__IoFb__Sequence * array = (yhs_can_interfaces__msg__IoFb__Sequence *)allocator.allocate(sizeof(yhs_can_interfaces__msg__IoFb__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yhs_can_interfaces__msg__IoFb__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yhs_can_interfaces__msg__IoFb__Sequence__destroy(yhs_can_interfaces__msg__IoFb__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yhs_can_interfaces__msg__IoFb__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yhs_can_interfaces__msg__IoFb__Sequence__are_equal(const yhs_can_interfaces__msg__IoFb__Sequence * lhs, const yhs_can_interfaces__msg__IoFb__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yhs_can_interfaces__msg__IoFb__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yhs_can_interfaces__msg__IoFb__Sequence__copy(
  const yhs_can_interfaces__msg__IoFb__Sequence * input,
  yhs_can_interfaces__msg__IoFb__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yhs_can_interfaces__msg__IoFb);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yhs_can_interfaces__msg__IoFb * data =
      (yhs_can_interfaces__msg__IoFb *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yhs_can_interfaces__msg__IoFb__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yhs_can_interfaces__msg__IoFb__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yhs_can_interfaces__msg__IoFb__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
