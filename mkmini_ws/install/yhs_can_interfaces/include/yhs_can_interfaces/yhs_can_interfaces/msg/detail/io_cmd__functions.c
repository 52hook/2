// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yhs_can_interfaces:msg/IoCmd.idl
// generated code does not contain a copyright notice
#include "yhs_can_interfaces/msg/detail/io_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
yhs_can_interfaces__msg__IoCmd__init(yhs_can_interfaces__msg__IoCmd * msg)
{
  if (!msg) {
    return false;
  }
  // io_cmd_enable
  // io_cmd_lower_beam_headlamp
  // io_cmd_upper_beam_headlamp
  // io_cmd_turn_lamp
  // io_cmd_braking_lamp
  // io_cmd_clearance_lamp
  // io_cmd_fog_lamp
  // io_cmd_speaker
  // io_cmd_dis_charge
  return true;
}

void
yhs_can_interfaces__msg__IoCmd__fini(yhs_can_interfaces__msg__IoCmd * msg)
{
  if (!msg) {
    return;
  }
  // io_cmd_enable
  // io_cmd_lower_beam_headlamp
  // io_cmd_upper_beam_headlamp
  // io_cmd_turn_lamp
  // io_cmd_braking_lamp
  // io_cmd_clearance_lamp
  // io_cmd_fog_lamp
  // io_cmd_speaker
  // io_cmd_dis_charge
}

bool
yhs_can_interfaces__msg__IoCmd__are_equal(const yhs_can_interfaces__msg__IoCmd * lhs, const yhs_can_interfaces__msg__IoCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // io_cmd_enable
  if (lhs->io_cmd_enable != rhs->io_cmd_enable) {
    return false;
  }
  // io_cmd_lower_beam_headlamp
  if (lhs->io_cmd_lower_beam_headlamp != rhs->io_cmd_lower_beam_headlamp) {
    return false;
  }
  // io_cmd_upper_beam_headlamp
  if (lhs->io_cmd_upper_beam_headlamp != rhs->io_cmd_upper_beam_headlamp) {
    return false;
  }
  // io_cmd_turn_lamp
  if (lhs->io_cmd_turn_lamp != rhs->io_cmd_turn_lamp) {
    return false;
  }
  // io_cmd_braking_lamp
  if (lhs->io_cmd_braking_lamp != rhs->io_cmd_braking_lamp) {
    return false;
  }
  // io_cmd_clearance_lamp
  if (lhs->io_cmd_clearance_lamp != rhs->io_cmd_clearance_lamp) {
    return false;
  }
  // io_cmd_fog_lamp
  if (lhs->io_cmd_fog_lamp != rhs->io_cmd_fog_lamp) {
    return false;
  }
  // io_cmd_speaker
  if (lhs->io_cmd_speaker != rhs->io_cmd_speaker) {
    return false;
  }
  // io_cmd_dis_charge
  if (lhs->io_cmd_dis_charge != rhs->io_cmd_dis_charge) {
    return false;
  }
  return true;
}

bool
yhs_can_interfaces__msg__IoCmd__copy(
  const yhs_can_interfaces__msg__IoCmd * input,
  yhs_can_interfaces__msg__IoCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // io_cmd_enable
  output->io_cmd_enable = input->io_cmd_enable;
  // io_cmd_lower_beam_headlamp
  output->io_cmd_lower_beam_headlamp = input->io_cmd_lower_beam_headlamp;
  // io_cmd_upper_beam_headlamp
  output->io_cmd_upper_beam_headlamp = input->io_cmd_upper_beam_headlamp;
  // io_cmd_turn_lamp
  output->io_cmd_turn_lamp = input->io_cmd_turn_lamp;
  // io_cmd_braking_lamp
  output->io_cmd_braking_lamp = input->io_cmd_braking_lamp;
  // io_cmd_clearance_lamp
  output->io_cmd_clearance_lamp = input->io_cmd_clearance_lamp;
  // io_cmd_fog_lamp
  output->io_cmd_fog_lamp = input->io_cmd_fog_lamp;
  // io_cmd_speaker
  output->io_cmd_speaker = input->io_cmd_speaker;
  // io_cmd_dis_charge
  output->io_cmd_dis_charge = input->io_cmd_dis_charge;
  return true;
}

yhs_can_interfaces__msg__IoCmd *
yhs_can_interfaces__msg__IoCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__IoCmd * msg = (yhs_can_interfaces__msg__IoCmd *)allocator.allocate(sizeof(yhs_can_interfaces__msg__IoCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yhs_can_interfaces__msg__IoCmd));
  bool success = yhs_can_interfaces__msg__IoCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yhs_can_interfaces__msg__IoCmd__destroy(yhs_can_interfaces__msg__IoCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yhs_can_interfaces__msg__IoCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yhs_can_interfaces__msg__IoCmd__Sequence__init(yhs_can_interfaces__msg__IoCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__IoCmd * data = NULL;

  if (size) {
    data = (yhs_can_interfaces__msg__IoCmd *)allocator.zero_allocate(size, sizeof(yhs_can_interfaces__msg__IoCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yhs_can_interfaces__msg__IoCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yhs_can_interfaces__msg__IoCmd__fini(&data[i - 1]);
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
yhs_can_interfaces__msg__IoCmd__Sequence__fini(yhs_can_interfaces__msg__IoCmd__Sequence * array)
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
      yhs_can_interfaces__msg__IoCmd__fini(&array->data[i]);
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

yhs_can_interfaces__msg__IoCmd__Sequence *
yhs_can_interfaces__msg__IoCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__IoCmd__Sequence * array = (yhs_can_interfaces__msg__IoCmd__Sequence *)allocator.allocate(sizeof(yhs_can_interfaces__msg__IoCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yhs_can_interfaces__msg__IoCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yhs_can_interfaces__msg__IoCmd__Sequence__destroy(yhs_can_interfaces__msg__IoCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yhs_can_interfaces__msg__IoCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yhs_can_interfaces__msg__IoCmd__Sequence__are_equal(const yhs_can_interfaces__msg__IoCmd__Sequence * lhs, const yhs_can_interfaces__msg__IoCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yhs_can_interfaces__msg__IoCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yhs_can_interfaces__msg__IoCmd__Sequence__copy(
  const yhs_can_interfaces__msg__IoCmd__Sequence * input,
  yhs_can_interfaces__msg__IoCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yhs_can_interfaces__msg__IoCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yhs_can_interfaces__msg__IoCmd * data =
      (yhs_can_interfaces__msg__IoCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yhs_can_interfaces__msg__IoCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yhs_can_interfaces__msg__IoCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yhs_can_interfaces__msg__IoCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
