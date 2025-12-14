// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yhs_can_interfaces:msg/CtrlCmd.idl
// generated code does not contain a copyright notice
#include "yhs_can_interfaces/msg/detail/ctrl_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
yhs_can_interfaces__msg__CtrlCmd__init(yhs_can_interfaces__msg__CtrlCmd * msg)
{
  if (!msg) {
    return false;
  }
  // ctrl_cmd_gear
  // ctrl_cmd_velocity
  // ctrl_cmd_steering
  return true;
}

void
yhs_can_interfaces__msg__CtrlCmd__fini(yhs_can_interfaces__msg__CtrlCmd * msg)
{
  if (!msg) {
    return;
  }
  // ctrl_cmd_gear
  // ctrl_cmd_velocity
  // ctrl_cmd_steering
}

bool
yhs_can_interfaces__msg__CtrlCmd__are_equal(const yhs_can_interfaces__msg__CtrlCmd * lhs, const yhs_can_interfaces__msg__CtrlCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ctrl_cmd_gear
  if (lhs->ctrl_cmd_gear != rhs->ctrl_cmd_gear) {
    return false;
  }
  // ctrl_cmd_velocity
  if (lhs->ctrl_cmd_velocity != rhs->ctrl_cmd_velocity) {
    return false;
  }
  // ctrl_cmd_steering
  if (lhs->ctrl_cmd_steering != rhs->ctrl_cmd_steering) {
    return false;
  }
  return true;
}

bool
yhs_can_interfaces__msg__CtrlCmd__copy(
  const yhs_can_interfaces__msg__CtrlCmd * input,
  yhs_can_interfaces__msg__CtrlCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // ctrl_cmd_gear
  output->ctrl_cmd_gear = input->ctrl_cmd_gear;
  // ctrl_cmd_velocity
  output->ctrl_cmd_velocity = input->ctrl_cmd_velocity;
  // ctrl_cmd_steering
  output->ctrl_cmd_steering = input->ctrl_cmd_steering;
  return true;
}

yhs_can_interfaces__msg__CtrlCmd *
yhs_can_interfaces__msg__CtrlCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__CtrlCmd * msg = (yhs_can_interfaces__msg__CtrlCmd *)allocator.allocate(sizeof(yhs_can_interfaces__msg__CtrlCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yhs_can_interfaces__msg__CtrlCmd));
  bool success = yhs_can_interfaces__msg__CtrlCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yhs_can_interfaces__msg__CtrlCmd__destroy(yhs_can_interfaces__msg__CtrlCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yhs_can_interfaces__msg__CtrlCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yhs_can_interfaces__msg__CtrlCmd__Sequence__init(yhs_can_interfaces__msg__CtrlCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__CtrlCmd * data = NULL;

  if (size) {
    data = (yhs_can_interfaces__msg__CtrlCmd *)allocator.zero_allocate(size, sizeof(yhs_can_interfaces__msg__CtrlCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yhs_can_interfaces__msg__CtrlCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yhs_can_interfaces__msg__CtrlCmd__fini(&data[i - 1]);
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
yhs_can_interfaces__msg__CtrlCmd__Sequence__fini(yhs_can_interfaces__msg__CtrlCmd__Sequence * array)
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
      yhs_can_interfaces__msg__CtrlCmd__fini(&array->data[i]);
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

yhs_can_interfaces__msg__CtrlCmd__Sequence *
yhs_can_interfaces__msg__CtrlCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__CtrlCmd__Sequence * array = (yhs_can_interfaces__msg__CtrlCmd__Sequence *)allocator.allocate(sizeof(yhs_can_interfaces__msg__CtrlCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yhs_can_interfaces__msg__CtrlCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yhs_can_interfaces__msg__CtrlCmd__Sequence__destroy(yhs_can_interfaces__msg__CtrlCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yhs_can_interfaces__msg__CtrlCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yhs_can_interfaces__msg__CtrlCmd__Sequence__are_equal(const yhs_can_interfaces__msg__CtrlCmd__Sequence * lhs, const yhs_can_interfaces__msg__CtrlCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yhs_can_interfaces__msg__CtrlCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yhs_can_interfaces__msg__CtrlCmd__Sequence__copy(
  const yhs_can_interfaces__msg__CtrlCmd__Sequence * input,
  yhs_can_interfaces__msg__CtrlCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yhs_can_interfaces__msg__CtrlCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yhs_can_interfaces__msg__CtrlCmd * data =
      (yhs_can_interfaces__msg__CtrlCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yhs_can_interfaces__msg__CtrlCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yhs_can_interfaces__msg__CtrlCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yhs_can_interfaces__msg__CtrlCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
