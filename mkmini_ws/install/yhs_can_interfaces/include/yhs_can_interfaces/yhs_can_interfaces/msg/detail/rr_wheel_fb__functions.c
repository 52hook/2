// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yhs_can_interfaces:msg/RrWheelFb.idl
// generated code does not contain a copyright notice
#include "yhs_can_interfaces/msg/detail/rr_wheel_fb__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
yhs_can_interfaces__msg__RrWheelFb__init(yhs_can_interfaces__msg__RrWheelFb * msg)
{
  if (!msg) {
    return false;
  }
  // rr_wheel_fb_velocity
  // rr_wheel_fb_pulse
  return true;
}

void
yhs_can_interfaces__msg__RrWheelFb__fini(yhs_can_interfaces__msg__RrWheelFb * msg)
{
  if (!msg) {
    return;
  }
  // rr_wheel_fb_velocity
  // rr_wheel_fb_pulse
}

bool
yhs_can_interfaces__msg__RrWheelFb__are_equal(const yhs_can_interfaces__msg__RrWheelFb * lhs, const yhs_can_interfaces__msg__RrWheelFb * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // rr_wheel_fb_velocity
  if (lhs->rr_wheel_fb_velocity != rhs->rr_wheel_fb_velocity) {
    return false;
  }
  // rr_wheel_fb_pulse
  if (lhs->rr_wheel_fb_pulse != rhs->rr_wheel_fb_pulse) {
    return false;
  }
  return true;
}

bool
yhs_can_interfaces__msg__RrWheelFb__copy(
  const yhs_can_interfaces__msg__RrWheelFb * input,
  yhs_can_interfaces__msg__RrWheelFb * output)
{
  if (!input || !output) {
    return false;
  }
  // rr_wheel_fb_velocity
  output->rr_wheel_fb_velocity = input->rr_wheel_fb_velocity;
  // rr_wheel_fb_pulse
  output->rr_wheel_fb_pulse = input->rr_wheel_fb_pulse;
  return true;
}

yhs_can_interfaces__msg__RrWheelFb *
yhs_can_interfaces__msg__RrWheelFb__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__RrWheelFb * msg = (yhs_can_interfaces__msg__RrWheelFb *)allocator.allocate(sizeof(yhs_can_interfaces__msg__RrWheelFb), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yhs_can_interfaces__msg__RrWheelFb));
  bool success = yhs_can_interfaces__msg__RrWheelFb__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yhs_can_interfaces__msg__RrWheelFb__destroy(yhs_can_interfaces__msg__RrWheelFb * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yhs_can_interfaces__msg__RrWheelFb__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yhs_can_interfaces__msg__RrWheelFb__Sequence__init(yhs_can_interfaces__msg__RrWheelFb__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__RrWheelFb * data = NULL;

  if (size) {
    data = (yhs_can_interfaces__msg__RrWheelFb *)allocator.zero_allocate(size, sizeof(yhs_can_interfaces__msg__RrWheelFb), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yhs_can_interfaces__msg__RrWheelFb__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yhs_can_interfaces__msg__RrWheelFb__fini(&data[i - 1]);
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
yhs_can_interfaces__msg__RrWheelFb__Sequence__fini(yhs_can_interfaces__msg__RrWheelFb__Sequence * array)
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
      yhs_can_interfaces__msg__RrWheelFb__fini(&array->data[i]);
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

yhs_can_interfaces__msg__RrWheelFb__Sequence *
yhs_can_interfaces__msg__RrWheelFb__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__RrWheelFb__Sequence * array = (yhs_can_interfaces__msg__RrWheelFb__Sequence *)allocator.allocate(sizeof(yhs_can_interfaces__msg__RrWheelFb__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yhs_can_interfaces__msg__RrWheelFb__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yhs_can_interfaces__msg__RrWheelFb__Sequence__destroy(yhs_can_interfaces__msg__RrWheelFb__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yhs_can_interfaces__msg__RrWheelFb__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yhs_can_interfaces__msg__RrWheelFb__Sequence__are_equal(const yhs_can_interfaces__msg__RrWheelFb__Sequence * lhs, const yhs_can_interfaces__msg__RrWheelFb__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yhs_can_interfaces__msg__RrWheelFb__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yhs_can_interfaces__msg__RrWheelFb__Sequence__copy(
  const yhs_can_interfaces__msg__RrWheelFb__Sequence * input,
  yhs_can_interfaces__msg__RrWheelFb__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yhs_can_interfaces__msg__RrWheelFb);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yhs_can_interfaces__msg__RrWheelFb * data =
      (yhs_can_interfaces__msg__RrWheelFb *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yhs_can_interfaces__msg__RrWheelFb__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yhs_can_interfaces__msg__RrWheelFb__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yhs_can_interfaces__msg__RrWheelFb__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
