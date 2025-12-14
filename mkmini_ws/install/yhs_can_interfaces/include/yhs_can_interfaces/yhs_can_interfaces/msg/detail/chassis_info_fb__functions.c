// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yhs_can_interfaces:msg/ChassisInfoFb.idl
// generated code does not contain a copyright notice
#include "yhs_can_interfaces/msg/detail/chassis_info_fb__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `ctrl_fb`
#include "yhs_can_interfaces/msg/detail/ctrl_fb__functions.h"
// Member `io_fb`
#include "yhs_can_interfaces/msg/detail/io_fb__functions.h"
// Member `lr_wheel_fb`
#include "yhs_can_interfaces/msg/detail/lr_wheel_fb__functions.h"
// Member `rr_wheel_fb`
#include "yhs_can_interfaces/msg/detail/rr_wheel_fb__functions.h"
// Member `bms_flag_info_fb`
#include "yhs_can_interfaces/msg/detail/bms_flag_info_fb__functions.h"
// Member `bms_info_fb`
#include "yhs_can_interfaces/msg/detail/bms_info_fb__functions.h"
// Member `drive_mcu_ecode_fb`
#include "yhs_can_interfaces/msg/detail/drive_mcu_ecode_fb__functions.h"
// Member `veh_diag_fb`
#include "yhs_can_interfaces/msg/detail/veh_diag_fb__functions.h"

bool
yhs_can_interfaces__msg__ChassisInfoFb__init(yhs_can_interfaces__msg__ChassisInfoFb * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    yhs_can_interfaces__msg__ChassisInfoFb__fini(msg);
    return false;
  }
  // ctrl_fb
  if (!yhs_can_interfaces__msg__CtrlFb__init(&msg->ctrl_fb)) {
    yhs_can_interfaces__msg__ChassisInfoFb__fini(msg);
    return false;
  }
  // io_fb
  if (!yhs_can_interfaces__msg__IoFb__init(&msg->io_fb)) {
    yhs_can_interfaces__msg__ChassisInfoFb__fini(msg);
    return false;
  }
  // lr_wheel_fb
  if (!yhs_can_interfaces__msg__LrWheelFb__init(&msg->lr_wheel_fb)) {
    yhs_can_interfaces__msg__ChassisInfoFb__fini(msg);
    return false;
  }
  // rr_wheel_fb
  if (!yhs_can_interfaces__msg__RrWheelFb__init(&msg->rr_wheel_fb)) {
    yhs_can_interfaces__msg__ChassisInfoFb__fini(msg);
    return false;
  }
  // bms_flag_info_fb
  if (!yhs_can_interfaces__msg__BmsFlagInfoFb__init(&msg->bms_flag_info_fb)) {
    yhs_can_interfaces__msg__ChassisInfoFb__fini(msg);
    return false;
  }
  // bms_info_fb
  if (!yhs_can_interfaces__msg__BmsInfoFb__init(&msg->bms_info_fb)) {
    yhs_can_interfaces__msg__ChassisInfoFb__fini(msg);
    return false;
  }
  // drive_mcu_ecode_fb
  if (!yhs_can_interfaces__msg__DriveMcuEcodeFb__init(&msg->drive_mcu_ecode_fb)) {
    yhs_can_interfaces__msg__ChassisInfoFb__fini(msg);
    return false;
  }
  // veh_diag_fb
  if (!yhs_can_interfaces__msg__VehDiagFb__init(&msg->veh_diag_fb)) {
    yhs_can_interfaces__msg__ChassisInfoFb__fini(msg);
    return false;
  }
  return true;
}

void
yhs_can_interfaces__msg__ChassisInfoFb__fini(yhs_can_interfaces__msg__ChassisInfoFb * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // ctrl_fb
  yhs_can_interfaces__msg__CtrlFb__fini(&msg->ctrl_fb);
  // io_fb
  yhs_can_interfaces__msg__IoFb__fini(&msg->io_fb);
  // lr_wheel_fb
  yhs_can_interfaces__msg__LrWheelFb__fini(&msg->lr_wheel_fb);
  // rr_wheel_fb
  yhs_can_interfaces__msg__RrWheelFb__fini(&msg->rr_wheel_fb);
  // bms_flag_info_fb
  yhs_can_interfaces__msg__BmsFlagInfoFb__fini(&msg->bms_flag_info_fb);
  // bms_info_fb
  yhs_can_interfaces__msg__BmsInfoFb__fini(&msg->bms_info_fb);
  // drive_mcu_ecode_fb
  yhs_can_interfaces__msg__DriveMcuEcodeFb__fini(&msg->drive_mcu_ecode_fb);
  // veh_diag_fb
  yhs_can_interfaces__msg__VehDiagFb__fini(&msg->veh_diag_fb);
}

bool
yhs_can_interfaces__msg__ChassisInfoFb__are_equal(const yhs_can_interfaces__msg__ChassisInfoFb * lhs, const yhs_can_interfaces__msg__ChassisInfoFb * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // ctrl_fb
  if (!yhs_can_interfaces__msg__CtrlFb__are_equal(
      &(lhs->ctrl_fb), &(rhs->ctrl_fb)))
  {
    return false;
  }
  // io_fb
  if (!yhs_can_interfaces__msg__IoFb__are_equal(
      &(lhs->io_fb), &(rhs->io_fb)))
  {
    return false;
  }
  // lr_wheel_fb
  if (!yhs_can_interfaces__msg__LrWheelFb__are_equal(
      &(lhs->lr_wheel_fb), &(rhs->lr_wheel_fb)))
  {
    return false;
  }
  // rr_wheel_fb
  if (!yhs_can_interfaces__msg__RrWheelFb__are_equal(
      &(lhs->rr_wheel_fb), &(rhs->rr_wheel_fb)))
  {
    return false;
  }
  // bms_flag_info_fb
  if (!yhs_can_interfaces__msg__BmsFlagInfoFb__are_equal(
      &(lhs->bms_flag_info_fb), &(rhs->bms_flag_info_fb)))
  {
    return false;
  }
  // bms_info_fb
  if (!yhs_can_interfaces__msg__BmsInfoFb__are_equal(
      &(lhs->bms_info_fb), &(rhs->bms_info_fb)))
  {
    return false;
  }
  // drive_mcu_ecode_fb
  if (!yhs_can_interfaces__msg__DriveMcuEcodeFb__are_equal(
      &(lhs->drive_mcu_ecode_fb), &(rhs->drive_mcu_ecode_fb)))
  {
    return false;
  }
  // veh_diag_fb
  if (!yhs_can_interfaces__msg__VehDiagFb__are_equal(
      &(lhs->veh_diag_fb), &(rhs->veh_diag_fb)))
  {
    return false;
  }
  return true;
}

bool
yhs_can_interfaces__msg__ChassisInfoFb__copy(
  const yhs_can_interfaces__msg__ChassisInfoFb * input,
  yhs_can_interfaces__msg__ChassisInfoFb * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // ctrl_fb
  if (!yhs_can_interfaces__msg__CtrlFb__copy(
      &(input->ctrl_fb), &(output->ctrl_fb)))
  {
    return false;
  }
  // io_fb
  if (!yhs_can_interfaces__msg__IoFb__copy(
      &(input->io_fb), &(output->io_fb)))
  {
    return false;
  }
  // lr_wheel_fb
  if (!yhs_can_interfaces__msg__LrWheelFb__copy(
      &(input->lr_wheel_fb), &(output->lr_wheel_fb)))
  {
    return false;
  }
  // rr_wheel_fb
  if (!yhs_can_interfaces__msg__RrWheelFb__copy(
      &(input->rr_wheel_fb), &(output->rr_wheel_fb)))
  {
    return false;
  }
  // bms_flag_info_fb
  if (!yhs_can_interfaces__msg__BmsFlagInfoFb__copy(
      &(input->bms_flag_info_fb), &(output->bms_flag_info_fb)))
  {
    return false;
  }
  // bms_info_fb
  if (!yhs_can_interfaces__msg__BmsInfoFb__copy(
      &(input->bms_info_fb), &(output->bms_info_fb)))
  {
    return false;
  }
  // drive_mcu_ecode_fb
  if (!yhs_can_interfaces__msg__DriveMcuEcodeFb__copy(
      &(input->drive_mcu_ecode_fb), &(output->drive_mcu_ecode_fb)))
  {
    return false;
  }
  // veh_diag_fb
  if (!yhs_can_interfaces__msg__VehDiagFb__copy(
      &(input->veh_diag_fb), &(output->veh_diag_fb)))
  {
    return false;
  }
  return true;
}

yhs_can_interfaces__msg__ChassisInfoFb *
yhs_can_interfaces__msg__ChassisInfoFb__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__ChassisInfoFb * msg = (yhs_can_interfaces__msg__ChassisInfoFb *)allocator.allocate(sizeof(yhs_can_interfaces__msg__ChassisInfoFb), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yhs_can_interfaces__msg__ChassisInfoFb));
  bool success = yhs_can_interfaces__msg__ChassisInfoFb__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yhs_can_interfaces__msg__ChassisInfoFb__destroy(yhs_can_interfaces__msg__ChassisInfoFb * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yhs_can_interfaces__msg__ChassisInfoFb__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yhs_can_interfaces__msg__ChassisInfoFb__Sequence__init(yhs_can_interfaces__msg__ChassisInfoFb__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__ChassisInfoFb * data = NULL;

  if (size) {
    data = (yhs_can_interfaces__msg__ChassisInfoFb *)allocator.zero_allocate(size, sizeof(yhs_can_interfaces__msg__ChassisInfoFb), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yhs_can_interfaces__msg__ChassisInfoFb__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yhs_can_interfaces__msg__ChassisInfoFb__fini(&data[i - 1]);
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
yhs_can_interfaces__msg__ChassisInfoFb__Sequence__fini(yhs_can_interfaces__msg__ChassisInfoFb__Sequence * array)
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
      yhs_can_interfaces__msg__ChassisInfoFb__fini(&array->data[i]);
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

yhs_can_interfaces__msg__ChassisInfoFb__Sequence *
yhs_can_interfaces__msg__ChassisInfoFb__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__ChassisInfoFb__Sequence * array = (yhs_can_interfaces__msg__ChassisInfoFb__Sequence *)allocator.allocate(sizeof(yhs_can_interfaces__msg__ChassisInfoFb__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yhs_can_interfaces__msg__ChassisInfoFb__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yhs_can_interfaces__msg__ChassisInfoFb__Sequence__destroy(yhs_can_interfaces__msg__ChassisInfoFb__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yhs_can_interfaces__msg__ChassisInfoFb__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yhs_can_interfaces__msg__ChassisInfoFb__Sequence__are_equal(const yhs_can_interfaces__msg__ChassisInfoFb__Sequence * lhs, const yhs_can_interfaces__msg__ChassisInfoFb__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yhs_can_interfaces__msg__ChassisInfoFb__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yhs_can_interfaces__msg__ChassisInfoFb__Sequence__copy(
  const yhs_can_interfaces__msg__ChassisInfoFb__Sequence * input,
  yhs_can_interfaces__msg__ChassisInfoFb__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yhs_can_interfaces__msg__ChassisInfoFb);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yhs_can_interfaces__msg__ChassisInfoFb * data =
      (yhs_can_interfaces__msg__ChassisInfoFb *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yhs_can_interfaces__msg__ChassisInfoFb__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yhs_can_interfaces__msg__ChassisInfoFb__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yhs_can_interfaces__msg__ChassisInfoFb__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
