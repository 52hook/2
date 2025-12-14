// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yhs_can_interfaces:msg/VehDiagFb.idl
// generated code does not contain a copyright notice
#include "yhs_can_interfaces/msg/detail/veh_diag_fb__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
yhs_can_interfaces__msg__VehDiagFb__init(yhs_can_interfaces__msg__VehDiagFb * msg)
{
  if (!msg) {
    return false;
  }
  // veh_fb_fault_level
  // veh_fb_auto_can_ctrl_cmd
  // veh_fb_auto_io_can_cmd
  // veh_fb_eps_dis_on_line
  // veh_fb_eps_fault
  // veh_fb_eps_mosf_et_ot
  // veh_fb_eps_warning
  // veh_fb_eps_dis_work
  // veh_fb_eps_over_current
  // veh_fb_st_reserve
  // veh_fb_ehb_ecu_fault
  // veh_fb_ehb_dis_on_line
  // veh_fb_ehb_work_model_fault
  // veh_fb_ehb_dis_en
  // veh_fb_ehb_anguler_fault
  // veh_fb_ehb_ot
  // veh_fb_ehb_power_fault
  // veh_fb_ehb_sensor_abnomal
  // veh_fb_ehb_motor_fault
  // veh_fb_ehb_oil_press_sensor_fault
  // veh_fb_ehb_oil_fault
  // veh_fb_ld_rv_mcu_fault
  // veh_fb_rd_rv_mcu_fault
  // veh_fb_aux_bms_dis_on_line
  // veh_fb_aux_scram
  // veh_fb_aux_remote_close
  // veh_fb_aux_remote_dis_on_line
  return true;
}

void
yhs_can_interfaces__msg__VehDiagFb__fini(yhs_can_interfaces__msg__VehDiagFb * msg)
{
  if (!msg) {
    return;
  }
  // veh_fb_fault_level
  // veh_fb_auto_can_ctrl_cmd
  // veh_fb_auto_io_can_cmd
  // veh_fb_eps_dis_on_line
  // veh_fb_eps_fault
  // veh_fb_eps_mosf_et_ot
  // veh_fb_eps_warning
  // veh_fb_eps_dis_work
  // veh_fb_eps_over_current
  // veh_fb_st_reserve
  // veh_fb_ehb_ecu_fault
  // veh_fb_ehb_dis_on_line
  // veh_fb_ehb_work_model_fault
  // veh_fb_ehb_dis_en
  // veh_fb_ehb_anguler_fault
  // veh_fb_ehb_ot
  // veh_fb_ehb_power_fault
  // veh_fb_ehb_sensor_abnomal
  // veh_fb_ehb_motor_fault
  // veh_fb_ehb_oil_press_sensor_fault
  // veh_fb_ehb_oil_fault
  // veh_fb_ld_rv_mcu_fault
  // veh_fb_rd_rv_mcu_fault
  // veh_fb_aux_bms_dis_on_line
  // veh_fb_aux_scram
  // veh_fb_aux_remote_close
  // veh_fb_aux_remote_dis_on_line
}

bool
yhs_can_interfaces__msg__VehDiagFb__are_equal(const yhs_can_interfaces__msg__VehDiagFb * lhs, const yhs_can_interfaces__msg__VehDiagFb * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // veh_fb_fault_level
  if (lhs->veh_fb_fault_level != rhs->veh_fb_fault_level) {
    return false;
  }
  // veh_fb_auto_can_ctrl_cmd
  if (lhs->veh_fb_auto_can_ctrl_cmd != rhs->veh_fb_auto_can_ctrl_cmd) {
    return false;
  }
  // veh_fb_auto_io_can_cmd
  if (lhs->veh_fb_auto_io_can_cmd != rhs->veh_fb_auto_io_can_cmd) {
    return false;
  }
  // veh_fb_eps_dis_on_line
  if (lhs->veh_fb_eps_dis_on_line != rhs->veh_fb_eps_dis_on_line) {
    return false;
  }
  // veh_fb_eps_fault
  if (lhs->veh_fb_eps_fault != rhs->veh_fb_eps_fault) {
    return false;
  }
  // veh_fb_eps_mosf_et_ot
  if (lhs->veh_fb_eps_mosf_et_ot != rhs->veh_fb_eps_mosf_et_ot) {
    return false;
  }
  // veh_fb_eps_warning
  if (lhs->veh_fb_eps_warning != rhs->veh_fb_eps_warning) {
    return false;
  }
  // veh_fb_eps_dis_work
  if (lhs->veh_fb_eps_dis_work != rhs->veh_fb_eps_dis_work) {
    return false;
  }
  // veh_fb_eps_over_current
  if (lhs->veh_fb_eps_over_current != rhs->veh_fb_eps_over_current) {
    return false;
  }
  // veh_fb_st_reserve
  if (lhs->veh_fb_st_reserve != rhs->veh_fb_st_reserve) {
    return false;
  }
  // veh_fb_ehb_ecu_fault
  if (lhs->veh_fb_ehb_ecu_fault != rhs->veh_fb_ehb_ecu_fault) {
    return false;
  }
  // veh_fb_ehb_dis_on_line
  if (lhs->veh_fb_ehb_dis_on_line != rhs->veh_fb_ehb_dis_on_line) {
    return false;
  }
  // veh_fb_ehb_work_model_fault
  if (lhs->veh_fb_ehb_work_model_fault != rhs->veh_fb_ehb_work_model_fault) {
    return false;
  }
  // veh_fb_ehb_dis_en
  if (lhs->veh_fb_ehb_dis_en != rhs->veh_fb_ehb_dis_en) {
    return false;
  }
  // veh_fb_ehb_anguler_fault
  if (lhs->veh_fb_ehb_anguler_fault != rhs->veh_fb_ehb_anguler_fault) {
    return false;
  }
  // veh_fb_ehb_ot
  if (lhs->veh_fb_ehb_ot != rhs->veh_fb_ehb_ot) {
    return false;
  }
  // veh_fb_ehb_power_fault
  if (lhs->veh_fb_ehb_power_fault != rhs->veh_fb_ehb_power_fault) {
    return false;
  }
  // veh_fb_ehb_sensor_abnomal
  if (lhs->veh_fb_ehb_sensor_abnomal != rhs->veh_fb_ehb_sensor_abnomal) {
    return false;
  }
  // veh_fb_ehb_motor_fault
  if (lhs->veh_fb_ehb_motor_fault != rhs->veh_fb_ehb_motor_fault) {
    return false;
  }
  // veh_fb_ehb_oil_press_sensor_fault
  if (lhs->veh_fb_ehb_oil_press_sensor_fault != rhs->veh_fb_ehb_oil_press_sensor_fault) {
    return false;
  }
  // veh_fb_ehb_oil_fault
  if (lhs->veh_fb_ehb_oil_fault != rhs->veh_fb_ehb_oil_fault) {
    return false;
  }
  // veh_fb_ld_rv_mcu_fault
  if (lhs->veh_fb_ld_rv_mcu_fault != rhs->veh_fb_ld_rv_mcu_fault) {
    return false;
  }
  // veh_fb_rd_rv_mcu_fault
  if (lhs->veh_fb_rd_rv_mcu_fault != rhs->veh_fb_rd_rv_mcu_fault) {
    return false;
  }
  // veh_fb_aux_bms_dis_on_line
  if (lhs->veh_fb_aux_bms_dis_on_line != rhs->veh_fb_aux_bms_dis_on_line) {
    return false;
  }
  // veh_fb_aux_scram
  if (lhs->veh_fb_aux_scram != rhs->veh_fb_aux_scram) {
    return false;
  }
  // veh_fb_aux_remote_close
  if (lhs->veh_fb_aux_remote_close != rhs->veh_fb_aux_remote_close) {
    return false;
  }
  // veh_fb_aux_remote_dis_on_line
  if (lhs->veh_fb_aux_remote_dis_on_line != rhs->veh_fb_aux_remote_dis_on_line) {
    return false;
  }
  return true;
}

bool
yhs_can_interfaces__msg__VehDiagFb__copy(
  const yhs_can_interfaces__msg__VehDiagFb * input,
  yhs_can_interfaces__msg__VehDiagFb * output)
{
  if (!input || !output) {
    return false;
  }
  // veh_fb_fault_level
  output->veh_fb_fault_level = input->veh_fb_fault_level;
  // veh_fb_auto_can_ctrl_cmd
  output->veh_fb_auto_can_ctrl_cmd = input->veh_fb_auto_can_ctrl_cmd;
  // veh_fb_auto_io_can_cmd
  output->veh_fb_auto_io_can_cmd = input->veh_fb_auto_io_can_cmd;
  // veh_fb_eps_dis_on_line
  output->veh_fb_eps_dis_on_line = input->veh_fb_eps_dis_on_line;
  // veh_fb_eps_fault
  output->veh_fb_eps_fault = input->veh_fb_eps_fault;
  // veh_fb_eps_mosf_et_ot
  output->veh_fb_eps_mosf_et_ot = input->veh_fb_eps_mosf_et_ot;
  // veh_fb_eps_warning
  output->veh_fb_eps_warning = input->veh_fb_eps_warning;
  // veh_fb_eps_dis_work
  output->veh_fb_eps_dis_work = input->veh_fb_eps_dis_work;
  // veh_fb_eps_over_current
  output->veh_fb_eps_over_current = input->veh_fb_eps_over_current;
  // veh_fb_st_reserve
  output->veh_fb_st_reserve = input->veh_fb_st_reserve;
  // veh_fb_ehb_ecu_fault
  output->veh_fb_ehb_ecu_fault = input->veh_fb_ehb_ecu_fault;
  // veh_fb_ehb_dis_on_line
  output->veh_fb_ehb_dis_on_line = input->veh_fb_ehb_dis_on_line;
  // veh_fb_ehb_work_model_fault
  output->veh_fb_ehb_work_model_fault = input->veh_fb_ehb_work_model_fault;
  // veh_fb_ehb_dis_en
  output->veh_fb_ehb_dis_en = input->veh_fb_ehb_dis_en;
  // veh_fb_ehb_anguler_fault
  output->veh_fb_ehb_anguler_fault = input->veh_fb_ehb_anguler_fault;
  // veh_fb_ehb_ot
  output->veh_fb_ehb_ot = input->veh_fb_ehb_ot;
  // veh_fb_ehb_power_fault
  output->veh_fb_ehb_power_fault = input->veh_fb_ehb_power_fault;
  // veh_fb_ehb_sensor_abnomal
  output->veh_fb_ehb_sensor_abnomal = input->veh_fb_ehb_sensor_abnomal;
  // veh_fb_ehb_motor_fault
  output->veh_fb_ehb_motor_fault = input->veh_fb_ehb_motor_fault;
  // veh_fb_ehb_oil_press_sensor_fault
  output->veh_fb_ehb_oil_press_sensor_fault = input->veh_fb_ehb_oil_press_sensor_fault;
  // veh_fb_ehb_oil_fault
  output->veh_fb_ehb_oil_fault = input->veh_fb_ehb_oil_fault;
  // veh_fb_ld_rv_mcu_fault
  output->veh_fb_ld_rv_mcu_fault = input->veh_fb_ld_rv_mcu_fault;
  // veh_fb_rd_rv_mcu_fault
  output->veh_fb_rd_rv_mcu_fault = input->veh_fb_rd_rv_mcu_fault;
  // veh_fb_aux_bms_dis_on_line
  output->veh_fb_aux_bms_dis_on_line = input->veh_fb_aux_bms_dis_on_line;
  // veh_fb_aux_scram
  output->veh_fb_aux_scram = input->veh_fb_aux_scram;
  // veh_fb_aux_remote_close
  output->veh_fb_aux_remote_close = input->veh_fb_aux_remote_close;
  // veh_fb_aux_remote_dis_on_line
  output->veh_fb_aux_remote_dis_on_line = input->veh_fb_aux_remote_dis_on_line;
  return true;
}

yhs_can_interfaces__msg__VehDiagFb *
yhs_can_interfaces__msg__VehDiagFb__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__VehDiagFb * msg = (yhs_can_interfaces__msg__VehDiagFb *)allocator.allocate(sizeof(yhs_can_interfaces__msg__VehDiagFb), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yhs_can_interfaces__msg__VehDiagFb));
  bool success = yhs_can_interfaces__msg__VehDiagFb__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yhs_can_interfaces__msg__VehDiagFb__destroy(yhs_can_interfaces__msg__VehDiagFb * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yhs_can_interfaces__msg__VehDiagFb__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yhs_can_interfaces__msg__VehDiagFb__Sequence__init(yhs_can_interfaces__msg__VehDiagFb__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__VehDiagFb * data = NULL;

  if (size) {
    data = (yhs_can_interfaces__msg__VehDiagFb *)allocator.zero_allocate(size, sizeof(yhs_can_interfaces__msg__VehDiagFb), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yhs_can_interfaces__msg__VehDiagFb__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yhs_can_interfaces__msg__VehDiagFb__fini(&data[i - 1]);
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
yhs_can_interfaces__msg__VehDiagFb__Sequence__fini(yhs_can_interfaces__msg__VehDiagFb__Sequence * array)
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
      yhs_can_interfaces__msg__VehDiagFb__fini(&array->data[i]);
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

yhs_can_interfaces__msg__VehDiagFb__Sequence *
yhs_can_interfaces__msg__VehDiagFb__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yhs_can_interfaces__msg__VehDiagFb__Sequence * array = (yhs_can_interfaces__msg__VehDiagFb__Sequence *)allocator.allocate(sizeof(yhs_can_interfaces__msg__VehDiagFb__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yhs_can_interfaces__msg__VehDiagFb__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yhs_can_interfaces__msg__VehDiagFb__Sequence__destroy(yhs_can_interfaces__msg__VehDiagFb__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yhs_can_interfaces__msg__VehDiagFb__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yhs_can_interfaces__msg__VehDiagFb__Sequence__are_equal(const yhs_can_interfaces__msg__VehDiagFb__Sequence * lhs, const yhs_can_interfaces__msg__VehDiagFb__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yhs_can_interfaces__msg__VehDiagFb__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yhs_can_interfaces__msg__VehDiagFb__Sequence__copy(
  const yhs_can_interfaces__msg__VehDiagFb__Sequence * input,
  yhs_can_interfaces__msg__VehDiagFb__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yhs_can_interfaces__msg__VehDiagFb);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yhs_can_interfaces__msg__VehDiagFb * data =
      (yhs_can_interfaces__msg__VehDiagFb *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yhs_can_interfaces__msg__VehDiagFb__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yhs_can_interfaces__msg__VehDiagFb__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yhs_can_interfaces__msg__VehDiagFb__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
