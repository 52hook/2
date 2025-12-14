// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from yhs_can_interfaces:msg/ChassisInfoFb.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__CHASSIS_INFO_FB__FUNCTIONS_H_
#define YHS_CAN_INTERFACES__MSG__DETAIL__CHASSIS_INFO_FB__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "yhs_can_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "yhs_can_interfaces/msg/detail/chassis_info_fb__struct.h"

/// Initialize msg/ChassisInfoFb message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * yhs_can_interfaces__msg__ChassisInfoFb
 * )) before or use
 * yhs_can_interfaces__msg__ChassisInfoFb__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
bool
yhs_can_interfaces__msg__ChassisInfoFb__init(yhs_can_interfaces__msg__ChassisInfoFb * msg);

/// Finalize msg/ChassisInfoFb message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
void
yhs_can_interfaces__msg__ChassisInfoFb__fini(yhs_can_interfaces__msg__ChassisInfoFb * msg);

/// Create msg/ChassisInfoFb message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * yhs_can_interfaces__msg__ChassisInfoFb__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
yhs_can_interfaces__msg__ChassisInfoFb *
yhs_can_interfaces__msg__ChassisInfoFb__create();

/// Destroy msg/ChassisInfoFb message.
/**
 * It calls
 * yhs_can_interfaces__msg__ChassisInfoFb__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
void
yhs_can_interfaces__msg__ChassisInfoFb__destroy(yhs_can_interfaces__msg__ChassisInfoFb * msg);

/// Check for msg/ChassisInfoFb message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
bool
yhs_can_interfaces__msg__ChassisInfoFb__are_equal(const yhs_can_interfaces__msg__ChassisInfoFb * lhs, const yhs_can_interfaces__msg__ChassisInfoFb * rhs);

/// Copy a msg/ChassisInfoFb message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
bool
yhs_can_interfaces__msg__ChassisInfoFb__copy(
  const yhs_can_interfaces__msg__ChassisInfoFb * input,
  yhs_can_interfaces__msg__ChassisInfoFb * output);

/// Initialize array of msg/ChassisInfoFb messages.
/**
 * It allocates the memory for the number of elements and calls
 * yhs_can_interfaces__msg__ChassisInfoFb__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
bool
yhs_can_interfaces__msg__ChassisInfoFb__Sequence__init(yhs_can_interfaces__msg__ChassisInfoFb__Sequence * array, size_t size);

/// Finalize array of msg/ChassisInfoFb messages.
/**
 * It calls
 * yhs_can_interfaces__msg__ChassisInfoFb__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
void
yhs_can_interfaces__msg__ChassisInfoFb__Sequence__fini(yhs_can_interfaces__msg__ChassisInfoFb__Sequence * array);

/// Create array of msg/ChassisInfoFb messages.
/**
 * It allocates the memory for the array and calls
 * yhs_can_interfaces__msg__ChassisInfoFb__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
yhs_can_interfaces__msg__ChassisInfoFb__Sequence *
yhs_can_interfaces__msg__ChassisInfoFb__Sequence__create(size_t size);

/// Destroy array of msg/ChassisInfoFb messages.
/**
 * It calls
 * yhs_can_interfaces__msg__ChassisInfoFb__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
void
yhs_can_interfaces__msg__ChassisInfoFb__Sequence__destroy(yhs_can_interfaces__msg__ChassisInfoFb__Sequence * array);

/// Check for msg/ChassisInfoFb message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
bool
yhs_can_interfaces__msg__ChassisInfoFb__Sequence__are_equal(const yhs_can_interfaces__msg__ChassisInfoFb__Sequence * lhs, const yhs_can_interfaces__msg__ChassisInfoFb__Sequence * rhs);

/// Copy an array of msg/ChassisInfoFb messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
bool
yhs_can_interfaces__msg__ChassisInfoFb__Sequence__copy(
  const yhs_can_interfaces__msg__ChassisInfoFb__Sequence * input,
  yhs_can_interfaces__msg__ChassisInfoFb__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__CHASSIS_INFO_FB__FUNCTIONS_H_
