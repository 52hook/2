// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from yhs_can_interfaces:msg/IoCmd.idl
// generated code does not contain a copyright notice

#ifndef YHS_CAN_INTERFACES__MSG__DETAIL__IO_CMD__FUNCTIONS_H_
#define YHS_CAN_INTERFACES__MSG__DETAIL__IO_CMD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "yhs_can_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "yhs_can_interfaces/msg/detail/io_cmd__struct.h"

/// Initialize msg/IoCmd message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * yhs_can_interfaces__msg__IoCmd
 * )) before or use
 * yhs_can_interfaces__msg__IoCmd__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
bool
yhs_can_interfaces__msg__IoCmd__init(yhs_can_interfaces__msg__IoCmd * msg);

/// Finalize msg/IoCmd message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
void
yhs_can_interfaces__msg__IoCmd__fini(yhs_can_interfaces__msg__IoCmd * msg);

/// Create msg/IoCmd message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * yhs_can_interfaces__msg__IoCmd__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
yhs_can_interfaces__msg__IoCmd *
yhs_can_interfaces__msg__IoCmd__create();

/// Destroy msg/IoCmd message.
/**
 * It calls
 * yhs_can_interfaces__msg__IoCmd__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
void
yhs_can_interfaces__msg__IoCmd__destroy(yhs_can_interfaces__msg__IoCmd * msg);

/// Check for msg/IoCmd message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
bool
yhs_can_interfaces__msg__IoCmd__are_equal(const yhs_can_interfaces__msg__IoCmd * lhs, const yhs_can_interfaces__msg__IoCmd * rhs);

/// Copy a msg/IoCmd message.
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
yhs_can_interfaces__msg__IoCmd__copy(
  const yhs_can_interfaces__msg__IoCmd * input,
  yhs_can_interfaces__msg__IoCmd * output);

/// Initialize array of msg/IoCmd messages.
/**
 * It allocates the memory for the number of elements and calls
 * yhs_can_interfaces__msg__IoCmd__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
bool
yhs_can_interfaces__msg__IoCmd__Sequence__init(yhs_can_interfaces__msg__IoCmd__Sequence * array, size_t size);

/// Finalize array of msg/IoCmd messages.
/**
 * It calls
 * yhs_can_interfaces__msg__IoCmd__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
void
yhs_can_interfaces__msg__IoCmd__Sequence__fini(yhs_can_interfaces__msg__IoCmd__Sequence * array);

/// Create array of msg/IoCmd messages.
/**
 * It allocates the memory for the array and calls
 * yhs_can_interfaces__msg__IoCmd__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
yhs_can_interfaces__msg__IoCmd__Sequence *
yhs_can_interfaces__msg__IoCmd__Sequence__create(size_t size);

/// Destroy array of msg/IoCmd messages.
/**
 * It calls
 * yhs_can_interfaces__msg__IoCmd__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
void
yhs_can_interfaces__msg__IoCmd__Sequence__destroy(yhs_can_interfaces__msg__IoCmd__Sequence * array);

/// Check for msg/IoCmd message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_yhs_can_interfaces
bool
yhs_can_interfaces__msg__IoCmd__Sequence__are_equal(const yhs_can_interfaces__msg__IoCmd__Sequence * lhs, const yhs_can_interfaces__msg__IoCmd__Sequence * rhs);

/// Copy an array of msg/IoCmd messages.
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
yhs_can_interfaces__msg__IoCmd__Sequence__copy(
  const yhs_can_interfaces__msg__IoCmd__Sequence * input,
  yhs_can_interfaces__msg__IoCmd__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // YHS_CAN_INTERFACES__MSG__DETAIL__IO_CMD__FUNCTIONS_H_
