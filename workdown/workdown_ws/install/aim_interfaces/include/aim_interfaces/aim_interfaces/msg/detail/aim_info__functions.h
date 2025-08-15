// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from aim_interfaces:msg/AimInfo.idl
// generated code does not contain a copyright notice

#ifndef AIM_INTERFACES__MSG__DETAIL__AIM_INFO__FUNCTIONS_H_
#define AIM_INTERFACES__MSG__DETAIL__AIM_INFO__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "aim_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "aim_interfaces/msg/detail/aim_info__struct.h"

/// Initialize msg/AimInfo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * aim_interfaces__msg__AimInfo
 * )) before or use
 * aim_interfaces__msg__AimInfo__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_aim_interfaces
bool
aim_interfaces__msg__AimInfo__init(aim_interfaces__msg__AimInfo * msg);

/// Finalize msg/AimInfo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_aim_interfaces
void
aim_interfaces__msg__AimInfo__fini(aim_interfaces__msg__AimInfo * msg);

/// Create msg/AimInfo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * aim_interfaces__msg__AimInfo__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_aim_interfaces
aim_interfaces__msg__AimInfo *
aim_interfaces__msg__AimInfo__create();

/// Destroy msg/AimInfo message.
/**
 * It calls
 * aim_interfaces__msg__AimInfo__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_aim_interfaces
void
aim_interfaces__msg__AimInfo__destroy(aim_interfaces__msg__AimInfo * msg);

/// Check for msg/AimInfo message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_aim_interfaces
bool
aim_interfaces__msg__AimInfo__are_equal(const aim_interfaces__msg__AimInfo * lhs, const aim_interfaces__msg__AimInfo * rhs);

/// Copy a msg/AimInfo message.
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
ROSIDL_GENERATOR_C_PUBLIC_aim_interfaces
bool
aim_interfaces__msg__AimInfo__copy(
  const aim_interfaces__msg__AimInfo * input,
  aim_interfaces__msg__AimInfo * output);

/// Initialize array of msg/AimInfo messages.
/**
 * It allocates the memory for the number of elements and calls
 * aim_interfaces__msg__AimInfo__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_aim_interfaces
bool
aim_interfaces__msg__AimInfo__Sequence__init(aim_interfaces__msg__AimInfo__Sequence * array, size_t size);

/// Finalize array of msg/AimInfo messages.
/**
 * It calls
 * aim_interfaces__msg__AimInfo__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_aim_interfaces
void
aim_interfaces__msg__AimInfo__Sequence__fini(aim_interfaces__msg__AimInfo__Sequence * array);

/// Create array of msg/AimInfo messages.
/**
 * It allocates the memory for the array and calls
 * aim_interfaces__msg__AimInfo__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_aim_interfaces
aim_interfaces__msg__AimInfo__Sequence *
aim_interfaces__msg__AimInfo__Sequence__create(size_t size);

/// Destroy array of msg/AimInfo messages.
/**
 * It calls
 * aim_interfaces__msg__AimInfo__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_aim_interfaces
void
aim_interfaces__msg__AimInfo__Sequence__destroy(aim_interfaces__msg__AimInfo__Sequence * array);

/// Check for msg/AimInfo message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_aim_interfaces
bool
aim_interfaces__msg__AimInfo__Sequence__are_equal(const aim_interfaces__msg__AimInfo__Sequence * lhs, const aim_interfaces__msg__AimInfo__Sequence * rhs);

/// Copy an array of msg/AimInfo messages.
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
ROSIDL_GENERATOR_C_PUBLIC_aim_interfaces
bool
aim_interfaces__msg__AimInfo__Sequence__copy(
  const aim_interfaces__msg__AimInfo__Sequence * input,
  aim_interfaces__msg__AimInfo__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AIM_INTERFACES__MSG__DETAIL__AIM_INFO__FUNCTIONS_H_
