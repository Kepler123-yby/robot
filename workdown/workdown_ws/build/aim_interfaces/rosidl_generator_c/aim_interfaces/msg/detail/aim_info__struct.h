// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aim_interfaces:msg/AimInfo.idl
// generated code does not contain a copyright notice

#ifndef AIM_INTERFACES__MSG__DETAIL__AIM_INFO__STRUCT_H_
#define AIM_INTERFACES__MSG__DETAIL__AIM_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'coordinate'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/AimInfo in the package aim_interfaces.
typedef struct aim_interfaces__msg__AimInfo
{
  rosidl_runtime_c__uint16__Sequence coordinate;
  uint16_t type;
} aim_interfaces__msg__AimInfo;

// Struct for a sequence of aim_interfaces__msg__AimInfo.
typedef struct aim_interfaces__msg__AimInfo__Sequence
{
  aim_interfaces__msg__AimInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aim_interfaces__msg__AimInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AIM_INTERFACES__MSG__DETAIL__AIM_INFO__STRUCT_H_
