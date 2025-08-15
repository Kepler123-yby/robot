// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from aim_interfaces:msg/AimInfo.idl
// generated code does not contain a copyright notice
#include "aim_interfaces/msg/detail/aim_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `coordinate`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
aim_interfaces__msg__AimInfo__init(aim_interfaces__msg__AimInfo * msg)
{
  if (!msg) {
    return false;
  }
  // coordinate
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->coordinate, 0)) {
    aim_interfaces__msg__AimInfo__fini(msg);
    return false;
  }
  // type
  return true;
}

void
aim_interfaces__msg__AimInfo__fini(aim_interfaces__msg__AimInfo * msg)
{
  if (!msg) {
    return;
  }
  // coordinate
  rosidl_runtime_c__uint16__Sequence__fini(&msg->coordinate);
  // type
}

bool
aim_interfaces__msg__AimInfo__are_equal(const aim_interfaces__msg__AimInfo * lhs, const aim_interfaces__msg__AimInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // coordinate
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->coordinate), &(rhs->coordinate)))
  {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  return true;
}

bool
aim_interfaces__msg__AimInfo__copy(
  const aim_interfaces__msg__AimInfo * input,
  aim_interfaces__msg__AimInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // coordinate
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->coordinate), &(output->coordinate)))
  {
    return false;
  }
  // type
  output->type = input->type;
  return true;
}

aim_interfaces__msg__AimInfo *
aim_interfaces__msg__AimInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aim_interfaces__msg__AimInfo * msg = (aim_interfaces__msg__AimInfo *)allocator.allocate(sizeof(aim_interfaces__msg__AimInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aim_interfaces__msg__AimInfo));
  bool success = aim_interfaces__msg__AimInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
aim_interfaces__msg__AimInfo__destroy(aim_interfaces__msg__AimInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    aim_interfaces__msg__AimInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
aim_interfaces__msg__AimInfo__Sequence__init(aim_interfaces__msg__AimInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aim_interfaces__msg__AimInfo * data = NULL;

  if (size) {
    data = (aim_interfaces__msg__AimInfo *)allocator.zero_allocate(size, sizeof(aim_interfaces__msg__AimInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aim_interfaces__msg__AimInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aim_interfaces__msg__AimInfo__fini(&data[i - 1]);
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
aim_interfaces__msg__AimInfo__Sequence__fini(aim_interfaces__msg__AimInfo__Sequence * array)
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
      aim_interfaces__msg__AimInfo__fini(&array->data[i]);
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

aim_interfaces__msg__AimInfo__Sequence *
aim_interfaces__msg__AimInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aim_interfaces__msg__AimInfo__Sequence * array = (aim_interfaces__msg__AimInfo__Sequence *)allocator.allocate(sizeof(aim_interfaces__msg__AimInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = aim_interfaces__msg__AimInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
aim_interfaces__msg__AimInfo__Sequence__destroy(aim_interfaces__msg__AimInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    aim_interfaces__msg__AimInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
aim_interfaces__msg__AimInfo__Sequence__are_equal(const aim_interfaces__msg__AimInfo__Sequence * lhs, const aim_interfaces__msg__AimInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aim_interfaces__msg__AimInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aim_interfaces__msg__AimInfo__Sequence__copy(
  const aim_interfaces__msg__AimInfo__Sequence * input,
  aim_interfaces__msg__AimInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aim_interfaces__msg__AimInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    aim_interfaces__msg__AimInfo * data =
      (aim_interfaces__msg__AimInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aim_interfaces__msg__AimInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          aim_interfaces__msg__AimInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!aim_interfaces__msg__AimInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
