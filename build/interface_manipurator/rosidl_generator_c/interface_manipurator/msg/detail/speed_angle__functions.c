// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interface_manipurator:msg/SpeedAngle.idl
// generated code does not contain a copyright notice
#include "interface_manipurator/msg/detail/speed_angle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
interface_manipurator__msg__SpeedAngle__init(interface_manipurator__msg__SpeedAngle * msg)
{
  if (!msg) {
    return false;
  }
  // speed
  // angle
  return true;
}

void
interface_manipurator__msg__SpeedAngle__fini(interface_manipurator__msg__SpeedAngle * msg)
{
  if (!msg) {
    return;
  }
  // speed
  // angle
}

bool
interface_manipurator__msg__SpeedAngle__are_equal(const interface_manipurator__msg__SpeedAngle * lhs, const interface_manipurator__msg__SpeedAngle * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // angle
  if (lhs->angle != rhs->angle) {
    return false;
  }
  return true;
}

bool
interface_manipurator__msg__SpeedAngle__copy(
  const interface_manipurator__msg__SpeedAngle * input,
  interface_manipurator__msg__SpeedAngle * output)
{
  if (!input || !output) {
    return false;
  }
  // speed
  output->speed = input->speed;
  // angle
  output->angle = input->angle;
  return true;
}

interface_manipurator__msg__SpeedAngle *
interface_manipurator__msg__SpeedAngle__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_manipurator__msg__SpeedAngle * msg = (interface_manipurator__msg__SpeedAngle *)allocator.allocate(sizeof(interface_manipurator__msg__SpeedAngle), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interface_manipurator__msg__SpeedAngle));
  bool success = interface_manipurator__msg__SpeedAngle__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interface_manipurator__msg__SpeedAngle__destroy(interface_manipurator__msg__SpeedAngle * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interface_manipurator__msg__SpeedAngle__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interface_manipurator__msg__SpeedAngle__Sequence__init(interface_manipurator__msg__SpeedAngle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_manipurator__msg__SpeedAngle * data = NULL;

  if (size) {
    data = (interface_manipurator__msg__SpeedAngle *)allocator.zero_allocate(size, sizeof(interface_manipurator__msg__SpeedAngle), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interface_manipurator__msg__SpeedAngle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interface_manipurator__msg__SpeedAngle__fini(&data[i - 1]);
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
interface_manipurator__msg__SpeedAngle__Sequence__fini(interface_manipurator__msg__SpeedAngle__Sequence * array)
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
      interface_manipurator__msg__SpeedAngle__fini(&array->data[i]);
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

interface_manipurator__msg__SpeedAngle__Sequence *
interface_manipurator__msg__SpeedAngle__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interface_manipurator__msg__SpeedAngle__Sequence * array = (interface_manipurator__msg__SpeedAngle__Sequence *)allocator.allocate(sizeof(interface_manipurator__msg__SpeedAngle__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interface_manipurator__msg__SpeedAngle__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interface_manipurator__msg__SpeedAngle__Sequence__destroy(interface_manipurator__msg__SpeedAngle__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interface_manipurator__msg__SpeedAngle__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interface_manipurator__msg__SpeedAngle__Sequence__are_equal(const interface_manipurator__msg__SpeedAngle__Sequence * lhs, const interface_manipurator__msg__SpeedAngle__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interface_manipurator__msg__SpeedAngle__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interface_manipurator__msg__SpeedAngle__Sequence__copy(
  const interface_manipurator__msg__SpeedAngle__Sequence * input,
  interface_manipurator__msg__SpeedAngle__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interface_manipurator__msg__SpeedAngle);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interface_manipurator__msg__SpeedAngle * data =
      (interface_manipurator__msg__SpeedAngle *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interface_manipurator__msg__SpeedAngle__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interface_manipurator__msg__SpeedAngle__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interface_manipurator__msg__SpeedAngle__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
