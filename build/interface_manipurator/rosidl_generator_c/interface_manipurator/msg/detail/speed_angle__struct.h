// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface_manipurator:msg/SpeedAngle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interface_manipurator/msg/speed_angle.h"


#ifndef INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__STRUCT_H_
#define INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/SpeedAngle in the package interface_manipurator.
typedef struct interface_manipurator__msg__SpeedAngle
{
  int64_t speed;
  int64_t angle;
} interface_manipurator__msg__SpeedAngle;

// Struct for a sequence of interface_manipurator__msg__SpeedAngle.
typedef struct interface_manipurator__msg__SpeedAngle__Sequence
{
  interface_manipurator__msg__SpeedAngle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_manipurator__msg__SpeedAngle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__STRUCT_H_
