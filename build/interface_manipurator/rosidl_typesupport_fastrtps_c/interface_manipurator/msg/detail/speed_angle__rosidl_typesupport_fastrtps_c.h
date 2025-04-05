// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from interface_manipurator:msg/SpeedAngle.idl
// generated code does not contain a copyright notice
#ifndef INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "interface_manipurator/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "interface_manipurator/msg/detail/speed_angle__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface_manipurator
bool cdr_serialize_interface_manipurator__msg__SpeedAngle(
  const interface_manipurator__msg__SpeedAngle * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface_manipurator
bool cdr_deserialize_interface_manipurator__msg__SpeedAngle(
  eprosima::fastcdr::Cdr &,
  interface_manipurator__msg__SpeedAngle * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface_manipurator
size_t get_serialized_size_interface_manipurator__msg__SpeedAngle(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface_manipurator
size_t max_serialized_size_interface_manipurator__msg__SpeedAngle(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface_manipurator
bool cdr_serialize_key_interface_manipurator__msg__SpeedAngle(
  const interface_manipurator__msg__SpeedAngle * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface_manipurator
size_t get_serialized_size_key_interface_manipurator__msg__SpeedAngle(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface_manipurator
size_t max_serialized_size_key_interface_manipurator__msg__SpeedAngle(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface_manipurator
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface_manipurator, msg, SpeedAngle)();

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
