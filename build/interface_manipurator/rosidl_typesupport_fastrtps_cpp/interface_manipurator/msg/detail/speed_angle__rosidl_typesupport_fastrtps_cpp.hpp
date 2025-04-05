// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from interface_manipurator:msg/SpeedAngle.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "interface_manipurator/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "interface_manipurator/msg/detail/speed_angle__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace interface_manipurator
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interface_manipurator
cdr_serialize(
  const interface_manipurator::msg::SpeedAngle & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interface_manipurator
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  interface_manipurator::msg::SpeedAngle & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interface_manipurator
get_serialized_size(
  const interface_manipurator::msg::SpeedAngle & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interface_manipurator
max_serialized_size_SpeedAngle(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interface_manipurator
cdr_serialize_key(
  const interface_manipurator::msg::SpeedAngle & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interface_manipurator
get_serialized_size_key(
  const interface_manipurator::msg::SpeedAngle & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interface_manipurator
max_serialized_size_key_SpeedAngle(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace interface_manipurator

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interface_manipurator
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, interface_manipurator, msg, SpeedAngle)();

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_MANIPURATOR__MSG__DETAIL__SPEED_ANGLE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
