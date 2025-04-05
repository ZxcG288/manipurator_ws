// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from interface_manipurator:msg/SpeedAngle.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "interface_manipurator/msg/detail/speed_angle__rosidl_typesupport_introspection_c.h"
#include "interface_manipurator/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "interface_manipurator/msg/detail/speed_angle__functions.h"
#include "interface_manipurator/msg/detail/speed_angle__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void interface_manipurator__msg__SpeedAngle__rosidl_typesupport_introspection_c__SpeedAngle_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interface_manipurator__msg__SpeedAngle__init(message_memory);
}

void interface_manipurator__msg__SpeedAngle__rosidl_typesupport_introspection_c__SpeedAngle_fini_function(void * message_memory)
{
  interface_manipurator__msg__SpeedAngle__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember interface_manipurator__msg__SpeedAngle__rosidl_typesupport_introspection_c__SpeedAngle_message_member_array[2] = {
  {
    "speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_manipurator__msg__SpeedAngle, speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_manipurator__msg__SpeedAngle, angle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers interface_manipurator__msg__SpeedAngle__rosidl_typesupport_introspection_c__SpeedAngle_message_members = {
  "interface_manipurator__msg",  // message namespace
  "SpeedAngle",  // message name
  2,  // number of fields
  sizeof(interface_manipurator__msg__SpeedAngle),
  false,  // has_any_key_member_
  interface_manipurator__msg__SpeedAngle__rosidl_typesupport_introspection_c__SpeedAngle_message_member_array,  // message members
  interface_manipurator__msg__SpeedAngle__rosidl_typesupport_introspection_c__SpeedAngle_init_function,  // function to initialize message memory (memory has to be allocated)
  interface_manipurator__msg__SpeedAngle__rosidl_typesupport_introspection_c__SpeedAngle_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t interface_manipurator__msg__SpeedAngle__rosidl_typesupport_introspection_c__SpeedAngle_message_type_support_handle = {
  0,
  &interface_manipurator__msg__SpeedAngle__rosidl_typesupport_introspection_c__SpeedAngle_message_members,
  get_message_typesupport_handle_function,
  &interface_manipurator__msg__SpeedAngle__get_type_hash,
  &interface_manipurator__msg__SpeedAngle__get_type_description,
  &interface_manipurator__msg__SpeedAngle__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface_manipurator
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_manipurator, msg, SpeedAngle)() {
  if (!interface_manipurator__msg__SpeedAngle__rosidl_typesupport_introspection_c__SpeedAngle_message_type_support_handle.typesupport_identifier) {
    interface_manipurator__msg__SpeedAngle__rosidl_typesupport_introspection_c__SpeedAngle_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &interface_manipurator__msg__SpeedAngle__rosidl_typesupport_introspection_c__SpeedAngle_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
