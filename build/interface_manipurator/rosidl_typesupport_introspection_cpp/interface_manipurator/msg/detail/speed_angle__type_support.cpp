// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from interface_manipurator:msg/SpeedAngle.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "interface_manipurator/msg/detail/speed_angle__functions.h"
#include "interface_manipurator/msg/detail/speed_angle__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace interface_manipurator
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void SpeedAngle_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) interface_manipurator::msg::SpeedAngle(_init);
}

void SpeedAngle_fini_function(void * message_memory)
{
  auto typed_message = static_cast<interface_manipurator::msg::SpeedAngle *>(message_memory);
  typed_message->~SpeedAngle();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SpeedAngle_message_member_array[2] = {
  {
    "speed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_manipurator::msg::SpeedAngle, speed),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "angle",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_manipurator::msg::SpeedAngle, angle),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SpeedAngle_message_members = {
  "interface_manipurator::msg",  // message namespace
  "SpeedAngle",  // message name
  2,  // number of fields
  sizeof(interface_manipurator::msg::SpeedAngle),
  false,  // has_any_key_member_
  SpeedAngle_message_member_array,  // message members
  SpeedAngle_init_function,  // function to initialize message memory (memory has to be allocated)
  SpeedAngle_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SpeedAngle_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SpeedAngle_message_members,
  get_message_typesupport_handle_function,
  &interface_manipurator__msg__SpeedAngle__get_type_hash,
  &interface_manipurator__msg__SpeedAngle__get_type_description,
  &interface_manipurator__msg__SpeedAngle__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace interface_manipurator


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<interface_manipurator::msg::SpeedAngle>()
{
  return &::interface_manipurator::msg::rosidl_typesupport_introspection_cpp::SpeedAngle_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, interface_manipurator, msg, SpeedAngle)() {
  return &::interface_manipurator::msg::rosidl_typesupport_introspection_cpp::SpeedAngle_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
