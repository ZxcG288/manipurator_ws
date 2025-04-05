// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from interface_manipurator:msg/SpeedAngle.idl
// generated code does not contain a copyright notice

#include "interface_manipurator/msg/detail/speed_angle__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_interface_manipurator
const rosidl_type_hash_t *
interface_manipurator__msg__SpeedAngle__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x01, 0xf0, 0x58, 0xca, 0x9f, 0x94, 0x5e, 0x42,
      0xf1, 0xbd, 0x70, 0x35, 0xeb, 0x3f, 0x5d, 0x54,
      0x7c, 0xb5, 0x88, 0x32, 0xa3, 0x64, 0x9b, 0x44,
      0xee, 0x9f, 0x83, 0x28, 0x69, 0xac, 0xaf, 0x79,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char interface_manipurator__msg__SpeedAngle__TYPE_NAME[] = "interface_manipurator/msg/SpeedAngle";

// Define type names, field names, and default values
static char interface_manipurator__msg__SpeedAngle__FIELD_NAME__speed[] = "speed";
static char interface_manipurator__msg__SpeedAngle__FIELD_NAME__angle[] = "angle";

static rosidl_runtime_c__type_description__Field interface_manipurator__msg__SpeedAngle__FIELDS[] = {
  {
    {interface_manipurator__msg__SpeedAngle__FIELD_NAME__speed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interface_manipurator__msg__SpeedAngle__FIELD_NAME__angle, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
interface_manipurator__msg__SpeedAngle__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interface_manipurator__msg__SpeedAngle__TYPE_NAME, 36, 36},
      {interface_manipurator__msg__SpeedAngle__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int64 speed\n"
  "int64 angle";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
interface_manipurator__msg__SpeedAngle__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interface_manipurator__msg__SpeedAngle__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 23, 23},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interface_manipurator__msg__SpeedAngle__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interface_manipurator__msg__SpeedAngle__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
