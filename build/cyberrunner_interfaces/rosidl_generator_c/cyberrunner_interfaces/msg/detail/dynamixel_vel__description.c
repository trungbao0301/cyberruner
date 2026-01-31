// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from cyberrunner_interfaces:msg/DynamixelVel.idl
// generated code does not contain a copyright notice

#include "cyberrunner_interfaces/msg/detail/dynamixel_vel__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_cyberrunner_interfaces
const rosidl_type_hash_t *
cyberrunner_interfaces__msg__DynamixelVel__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd8, 0x0c, 0x8c, 0x84, 0x79, 0xd1, 0x33, 0xa0,
      0xd2, 0x32, 0x69, 0x18, 0xcd, 0xd7, 0x04, 0xb4,
      0x6e, 0xe6, 0xd4, 0xd2, 0x13, 0x4e, 0x7e, 0xfd,
      0x3a, 0xd8, 0xe6, 0x43, 0x15, 0x16, 0x96, 0x6c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char cyberrunner_interfaces__msg__DynamixelVel__TYPE_NAME[] = "cyberrunner_interfaces/msg/DynamixelVel";

// Define type names, field names, and default values
static char cyberrunner_interfaces__msg__DynamixelVel__FIELD_NAME__vel_1[] = "vel_1";
static char cyberrunner_interfaces__msg__DynamixelVel__FIELD_NAME__vel_2[] = "vel_2";

static rosidl_runtime_c__type_description__Field cyberrunner_interfaces__msg__DynamixelVel__FIELDS[] = {
  {
    {cyberrunner_interfaces__msg__DynamixelVel__FIELD_NAME__vel_1, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cyberrunner_interfaces__msg__DynamixelVel__FIELD_NAME__vel_2, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cyberrunner_interfaces__msg__DynamixelVel__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cyberrunner_interfaces__msg__DynamixelVel__TYPE_NAME, 39, 39},
      {cyberrunner_interfaces__msg__DynamixelVel__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Velocity commands for the dynamixels  TODO: refactor\n"
  "\n"
  "float64 vel_1\n"
  "float64 vel_2";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
cyberrunner_interfaces__msg__DynamixelVel__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cyberrunner_interfaces__msg__DynamixelVel__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 84, 84},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cyberrunner_interfaces__msg__DynamixelVel__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cyberrunner_interfaces__msg__DynamixelVel__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
