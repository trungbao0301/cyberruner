// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from cyberrunner_interfaces:msg/StateEstimate.idl
// generated code does not contain a copyright notice

#include "cyberrunner_interfaces/msg/detail/state_estimate__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_cyberrunner_interfaces
const rosidl_type_hash_t *
cyberrunner_interfaces__msg__StateEstimate__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7e, 0xe4, 0xee, 0x2d, 0x4d, 0x75, 0xae, 0xd4,
      0x36, 0x8b, 0x4f, 0x73, 0xc3, 0x5b, 0xc4, 0x67,
      0x7b, 0x9b, 0xc3, 0x33, 0xfa, 0x37, 0x02, 0x34,
      0x94, 0x3a, 0x05, 0x0e, 0x38, 0x9c, 0x4c, 0x84,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char cyberrunner_interfaces__msg__StateEstimate__TYPE_NAME[] = "cyberrunner_interfaces/msg/StateEstimate";

// Define type names, field names, and default values
static char cyberrunner_interfaces__msg__StateEstimate__FIELD_NAME__x_b[] = "x_b";
static char cyberrunner_interfaces__msg__StateEstimate__FIELD_NAME__y_b[] = "y_b";
static char cyberrunner_interfaces__msg__StateEstimate__FIELD_NAME__x_b_dot[] = "x_b_dot";
static char cyberrunner_interfaces__msg__StateEstimate__FIELD_NAME__y_b_dot[] = "y_b_dot";
static char cyberrunner_interfaces__msg__StateEstimate__FIELD_NAME__alpha[] = "alpha";
static char cyberrunner_interfaces__msg__StateEstimate__FIELD_NAME__beta[] = "beta";

static rosidl_runtime_c__type_description__Field cyberrunner_interfaces__msg__StateEstimate__FIELDS[] = {
  {
    {cyberrunner_interfaces__msg__StateEstimate__FIELD_NAME__x_b, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cyberrunner_interfaces__msg__StateEstimate__FIELD_NAME__y_b, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cyberrunner_interfaces__msg__StateEstimate__FIELD_NAME__x_b_dot, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cyberrunner_interfaces__msg__StateEstimate__FIELD_NAME__y_b_dot, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cyberrunner_interfaces__msg__StateEstimate__FIELD_NAME__alpha, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cyberrunner_interfaces__msg__StateEstimate__FIELD_NAME__beta, 4, 4},
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
cyberrunner_interfaces__msg__StateEstimate__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cyberrunner_interfaces__msg__StateEstimate__TYPE_NAME, 40, 40},
      {cyberrunner_interfaces__msg__StateEstimate__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 x_b\n"
  "float64 y_b\n"
  "float64 x_b_dot\n"
  "float64 y_b_dot\n"
  "float64 alpha\n"
  "float64 beta";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
cyberrunner_interfaces__msg__StateEstimate__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cyberrunner_interfaces__msg__StateEstimate__TYPE_NAME, 40, 40},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 83, 83},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cyberrunner_interfaces__msg__StateEstimate__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cyberrunner_interfaces__msg__StateEstimate__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
