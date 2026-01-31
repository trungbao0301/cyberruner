// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from cyberrunner_interfaces:msg/StateEstimateSub.idl
// generated code does not contain a copyright notice

#include "cyberrunner_interfaces/msg/detail/state_estimate_sub__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_cyberrunner_interfaces
const rosidl_type_hash_t *
cyberrunner_interfaces__msg__StateEstimateSub__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0f, 0x1d, 0x7d, 0x7a, 0xb0, 0x80, 0x1f, 0x59,
      0x30, 0x55, 0x04, 0x64, 0x4c, 0x56, 0xa2, 0xe0,
      0xe7, 0xe9, 0x98, 0x46, 0x13, 0x74, 0xa9, 0x96,
      0x96, 0x97, 0x6e, 0x48, 0x3c, 0x2e, 0xd3, 0x57,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "cyberrunner_interfaces/msg/detail/state_estimate__functions.h"
#include "sensor_msgs/msg/detail/image__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t cyberrunner_interfaces__msg__StateEstimate__EXPECTED_HASH = {1, {
    0x7e, 0xe4, 0xee, 0x2d, 0x4d, 0x75, 0xae, 0xd4,
    0x36, 0x8b, 0x4f, 0x73, 0xc3, 0x5b, 0xc4, 0x67,
    0x7b, 0x9b, 0xc3, 0x33, 0xfa, 0x37, 0x02, 0x34,
    0x94, 0x3a, 0x05, 0x0e, 0x38, 0x9c, 0x4c, 0x84,
  }};
static const rosidl_type_hash_t sensor_msgs__msg__Image__EXPECTED_HASH = {1, {
    0xd3, 0x1d, 0x41, 0xa9, 0xa4, 0xc4, 0xbc, 0x8e,
    0xae, 0x9b, 0xe7, 0x57, 0xb0, 0xbe, 0xed, 0x30,
    0x65, 0x64, 0xf7, 0x52, 0x6c, 0x88, 0xea, 0x6a,
    0x45, 0x88, 0xfb, 0x95, 0x82, 0x52, 0x7d, 0x47,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char cyberrunner_interfaces__msg__StateEstimateSub__TYPE_NAME[] = "cyberrunner_interfaces/msg/StateEstimateSub";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char cyberrunner_interfaces__msg__StateEstimate__TYPE_NAME[] = "cyberrunner_interfaces/msg/StateEstimate";
static char sensor_msgs__msg__Image__TYPE_NAME[] = "sensor_msgs/msg/Image";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char cyberrunner_interfaces__msg__StateEstimateSub__FIELD_NAME__state[] = "state";
static char cyberrunner_interfaces__msg__StateEstimateSub__FIELD_NAME__subimg[] = "subimg";

static rosidl_runtime_c__type_description__Field cyberrunner_interfaces__msg__StateEstimateSub__FIELDS[] = {
  {
    {cyberrunner_interfaces__msg__StateEstimateSub__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cyberrunner_interfaces__msg__StateEstimate__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {cyberrunner_interfaces__msg__StateEstimateSub__FIELD_NAME__subimg, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {sensor_msgs__msg__Image__TYPE_NAME, 21, 21},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cyberrunner_interfaces__msg__StateEstimateSub__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {cyberrunner_interfaces__msg__StateEstimate__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {sensor_msgs__msg__Image__TYPE_NAME, 21, 21},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cyberrunner_interfaces__msg__StateEstimateSub__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cyberrunner_interfaces__msg__StateEstimateSub__TYPE_NAME, 43, 43},
      {cyberrunner_interfaces__msg__StateEstimateSub__FIELDS, 2, 2},
    },
    {cyberrunner_interfaces__msg__StateEstimateSub__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cyberrunner_interfaces__msg__StateEstimate__EXPECTED_HASH, cyberrunner_interfaces__msg__StateEstimate__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = cyberrunner_interfaces__msg__StateEstimate__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&sensor_msgs__msg__Image__EXPECTED_HASH, sensor_msgs__msg__Image__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = sensor_msgs__msg__Image__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "StateEstimate state\n"
  "sensor_msgs/Image subimg";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
cyberrunner_interfaces__msg__StateEstimateSub__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cyberrunner_interfaces__msg__StateEstimateSub__TYPE_NAME, 43, 43},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 44, 44},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cyberrunner_interfaces__msg__StateEstimateSub__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cyberrunner_interfaces__msg__StateEstimateSub__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *cyberrunner_interfaces__msg__StateEstimate__get_individual_type_description_source(NULL);
    sources[3] = *sensor_msgs__msg__Image__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
