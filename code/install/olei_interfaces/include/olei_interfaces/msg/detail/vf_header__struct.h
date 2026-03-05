// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from olei_interfaces:msg/VFHeader.idl
// generated code does not contain a copyright notice

#ifndef OLEI_INTERFACES__MSG__DETAIL__VF_HEADER__STRUCT_H_
#define OLEI_INTERFACES__MSG__DETAIL__VF_HEADER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/VFHeader in the package olei_interfaces.
typedef struct olei_interfaces__msg__VFHeader
{
  uint16_t magic;
  uint16_t version;
  uint32_t packet_size;
  uint16_t header_size;
  uint8_t distance_ratio;
  uint8_t types;
  uint16_t scan_number;
  uint16_t packet_number;
  uint32_t timestamp_decimal;
  uint32_t timestamp_integer;
  uint16_t scan_frequency;
  uint16_t num_points_scan;
  uint16_t iutput_status;
  uint16_t output_status;
  uint32_t field_status;
  uint16_t start_index;
  uint16_t end_index;
  uint16_t first_index;
  uint16_t num_points_packet;
  uint32_t status_flags;
} olei_interfaces__msg__VFHeader;

// Struct for a sequence of olei_interfaces__msg__VFHeader.
typedef struct olei_interfaces__msg__VFHeader__Sequence
{
  olei_interfaces__msg__VFHeader * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} olei_interfaces__msg__VFHeader__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OLEI_INTERFACES__MSG__DETAIL__VF_HEADER__STRUCT_H_
