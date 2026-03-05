// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from olei_interfaces:msg/V2Header.idl
// generated code does not contain a copyright notice

#ifndef OLEI_INTERFACES__MSG__DETAIL__V2_HEADER__STRUCT_H_
#define OLEI_INTERFACES__MSG__DETAIL__V2_HEADER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/V2Header in the package olei_interfaces.
typedef struct olei_interfaces__msg__V2Header
{
  uint32_t magic;
  uint16_t version;
  uint8_t distance_ratio;
  uint8_t brand[3];
  uint8_t vommercial[12];
  uint16_t internal;
  uint16_t hardware;
  uint16_t software;
  uint32_t timestamp;
  uint16_t scan_frequency;
  uint8_t safe_status;
  uint8_t error_status;
  uint32_t status_flags;
} olei_interfaces__msg__V2Header;

// Struct for a sequence of olei_interfaces__msg__V2Header.
typedef struct olei_interfaces__msg__V2Header__Sequence
{
  olei_interfaces__msg__V2Header * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} olei_interfaces__msg__V2Header__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OLEI_INTERFACES__MSG__DETAIL__V2_HEADER__STRUCT_H_
