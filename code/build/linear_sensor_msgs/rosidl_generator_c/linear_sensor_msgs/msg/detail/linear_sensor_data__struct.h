// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from linear_sensor_msgs:msg/LinearSensorData.idl
// generated code does not contain a copyright notice

#ifndef LINEAR_SENSOR_MSGS__MSG__DETAIL__LINEAR_SENSOR_DATA__STRUCT_H_
#define LINEAR_SENSOR_MSGS__MSG__DETAIL__LINEAR_SENSOR_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/LinearSensorData in the package linear_sensor_msgs.
typedef struct linear_sensor_msgs__msg__LinearSensorData
{
  std_msgs__msg__Header header;
  float data;
} linear_sensor_msgs__msg__LinearSensorData;

// Struct for a sequence of linear_sensor_msgs__msg__LinearSensorData.
typedef struct linear_sensor_msgs__msg__LinearSensorData__Sequence
{
  linear_sensor_msgs__msg__LinearSensorData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} linear_sensor_msgs__msg__LinearSensorData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LINEAR_SENSOR_MSGS__MSG__DETAIL__LINEAR_SENSOR_DATA__STRUCT_H_
