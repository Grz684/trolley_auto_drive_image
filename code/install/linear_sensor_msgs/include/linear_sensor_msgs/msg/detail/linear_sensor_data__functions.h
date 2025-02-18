// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from linear_sensor_msgs:msg/LinearSensorData.idl
// generated code does not contain a copyright notice

#ifndef LINEAR_SENSOR_MSGS__MSG__DETAIL__LINEAR_SENSOR_DATA__FUNCTIONS_H_
#define LINEAR_SENSOR_MSGS__MSG__DETAIL__LINEAR_SENSOR_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "linear_sensor_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "linear_sensor_msgs/msg/detail/linear_sensor_data__struct.h"

/// Initialize msg/LinearSensorData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * linear_sensor_msgs__msg__LinearSensorData
 * )) before or use
 * linear_sensor_msgs__msg__LinearSensorData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_linear_sensor_msgs
bool
linear_sensor_msgs__msg__LinearSensorData__init(linear_sensor_msgs__msg__LinearSensorData * msg);

/// Finalize msg/LinearSensorData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_linear_sensor_msgs
void
linear_sensor_msgs__msg__LinearSensorData__fini(linear_sensor_msgs__msg__LinearSensorData * msg);

/// Create msg/LinearSensorData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * linear_sensor_msgs__msg__LinearSensorData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_linear_sensor_msgs
linear_sensor_msgs__msg__LinearSensorData *
linear_sensor_msgs__msg__LinearSensorData__create();

/// Destroy msg/LinearSensorData message.
/**
 * It calls
 * linear_sensor_msgs__msg__LinearSensorData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_linear_sensor_msgs
void
linear_sensor_msgs__msg__LinearSensorData__destroy(linear_sensor_msgs__msg__LinearSensorData * msg);

/// Check for msg/LinearSensorData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_linear_sensor_msgs
bool
linear_sensor_msgs__msg__LinearSensorData__are_equal(const linear_sensor_msgs__msg__LinearSensorData * lhs, const linear_sensor_msgs__msg__LinearSensorData * rhs);

/// Copy a msg/LinearSensorData message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_linear_sensor_msgs
bool
linear_sensor_msgs__msg__LinearSensorData__copy(
  const linear_sensor_msgs__msg__LinearSensorData * input,
  linear_sensor_msgs__msg__LinearSensorData * output);

/// Initialize array of msg/LinearSensorData messages.
/**
 * It allocates the memory for the number of elements and calls
 * linear_sensor_msgs__msg__LinearSensorData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_linear_sensor_msgs
bool
linear_sensor_msgs__msg__LinearSensorData__Sequence__init(linear_sensor_msgs__msg__LinearSensorData__Sequence * array, size_t size);

/// Finalize array of msg/LinearSensorData messages.
/**
 * It calls
 * linear_sensor_msgs__msg__LinearSensorData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_linear_sensor_msgs
void
linear_sensor_msgs__msg__LinearSensorData__Sequence__fini(linear_sensor_msgs__msg__LinearSensorData__Sequence * array);

/// Create array of msg/LinearSensorData messages.
/**
 * It allocates the memory for the array and calls
 * linear_sensor_msgs__msg__LinearSensorData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_linear_sensor_msgs
linear_sensor_msgs__msg__LinearSensorData__Sequence *
linear_sensor_msgs__msg__LinearSensorData__Sequence__create(size_t size);

/// Destroy array of msg/LinearSensorData messages.
/**
 * It calls
 * linear_sensor_msgs__msg__LinearSensorData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_linear_sensor_msgs
void
linear_sensor_msgs__msg__LinearSensorData__Sequence__destroy(linear_sensor_msgs__msg__LinearSensorData__Sequence * array);

/// Check for msg/LinearSensorData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_linear_sensor_msgs
bool
linear_sensor_msgs__msg__LinearSensorData__Sequence__are_equal(const linear_sensor_msgs__msg__LinearSensorData__Sequence * lhs, const linear_sensor_msgs__msg__LinearSensorData__Sequence * rhs);

/// Copy an array of msg/LinearSensorData messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_linear_sensor_msgs
bool
linear_sensor_msgs__msg__LinearSensorData__Sequence__copy(
  const linear_sensor_msgs__msg__LinearSensorData__Sequence * input,
  linear_sensor_msgs__msg__LinearSensorData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // LINEAR_SENSOR_MSGS__MSG__DETAIL__LINEAR_SENSOR_DATA__FUNCTIONS_H_
