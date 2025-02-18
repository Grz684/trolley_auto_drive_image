// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from linear_sensor_msgs:msg/LinearSensorData.idl
// generated code does not contain a copyright notice
#include "linear_sensor_msgs/msg/detail/linear_sensor_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
linear_sensor_msgs__msg__LinearSensorData__init(linear_sensor_msgs__msg__LinearSensorData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    linear_sensor_msgs__msg__LinearSensorData__fini(msg);
    return false;
  }
  // data
  return true;
}

void
linear_sensor_msgs__msg__LinearSensorData__fini(linear_sensor_msgs__msg__LinearSensorData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // data
}

bool
linear_sensor_msgs__msg__LinearSensorData__are_equal(const linear_sensor_msgs__msg__LinearSensorData * lhs, const linear_sensor_msgs__msg__LinearSensorData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // data
  if (lhs->data != rhs->data) {
    return false;
  }
  return true;
}

bool
linear_sensor_msgs__msg__LinearSensorData__copy(
  const linear_sensor_msgs__msg__LinearSensorData * input,
  linear_sensor_msgs__msg__LinearSensorData * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // data
  output->data = input->data;
  return true;
}

linear_sensor_msgs__msg__LinearSensorData *
linear_sensor_msgs__msg__LinearSensorData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  linear_sensor_msgs__msg__LinearSensorData * msg = (linear_sensor_msgs__msg__LinearSensorData *)allocator.allocate(sizeof(linear_sensor_msgs__msg__LinearSensorData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(linear_sensor_msgs__msg__LinearSensorData));
  bool success = linear_sensor_msgs__msg__LinearSensorData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
linear_sensor_msgs__msg__LinearSensorData__destroy(linear_sensor_msgs__msg__LinearSensorData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    linear_sensor_msgs__msg__LinearSensorData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
linear_sensor_msgs__msg__LinearSensorData__Sequence__init(linear_sensor_msgs__msg__LinearSensorData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  linear_sensor_msgs__msg__LinearSensorData * data = NULL;

  if (size) {
    data = (linear_sensor_msgs__msg__LinearSensorData *)allocator.zero_allocate(size, sizeof(linear_sensor_msgs__msg__LinearSensorData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = linear_sensor_msgs__msg__LinearSensorData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        linear_sensor_msgs__msg__LinearSensorData__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
linear_sensor_msgs__msg__LinearSensorData__Sequence__fini(linear_sensor_msgs__msg__LinearSensorData__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      linear_sensor_msgs__msg__LinearSensorData__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

linear_sensor_msgs__msg__LinearSensorData__Sequence *
linear_sensor_msgs__msg__LinearSensorData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  linear_sensor_msgs__msg__LinearSensorData__Sequence * array = (linear_sensor_msgs__msg__LinearSensorData__Sequence *)allocator.allocate(sizeof(linear_sensor_msgs__msg__LinearSensorData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = linear_sensor_msgs__msg__LinearSensorData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
linear_sensor_msgs__msg__LinearSensorData__Sequence__destroy(linear_sensor_msgs__msg__LinearSensorData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    linear_sensor_msgs__msg__LinearSensorData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
linear_sensor_msgs__msg__LinearSensorData__Sequence__are_equal(const linear_sensor_msgs__msg__LinearSensorData__Sequence * lhs, const linear_sensor_msgs__msg__LinearSensorData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!linear_sensor_msgs__msg__LinearSensorData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
linear_sensor_msgs__msg__LinearSensorData__Sequence__copy(
  const linear_sensor_msgs__msg__LinearSensorData__Sequence * input,
  linear_sensor_msgs__msg__LinearSensorData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(linear_sensor_msgs__msg__LinearSensorData);
    linear_sensor_msgs__msg__LinearSensorData * data =
      (linear_sensor_msgs__msg__LinearSensorData *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!linear_sensor_msgs__msg__LinearSensorData__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          linear_sensor_msgs__msg__LinearSensorData__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!linear_sensor_msgs__msg__LinearSensorData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
