// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lidar_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice
#include "lidar_msgs/msg/detail/metadata__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `computer_ip`
// Member `lidar_ip`
#include "rosidl_runtime_c/string_functions.h"

bool
lidar_msgs__msg__Metadata__init(lidar_msgs__msg__Metadata * msg)
{
  if (!msg) {
    return false;
  }
  // computer_ip
  if (!rosidl_runtime_c__String__init(&msg->computer_ip)) {
    lidar_msgs__msg__Metadata__fini(msg);
    return false;
  }
  // lidar_ip
  if (!rosidl_runtime_c__String__init(&msg->lidar_ip)) {
    lidar_msgs__msg__Metadata__fini(msg);
    return false;
  }
  // imu_port
  // lidar_port
  return true;
}

void
lidar_msgs__msg__Metadata__fini(lidar_msgs__msg__Metadata * msg)
{
  if (!msg) {
    return;
  }
  // computer_ip
  rosidl_runtime_c__String__fini(&msg->computer_ip);
  // lidar_ip
  rosidl_runtime_c__String__fini(&msg->lidar_ip);
  // imu_port
  // lidar_port
}

bool
lidar_msgs__msg__Metadata__are_equal(const lidar_msgs__msg__Metadata * lhs, const lidar_msgs__msg__Metadata * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // computer_ip
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->computer_ip), &(rhs->computer_ip)))
  {
    return false;
  }
  // lidar_ip
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->lidar_ip), &(rhs->lidar_ip)))
  {
    return false;
  }
  // imu_port
  if (lhs->imu_port != rhs->imu_port) {
    return false;
  }
  // lidar_port
  if (lhs->lidar_port != rhs->lidar_port) {
    return false;
  }
  return true;
}

bool
lidar_msgs__msg__Metadata__copy(
  const lidar_msgs__msg__Metadata * input,
  lidar_msgs__msg__Metadata * output)
{
  if (!input || !output) {
    return false;
  }
  // computer_ip
  if (!rosidl_runtime_c__String__copy(
      &(input->computer_ip), &(output->computer_ip)))
  {
    return false;
  }
  // lidar_ip
  if (!rosidl_runtime_c__String__copy(
      &(input->lidar_ip), &(output->lidar_ip)))
  {
    return false;
  }
  // imu_port
  output->imu_port = input->imu_port;
  // lidar_port
  output->lidar_port = input->lidar_port;
  return true;
}

lidar_msgs__msg__Metadata *
lidar_msgs__msg__Metadata__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_msgs__msg__Metadata * msg = (lidar_msgs__msg__Metadata *)allocator.allocate(sizeof(lidar_msgs__msg__Metadata), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lidar_msgs__msg__Metadata));
  bool success = lidar_msgs__msg__Metadata__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lidar_msgs__msg__Metadata__destroy(lidar_msgs__msg__Metadata * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lidar_msgs__msg__Metadata__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lidar_msgs__msg__Metadata__Sequence__init(lidar_msgs__msg__Metadata__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_msgs__msg__Metadata * data = NULL;

  if (size) {
    data = (lidar_msgs__msg__Metadata *)allocator.zero_allocate(size, sizeof(lidar_msgs__msg__Metadata), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lidar_msgs__msg__Metadata__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lidar_msgs__msg__Metadata__fini(&data[i - 1]);
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
lidar_msgs__msg__Metadata__Sequence__fini(lidar_msgs__msg__Metadata__Sequence * array)
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
      lidar_msgs__msg__Metadata__fini(&array->data[i]);
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

lidar_msgs__msg__Metadata__Sequence *
lidar_msgs__msg__Metadata__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_msgs__msg__Metadata__Sequence * array = (lidar_msgs__msg__Metadata__Sequence *)allocator.allocate(sizeof(lidar_msgs__msg__Metadata__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lidar_msgs__msg__Metadata__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lidar_msgs__msg__Metadata__Sequence__destroy(lidar_msgs__msg__Metadata__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lidar_msgs__msg__Metadata__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lidar_msgs__msg__Metadata__Sequence__are_equal(const lidar_msgs__msg__Metadata__Sequence * lhs, const lidar_msgs__msg__Metadata__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lidar_msgs__msg__Metadata__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lidar_msgs__msg__Metadata__Sequence__copy(
  const lidar_msgs__msg__Metadata__Sequence * input,
  lidar_msgs__msg__Metadata__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lidar_msgs__msg__Metadata);
    lidar_msgs__msg__Metadata * data =
      (lidar_msgs__msg__Metadata *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lidar_msgs__msg__Metadata__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          lidar_msgs__msg__Metadata__fini(&data[i]);
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
    if (!lidar_msgs__msg__Metadata__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
