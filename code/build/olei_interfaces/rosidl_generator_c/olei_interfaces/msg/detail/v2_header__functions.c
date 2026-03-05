// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from olei_interfaces:msg/V2Header.idl
// generated code does not contain a copyright notice
#include "olei_interfaces/msg/detail/v2_header__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
olei_interfaces__msg__V2Header__init(olei_interfaces__msg__V2Header * msg)
{
  if (!msg) {
    return false;
  }
  // magic
  // version
  // distance_ratio
  // brand
  // vommercial
  // internal
  // hardware
  // software
  // timestamp
  // scan_frequency
  // safe_status
  // error_status
  // status_flags
  return true;
}

void
olei_interfaces__msg__V2Header__fini(olei_interfaces__msg__V2Header * msg)
{
  if (!msg) {
    return;
  }
  // magic
  // version
  // distance_ratio
  // brand
  // vommercial
  // internal
  // hardware
  // software
  // timestamp
  // scan_frequency
  // safe_status
  // error_status
  // status_flags
}

bool
olei_interfaces__msg__V2Header__are_equal(const olei_interfaces__msg__V2Header * lhs, const olei_interfaces__msg__V2Header * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // magic
  if (lhs->magic != rhs->magic) {
    return false;
  }
  // version
  if (lhs->version != rhs->version) {
    return false;
  }
  // distance_ratio
  if (lhs->distance_ratio != rhs->distance_ratio) {
    return false;
  }
  // brand
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->brand[i] != rhs->brand[i]) {
      return false;
    }
  }
  // vommercial
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->vommercial[i] != rhs->vommercial[i]) {
      return false;
    }
  }
  // internal
  if (lhs->internal != rhs->internal) {
    return false;
  }
  // hardware
  if (lhs->hardware != rhs->hardware) {
    return false;
  }
  // software
  if (lhs->software != rhs->software) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // scan_frequency
  if (lhs->scan_frequency != rhs->scan_frequency) {
    return false;
  }
  // safe_status
  if (lhs->safe_status != rhs->safe_status) {
    return false;
  }
  // error_status
  if (lhs->error_status != rhs->error_status) {
    return false;
  }
  // status_flags
  if (lhs->status_flags != rhs->status_flags) {
    return false;
  }
  return true;
}

bool
olei_interfaces__msg__V2Header__copy(
  const olei_interfaces__msg__V2Header * input,
  olei_interfaces__msg__V2Header * output)
{
  if (!input || !output) {
    return false;
  }
  // magic
  output->magic = input->magic;
  // version
  output->version = input->version;
  // distance_ratio
  output->distance_ratio = input->distance_ratio;
  // brand
  for (size_t i = 0; i < 3; ++i) {
    output->brand[i] = input->brand[i];
  }
  // vommercial
  for (size_t i = 0; i < 12; ++i) {
    output->vommercial[i] = input->vommercial[i];
  }
  // internal
  output->internal = input->internal;
  // hardware
  output->hardware = input->hardware;
  // software
  output->software = input->software;
  // timestamp
  output->timestamp = input->timestamp;
  // scan_frequency
  output->scan_frequency = input->scan_frequency;
  // safe_status
  output->safe_status = input->safe_status;
  // error_status
  output->error_status = input->error_status;
  // status_flags
  output->status_flags = input->status_flags;
  return true;
}

olei_interfaces__msg__V2Header *
olei_interfaces__msg__V2Header__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  olei_interfaces__msg__V2Header * msg = (olei_interfaces__msg__V2Header *)allocator.allocate(sizeof(olei_interfaces__msg__V2Header), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(olei_interfaces__msg__V2Header));
  bool success = olei_interfaces__msg__V2Header__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
olei_interfaces__msg__V2Header__destroy(olei_interfaces__msg__V2Header * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    olei_interfaces__msg__V2Header__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
olei_interfaces__msg__V2Header__Sequence__init(olei_interfaces__msg__V2Header__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  olei_interfaces__msg__V2Header * data = NULL;

  if (size) {
    data = (olei_interfaces__msg__V2Header *)allocator.zero_allocate(size, sizeof(olei_interfaces__msg__V2Header), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = olei_interfaces__msg__V2Header__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        olei_interfaces__msg__V2Header__fini(&data[i - 1]);
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
olei_interfaces__msg__V2Header__Sequence__fini(olei_interfaces__msg__V2Header__Sequence * array)
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
      olei_interfaces__msg__V2Header__fini(&array->data[i]);
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

olei_interfaces__msg__V2Header__Sequence *
olei_interfaces__msg__V2Header__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  olei_interfaces__msg__V2Header__Sequence * array = (olei_interfaces__msg__V2Header__Sequence *)allocator.allocate(sizeof(olei_interfaces__msg__V2Header__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = olei_interfaces__msg__V2Header__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
olei_interfaces__msg__V2Header__Sequence__destroy(olei_interfaces__msg__V2Header__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    olei_interfaces__msg__V2Header__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
olei_interfaces__msg__V2Header__Sequence__are_equal(const olei_interfaces__msg__V2Header__Sequence * lhs, const olei_interfaces__msg__V2Header__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!olei_interfaces__msg__V2Header__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
olei_interfaces__msg__V2Header__Sequence__copy(
  const olei_interfaces__msg__V2Header__Sequence * input,
  olei_interfaces__msg__V2Header__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(olei_interfaces__msg__V2Header);
    olei_interfaces__msg__V2Header * data =
      (olei_interfaces__msg__V2Header *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!olei_interfaces__msg__V2Header__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          olei_interfaces__msg__V2Header__fini(&data[i]);
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
    if (!olei_interfaces__msg__V2Header__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
