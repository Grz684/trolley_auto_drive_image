// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from olei_interfaces:msg/VFHeader.idl
// generated code does not contain a copyright notice
#include "olei_interfaces/msg/detail/vf_header__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
olei_interfaces__msg__VFHeader__init(olei_interfaces__msg__VFHeader * msg)
{
  if (!msg) {
    return false;
  }
  // magic
  // version
  // packet_size
  // header_size
  // distance_ratio
  // types
  // scan_number
  // packet_number
  // timestamp_decimal
  // timestamp_integer
  // scan_frequency
  // num_points_scan
  // iutput_status
  // output_status
  // field_status
  // start_index
  // end_index
  // first_index
  // num_points_packet
  // status_flags
  return true;
}

void
olei_interfaces__msg__VFHeader__fini(olei_interfaces__msg__VFHeader * msg)
{
  if (!msg) {
    return;
  }
  // magic
  // version
  // packet_size
  // header_size
  // distance_ratio
  // types
  // scan_number
  // packet_number
  // timestamp_decimal
  // timestamp_integer
  // scan_frequency
  // num_points_scan
  // iutput_status
  // output_status
  // field_status
  // start_index
  // end_index
  // first_index
  // num_points_packet
  // status_flags
}

bool
olei_interfaces__msg__VFHeader__are_equal(const olei_interfaces__msg__VFHeader * lhs, const olei_interfaces__msg__VFHeader * rhs)
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
  // packet_size
  if (lhs->packet_size != rhs->packet_size) {
    return false;
  }
  // header_size
  if (lhs->header_size != rhs->header_size) {
    return false;
  }
  // distance_ratio
  if (lhs->distance_ratio != rhs->distance_ratio) {
    return false;
  }
  // types
  if (lhs->types != rhs->types) {
    return false;
  }
  // scan_number
  if (lhs->scan_number != rhs->scan_number) {
    return false;
  }
  // packet_number
  if (lhs->packet_number != rhs->packet_number) {
    return false;
  }
  // timestamp_decimal
  if (lhs->timestamp_decimal != rhs->timestamp_decimal) {
    return false;
  }
  // timestamp_integer
  if (lhs->timestamp_integer != rhs->timestamp_integer) {
    return false;
  }
  // scan_frequency
  if (lhs->scan_frequency != rhs->scan_frequency) {
    return false;
  }
  // num_points_scan
  if (lhs->num_points_scan != rhs->num_points_scan) {
    return false;
  }
  // iutput_status
  if (lhs->iutput_status != rhs->iutput_status) {
    return false;
  }
  // output_status
  if (lhs->output_status != rhs->output_status) {
    return false;
  }
  // field_status
  if (lhs->field_status != rhs->field_status) {
    return false;
  }
  // start_index
  if (lhs->start_index != rhs->start_index) {
    return false;
  }
  // end_index
  if (lhs->end_index != rhs->end_index) {
    return false;
  }
  // first_index
  if (lhs->first_index != rhs->first_index) {
    return false;
  }
  // num_points_packet
  if (lhs->num_points_packet != rhs->num_points_packet) {
    return false;
  }
  // status_flags
  if (lhs->status_flags != rhs->status_flags) {
    return false;
  }
  return true;
}

bool
olei_interfaces__msg__VFHeader__copy(
  const olei_interfaces__msg__VFHeader * input,
  olei_interfaces__msg__VFHeader * output)
{
  if (!input || !output) {
    return false;
  }
  // magic
  output->magic = input->magic;
  // version
  output->version = input->version;
  // packet_size
  output->packet_size = input->packet_size;
  // header_size
  output->header_size = input->header_size;
  // distance_ratio
  output->distance_ratio = input->distance_ratio;
  // types
  output->types = input->types;
  // scan_number
  output->scan_number = input->scan_number;
  // packet_number
  output->packet_number = input->packet_number;
  // timestamp_decimal
  output->timestamp_decimal = input->timestamp_decimal;
  // timestamp_integer
  output->timestamp_integer = input->timestamp_integer;
  // scan_frequency
  output->scan_frequency = input->scan_frequency;
  // num_points_scan
  output->num_points_scan = input->num_points_scan;
  // iutput_status
  output->iutput_status = input->iutput_status;
  // output_status
  output->output_status = input->output_status;
  // field_status
  output->field_status = input->field_status;
  // start_index
  output->start_index = input->start_index;
  // end_index
  output->end_index = input->end_index;
  // first_index
  output->first_index = input->first_index;
  // num_points_packet
  output->num_points_packet = input->num_points_packet;
  // status_flags
  output->status_flags = input->status_flags;
  return true;
}

olei_interfaces__msg__VFHeader *
olei_interfaces__msg__VFHeader__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  olei_interfaces__msg__VFHeader * msg = (olei_interfaces__msg__VFHeader *)allocator.allocate(sizeof(olei_interfaces__msg__VFHeader), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(olei_interfaces__msg__VFHeader));
  bool success = olei_interfaces__msg__VFHeader__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
olei_interfaces__msg__VFHeader__destroy(olei_interfaces__msg__VFHeader * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    olei_interfaces__msg__VFHeader__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
olei_interfaces__msg__VFHeader__Sequence__init(olei_interfaces__msg__VFHeader__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  olei_interfaces__msg__VFHeader * data = NULL;

  if (size) {
    data = (olei_interfaces__msg__VFHeader *)allocator.zero_allocate(size, sizeof(olei_interfaces__msg__VFHeader), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = olei_interfaces__msg__VFHeader__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        olei_interfaces__msg__VFHeader__fini(&data[i - 1]);
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
olei_interfaces__msg__VFHeader__Sequence__fini(olei_interfaces__msg__VFHeader__Sequence * array)
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
      olei_interfaces__msg__VFHeader__fini(&array->data[i]);
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

olei_interfaces__msg__VFHeader__Sequence *
olei_interfaces__msg__VFHeader__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  olei_interfaces__msg__VFHeader__Sequence * array = (olei_interfaces__msg__VFHeader__Sequence *)allocator.allocate(sizeof(olei_interfaces__msg__VFHeader__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = olei_interfaces__msg__VFHeader__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
olei_interfaces__msg__VFHeader__Sequence__destroy(olei_interfaces__msg__VFHeader__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    olei_interfaces__msg__VFHeader__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
olei_interfaces__msg__VFHeader__Sequence__are_equal(const olei_interfaces__msg__VFHeader__Sequence * lhs, const olei_interfaces__msg__VFHeader__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!olei_interfaces__msg__VFHeader__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
olei_interfaces__msg__VFHeader__Sequence__copy(
  const olei_interfaces__msg__VFHeader__Sequence * input,
  olei_interfaces__msg__VFHeader__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(olei_interfaces__msg__VFHeader);
    olei_interfaces__msg__VFHeader * data =
      (olei_interfaces__msg__VFHeader *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!olei_interfaces__msg__VFHeader__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          olei_interfaces__msg__VFHeader__fini(&data[i]);
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
    if (!olei_interfaces__msg__VFHeader__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
