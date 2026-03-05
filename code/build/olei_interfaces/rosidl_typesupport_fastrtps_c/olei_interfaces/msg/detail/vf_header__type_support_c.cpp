// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from olei_interfaces:msg/VFHeader.idl
// generated code does not contain a copyright notice
#include "olei_interfaces/msg/detail/vf_header__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "olei_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "olei_interfaces/msg/detail/vf_header__struct.h"
#include "olei_interfaces/msg/detail/vf_header__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _VFHeader__ros_msg_type = olei_interfaces__msg__VFHeader;

static bool _VFHeader__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _VFHeader__ros_msg_type * ros_message = static_cast<const _VFHeader__ros_msg_type *>(untyped_ros_message);
  // Field name: magic
  {
    cdr << ros_message->magic;
  }

  // Field name: version
  {
    cdr << ros_message->version;
  }

  // Field name: packet_size
  {
    cdr << ros_message->packet_size;
  }

  // Field name: header_size
  {
    cdr << ros_message->header_size;
  }

  // Field name: distance_ratio
  {
    cdr << ros_message->distance_ratio;
  }

  // Field name: types
  {
    cdr << ros_message->types;
  }

  // Field name: scan_number
  {
    cdr << ros_message->scan_number;
  }

  // Field name: packet_number
  {
    cdr << ros_message->packet_number;
  }

  // Field name: timestamp_decimal
  {
    cdr << ros_message->timestamp_decimal;
  }

  // Field name: timestamp_integer
  {
    cdr << ros_message->timestamp_integer;
  }

  // Field name: scan_frequency
  {
    cdr << ros_message->scan_frequency;
  }

  // Field name: num_points_scan
  {
    cdr << ros_message->num_points_scan;
  }

  // Field name: iutput_status
  {
    cdr << ros_message->iutput_status;
  }

  // Field name: output_status
  {
    cdr << ros_message->output_status;
  }

  // Field name: field_status
  {
    cdr << ros_message->field_status;
  }

  // Field name: start_index
  {
    cdr << ros_message->start_index;
  }

  // Field name: end_index
  {
    cdr << ros_message->end_index;
  }

  // Field name: first_index
  {
    cdr << ros_message->first_index;
  }

  // Field name: num_points_packet
  {
    cdr << ros_message->num_points_packet;
  }

  // Field name: status_flags
  {
    cdr << ros_message->status_flags;
  }

  return true;
}

static bool _VFHeader__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _VFHeader__ros_msg_type * ros_message = static_cast<_VFHeader__ros_msg_type *>(untyped_ros_message);
  // Field name: magic
  {
    cdr >> ros_message->magic;
  }

  // Field name: version
  {
    cdr >> ros_message->version;
  }

  // Field name: packet_size
  {
    cdr >> ros_message->packet_size;
  }

  // Field name: header_size
  {
    cdr >> ros_message->header_size;
  }

  // Field name: distance_ratio
  {
    cdr >> ros_message->distance_ratio;
  }

  // Field name: types
  {
    cdr >> ros_message->types;
  }

  // Field name: scan_number
  {
    cdr >> ros_message->scan_number;
  }

  // Field name: packet_number
  {
    cdr >> ros_message->packet_number;
  }

  // Field name: timestamp_decimal
  {
    cdr >> ros_message->timestamp_decimal;
  }

  // Field name: timestamp_integer
  {
    cdr >> ros_message->timestamp_integer;
  }

  // Field name: scan_frequency
  {
    cdr >> ros_message->scan_frequency;
  }

  // Field name: num_points_scan
  {
    cdr >> ros_message->num_points_scan;
  }

  // Field name: iutput_status
  {
    cdr >> ros_message->iutput_status;
  }

  // Field name: output_status
  {
    cdr >> ros_message->output_status;
  }

  // Field name: field_status
  {
    cdr >> ros_message->field_status;
  }

  // Field name: start_index
  {
    cdr >> ros_message->start_index;
  }

  // Field name: end_index
  {
    cdr >> ros_message->end_index;
  }

  // Field name: first_index
  {
    cdr >> ros_message->first_index;
  }

  // Field name: num_points_packet
  {
    cdr >> ros_message->num_points_packet;
  }

  // Field name: status_flags
  {
    cdr >> ros_message->status_flags;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_olei_interfaces
size_t get_serialized_size_olei_interfaces__msg__VFHeader(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _VFHeader__ros_msg_type * ros_message = static_cast<const _VFHeader__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name magic
  {
    size_t item_size = sizeof(ros_message->magic);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name version
  {
    size_t item_size = sizeof(ros_message->version);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name packet_size
  {
    size_t item_size = sizeof(ros_message->packet_size);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name header_size
  {
    size_t item_size = sizeof(ros_message->header_size);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name distance_ratio
  {
    size_t item_size = sizeof(ros_message->distance_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name types
  {
    size_t item_size = sizeof(ros_message->types);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name scan_number
  {
    size_t item_size = sizeof(ros_message->scan_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name packet_number
  {
    size_t item_size = sizeof(ros_message->packet_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name timestamp_decimal
  {
    size_t item_size = sizeof(ros_message->timestamp_decimal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name timestamp_integer
  {
    size_t item_size = sizeof(ros_message->timestamp_integer);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name scan_frequency
  {
    size_t item_size = sizeof(ros_message->scan_frequency);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name num_points_scan
  {
    size_t item_size = sizeof(ros_message->num_points_scan);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name iutput_status
  {
    size_t item_size = sizeof(ros_message->iutput_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name output_status
  {
    size_t item_size = sizeof(ros_message->output_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name field_status
  {
    size_t item_size = sizeof(ros_message->field_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name start_index
  {
    size_t item_size = sizeof(ros_message->start_index);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name end_index
  {
    size_t item_size = sizeof(ros_message->end_index);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name first_index
  {
    size_t item_size = sizeof(ros_message->first_index);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name num_points_packet
  {
    size_t item_size = sizeof(ros_message->num_points_packet);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name status_flags
  {
    size_t item_size = sizeof(ros_message->status_flags);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _VFHeader__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_olei_interfaces__msg__VFHeader(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_olei_interfaces
size_t max_serialized_size_olei_interfaces__msg__VFHeader(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: magic
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: version
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: packet_size
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: header_size
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: distance_ratio
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: types
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: scan_number
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: packet_number
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: timestamp_decimal
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: timestamp_integer
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: scan_frequency
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: num_points_scan
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: iutput_status
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: output_status
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: field_status
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: start_index
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: end_index
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: first_index
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: num_points_packet
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: status_flags
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _VFHeader__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_olei_interfaces__msg__VFHeader(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_VFHeader = {
  "olei_interfaces::msg",
  "VFHeader",
  _VFHeader__cdr_serialize,
  _VFHeader__cdr_deserialize,
  _VFHeader__get_serialized_size,
  _VFHeader__max_serialized_size
};

static rosidl_message_type_support_t _VFHeader__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_VFHeader,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, olei_interfaces, msg, VFHeader)() {
  return &_VFHeader__type_support;
}

#if defined(__cplusplus)
}
#endif
