// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from olei_interfaces:msg/V2Header.idl
// generated code does not contain a copyright notice
#include "olei_interfaces/msg/detail/v2_header__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "olei_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "olei_interfaces/msg/detail/v2_header__struct.h"
#include "olei_interfaces/msg/detail/v2_header__functions.h"
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


using _V2Header__ros_msg_type = olei_interfaces__msg__V2Header;

static bool _V2Header__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _V2Header__ros_msg_type * ros_message = static_cast<const _V2Header__ros_msg_type *>(untyped_ros_message);
  // Field name: magic
  {
    cdr << ros_message->magic;
  }

  // Field name: version
  {
    cdr << ros_message->version;
  }

  // Field name: distance_ratio
  {
    cdr << ros_message->distance_ratio;
  }

  // Field name: brand
  {
    size_t size = 3;
    auto array_ptr = ros_message->brand;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: vommercial
  {
    size_t size = 12;
    auto array_ptr = ros_message->vommercial;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: internal
  {
    cdr << ros_message->internal;
  }

  // Field name: hardware
  {
    cdr << ros_message->hardware;
  }

  // Field name: software
  {
    cdr << ros_message->software;
  }

  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: scan_frequency
  {
    cdr << ros_message->scan_frequency;
  }

  // Field name: safe_status
  {
    cdr << ros_message->safe_status;
  }

  // Field name: error_status
  {
    cdr << ros_message->error_status;
  }

  // Field name: status_flags
  {
    cdr << ros_message->status_flags;
  }

  return true;
}

static bool _V2Header__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _V2Header__ros_msg_type * ros_message = static_cast<_V2Header__ros_msg_type *>(untyped_ros_message);
  // Field name: magic
  {
    cdr >> ros_message->magic;
  }

  // Field name: version
  {
    cdr >> ros_message->version;
  }

  // Field name: distance_ratio
  {
    cdr >> ros_message->distance_ratio;
  }

  // Field name: brand
  {
    size_t size = 3;
    auto array_ptr = ros_message->brand;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: vommercial
  {
    size_t size = 12;
    auto array_ptr = ros_message->vommercial;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: internal
  {
    cdr >> ros_message->internal;
  }

  // Field name: hardware
  {
    cdr >> ros_message->hardware;
  }

  // Field name: software
  {
    cdr >> ros_message->software;
  }

  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: scan_frequency
  {
    cdr >> ros_message->scan_frequency;
  }

  // Field name: safe_status
  {
    cdr >> ros_message->safe_status;
  }

  // Field name: error_status
  {
    cdr >> ros_message->error_status;
  }

  // Field name: status_flags
  {
    cdr >> ros_message->status_flags;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_olei_interfaces
size_t get_serialized_size_olei_interfaces__msg__V2Header(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _V2Header__ros_msg_type * ros_message = static_cast<const _V2Header__ros_msg_type *>(untyped_ros_message);
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
  // field.name distance_ratio
  {
    size_t item_size = sizeof(ros_message->distance_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name brand
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->brand;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vommercial
  {
    size_t array_size = 12;
    auto array_ptr = ros_message->vommercial;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name internal
  {
    size_t item_size = sizeof(ros_message->internal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name hardware
  {
    size_t item_size = sizeof(ros_message->hardware);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name software
  {
    size_t item_size = sizeof(ros_message->software);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name timestamp
  {
    size_t item_size = sizeof(ros_message->timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name scan_frequency
  {
    size_t item_size = sizeof(ros_message->scan_frequency);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name safe_status
  {
    size_t item_size = sizeof(ros_message->safe_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name error_status
  {
    size_t item_size = sizeof(ros_message->error_status);
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

static uint32_t _V2Header__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_olei_interfaces__msg__V2Header(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_olei_interfaces
size_t max_serialized_size_olei_interfaces__msg__V2Header(
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

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: version
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
  // member: brand
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: vommercial
  {
    size_t array_size = 12;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: internal
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: hardware
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: software
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: timestamp
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
  // member: safe_status
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: error_status
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: status_flags
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _V2Header__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_olei_interfaces__msg__V2Header(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_V2Header = {
  "olei_interfaces::msg",
  "V2Header",
  _V2Header__cdr_serialize,
  _V2Header__cdr_deserialize,
  _V2Header__get_serialized_size,
  _V2Header__max_serialized_size
};

static rosidl_message_type_support_t _V2Header__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_V2Header,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, olei_interfaces, msg, V2Header)() {
  return &_V2Header__type_support;
}

#if defined(__cplusplus)
}
#endif
