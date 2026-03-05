// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from olei_interfaces:msg/V2Header.idl
// generated code does not contain a copyright notice
#include "olei_interfaces/msg/detail/v2_header__rosidl_typesupport_fastrtps_cpp.hpp"
#include "olei_interfaces/msg/detail/v2_header__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace olei_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_olei_interfaces
cdr_serialize(
  const olei_interfaces::msg::V2Header & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: magic
  cdr << ros_message.magic;
  // Member: version
  cdr << ros_message.version;
  // Member: distance_ratio
  cdr << ros_message.distance_ratio;
  // Member: brand
  {
    cdr << ros_message.brand;
  }
  // Member: vommercial
  {
    cdr << ros_message.vommercial;
  }
  // Member: internal
  cdr << ros_message.internal;
  // Member: hardware
  cdr << ros_message.hardware;
  // Member: software
  cdr << ros_message.software;
  // Member: timestamp
  cdr << ros_message.timestamp;
  // Member: scan_frequency
  cdr << ros_message.scan_frequency;
  // Member: safe_status
  cdr << ros_message.safe_status;
  // Member: error_status
  cdr << ros_message.error_status;
  // Member: status_flags
  cdr << ros_message.status_flags;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_olei_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  olei_interfaces::msg::V2Header & ros_message)
{
  // Member: magic
  cdr >> ros_message.magic;

  // Member: version
  cdr >> ros_message.version;

  // Member: distance_ratio
  cdr >> ros_message.distance_ratio;

  // Member: brand
  {
    cdr >> ros_message.brand;
  }

  // Member: vommercial
  {
    cdr >> ros_message.vommercial;
  }

  // Member: internal
  cdr >> ros_message.internal;

  // Member: hardware
  cdr >> ros_message.hardware;

  // Member: software
  cdr >> ros_message.software;

  // Member: timestamp
  cdr >> ros_message.timestamp;

  // Member: scan_frequency
  cdr >> ros_message.scan_frequency;

  // Member: safe_status
  cdr >> ros_message.safe_status;

  // Member: error_status
  cdr >> ros_message.error_status;

  // Member: status_flags
  cdr >> ros_message.status_flags;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_olei_interfaces
get_serialized_size(
  const olei_interfaces::msg::V2Header & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: magic
  {
    size_t item_size = sizeof(ros_message.magic);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: version
  {
    size_t item_size = sizeof(ros_message.version);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: distance_ratio
  {
    size_t item_size = sizeof(ros_message.distance_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: brand
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.brand[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: vommercial
  {
    size_t array_size = 12;
    size_t item_size = sizeof(ros_message.vommercial[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: internal
  {
    size_t item_size = sizeof(ros_message.internal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: hardware
  {
    size_t item_size = sizeof(ros_message.hardware);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: software
  {
    size_t item_size = sizeof(ros_message.software);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: timestamp
  {
    size_t item_size = sizeof(ros_message.timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: scan_frequency
  {
    size_t item_size = sizeof(ros_message.scan_frequency);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: safe_status
  {
    size_t item_size = sizeof(ros_message.safe_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: error_status
  {
    size_t item_size = sizeof(ros_message.error_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: status_flags
  {
    size_t item_size = sizeof(ros_message.status_flags);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_olei_interfaces
max_serialized_size_V2Header(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: magic
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: version
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: distance_ratio
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: brand
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: vommercial
  {
    size_t array_size = 12;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: internal
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: hardware
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: software
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: timestamp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: scan_frequency
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: safe_status
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: error_status
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: status_flags
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _V2Header__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const olei_interfaces::msg::V2Header *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _V2Header__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<olei_interfaces::msg::V2Header *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _V2Header__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const olei_interfaces::msg::V2Header *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _V2Header__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_V2Header(full_bounded, 0);
}

static message_type_support_callbacks_t _V2Header__callbacks = {
  "olei_interfaces::msg",
  "V2Header",
  _V2Header__cdr_serialize,
  _V2Header__cdr_deserialize,
  _V2Header__get_serialized_size,
  _V2Header__max_serialized_size
};

static rosidl_message_type_support_t _V2Header__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_V2Header__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace olei_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_olei_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<olei_interfaces::msg::V2Header>()
{
  return &olei_interfaces::msg::typesupport_fastrtps_cpp::_V2Header__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, olei_interfaces, msg, V2Header)() {
  return &olei_interfaces::msg::typesupport_fastrtps_cpp::_V2Header__handle;
}

#ifdef __cplusplus
}
#endif
