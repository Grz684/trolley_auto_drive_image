// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from olei_interfaces:msg/VFHeader.idl
// generated code does not contain a copyright notice
#include "olei_interfaces/msg/detail/vf_header__rosidl_typesupport_fastrtps_cpp.hpp"
#include "olei_interfaces/msg/detail/vf_header__struct.hpp"

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
  const olei_interfaces::msg::VFHeader & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: magic
  cdr << ros_message.magic;
  // Member: version
  cdr << ros_message.version;
  // Member: packet_size
  cdr << ros_message.packet_size;
  // Member: header_size
  cdr << ros_message.header_size;
  // Member: distance_ratio
  cdr << ros_message.distance_ratio;
  // Member: types
  cdr << ros_message.types;
  // Member: scan_number
  cdr << ros_message.scan_number;
  // Member: packet_number
  cdr << ros_message.packet_number;
  // Member: timestamp_decimal
  cdr << ros_message.timestamp_decimal;
  // Member: timestamp_integer
  cdr << ros_message.timestamp_integer;
  // Member: scan_frequency
  cdr << ros_message.scan_frequency;
  // Member: num_points_scan
  cdr << ros_message.num_points_scan;
  // Member: iutput_status
  cdr << ros_message.iutput_status;
  // Member: output_status
  cdr << ros_message.output_status;
  // Member: field_status
  cdr << ros_message.field_status;
  // Member: start_index
  cdr << ros_message.start_index;
  // Member: end_index
  cdr << ros_message.end_index;
  // Member: first_index
  cdr << ros_message.first_index;
  // Member: num_points_packet
  cdr << ros_message.num_points_packet;
  // Member: status_flags
  cdr << ros_message.status_flags;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_olei_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  olei_interfaces::msg::VFHeader & ros_message)
{
  // Member: magic
  cdr >> ros_message.magic;

  // Member: version
  cdr >> ros_message.version;

  // Member: packet_size
  cdr >> ros_message.packet_size;

  // Member: header_size
  cdr >> ros_message.header_size;

  // Member: distance_ratio
  cdr >> ros_message.distance_ratio;

  // Member: types
  cdr >> ros_message.types;

  // Member: scan_number
  cdr >> ros_message.scan_number;

  // Member: packet_number
  cdr >> ros_message.packet_number;

  // Member: timestamp_decimal
  cdr >> ros_message.timestamp_decimal;

  // Member: timestamp_integer
  cdr >> ros_message.timestamp_integer;

  // Member: scan_frequency
  cdr >> ros_message.scan_frequency;

  // Member: num_points_scan
  cdr >> ros_message.num_points_scan;

  // Member: iutput_status
  cdr >> ros_message.iutput_status;

  // Member: output_status
  cdr >> ros_message.output_status;

  // Member: field_status
  cdr >> ros_message.field_status;

  // Member: start_index
  cdr >> ros_message.start_index;

  // Member: end_index
  cdr >> ros_message.end_index;

  // Member: first_index
  cdr >> ros_message.first_index;

  // Member: num_points_packet
  cdr >> ros_message.num_points_packet;

  // Member: status_flags
  cdr >> ros_message.status_flags;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_olei_interfaces
get_serialized_size(
  const olei_interfaces::msg::VFHeader & ros_message,
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
  // Member: packet_size
  {
    size_t item_size = sizeof(ros_message.packet_size);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: header_size
  {
    size_t item_size = sizeof(ros_message.header_size);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: distance_ratio
  {
    size_t item_size = sizeof(ros_message.distance_ratio);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: types
  {
    size_t item_size = sizeof(ros_message.types);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: scan_number
  {
    size_t item_size = sizeof(ros_message.scan_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: packet_number
  {
    size_t item_size = sizeof(ros_message.packet_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: timestamp_decimal
  {
    size_t item_size = sizeof(ros_message.timestamp_decimal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: timestamp_integer
  {
    size_t item_size = sizeof(ros_message.timestamp_integer);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: scan_frequency
  {
    size_t item_size = sizeof(ros_message.scan_frequency);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: num_points_scan
  {
    size_t item_size = sizeof(ros_message.num_points_scan);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: iutput_status
  {
    size_t item_size = sizeof(ros_message.iutput_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: output_status
  {
    size_t item_size = sizeof(ros_message.output_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: field_status
  {
    size_t item_size = sizeof(ros_message.field_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: start_index
  {
    size_t item_size = sizeof(ros_message.start_index);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: end_index
  {
    size_t item_size = sizeof(ros_message.end_index);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: first_index
  {
    size_t item_size = sizeof(ros_message.first_index);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: num_points_packet
  {
    size_t item_size = sizeof(ros_message.num_points_packet);
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
max_serialized_size_VFHeader(
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

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: version
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: packet_size
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: header_size
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

  // Member: types
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: scan_number
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: packet_number
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: timestamp_decimal
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: timestamp_integer
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

  // Member: num_points_scan
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: iutput_status
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: output_status
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: field_status
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: start_index
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: end_index
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: first_index
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: num_points_packet
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: status_flags
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _VFHeader__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const olei_interfaces::msg::VFHeader *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _VFHeader__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<olei_interfaces::msg::VFHeader *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _VFHeader__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const olei_interfaces::msg::VFHeader *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _VFHeader__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_VFHeader(full_bounded, 0);
}

static message_type_support_callbacks_t _VFHeader__callbacks = {
  "olei_interfaces::msg",
  "VFHeader",
  _VFHeader__cdr_serialize,
  _VFHeader__cdr_deserialize,
  _VFHeader__get_serialized_size,
  _VFHeader__max_serialized_size
};

static rosidl_message_type_support_t _VFHeader__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_VFHeader__callbacks,
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
get_message_type_support_handle<olei_interfaces::msg::VFHeader>()
{
  return &olei_interfaces::msg::typesupport_fastrtps_cpp::_VFHeader__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, olei_interfaces, msg, VFHeader)() {
  return &olei_interfaces::msg::typesupport_fastrtps_cpp::_VFHeader__handle;
}

#ifdef __cplusplus
}
#endif
