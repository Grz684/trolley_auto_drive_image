// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from linear_sensor_msgs:msg/LinearSensorData.idl
// generated code does not contain a copyright notice
#include "linear_sensor_msgs/msg/detail/linear_sensor_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "linear_sensor_msgs/msg/detail/linear_sensor_data__struct.hpp"

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
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace linear_sensor_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_linear_sensor_msgs
cdr_serialize(
  const linear_sensor_msgs::msg::LinearSensorData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: data
  cdr << ros_message.data;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_linear_sensor_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  linear_sensor_msgs::msg::LinearSensorData & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: data
  cdr >> ros_message.data;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_linear_sensor_msgs
get_serialized_size(
  const linear_sensor_msgs::msg::LinearSensorData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: data
  {
    size_t item_size = sizeof(ros_message.data);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_linear_sensor_msgs
max_serialized_size_LinearSensorData(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        full_bounded, current_alignment);
    }
  }

  // Member: data
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _LinearSensorData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const linear_sensor_msgs::msg::LinearSensorData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _LinearSensorData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<linear_sensor_msgs::msg::LinearSensorData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _LinearSensorData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const linear_sensor_msgs::msg::LinearSensorData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _LinearSensorData__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_LinearSensorData(full_bounded, 0);
}

static message_type_support_callbacks_t _LinearSensorData__callbacks = {
  "linear_sensor_msgs::msg",
  "LinearSensorData",
  _LinearSensorData__cdr_serialize,
  _LinearSensorData__cdr_deserialize,
  _LinearSensorData__get_serialized_size,
  _LinearSensorData__max_serialized_size
};

static rosidl_message_type_support_t _LinearSensorData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_LinearSensorData__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace linear_sensor_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_linear_sensor_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<linear_sensor_msgs::msg::LinearSensorData>()
{
  return &linear_sensor_msgs::msg::typesupport_fastrtps_cpp::_LinearSensorData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, linear_sensor_msgs, msg, LinearSensorData)() {
  return &linear_sensor_msgs::msg::typesupport_fastrtps_cpp::_LinearSensorData__handle;
}

#ifdef __cplusplus
}
#endif
