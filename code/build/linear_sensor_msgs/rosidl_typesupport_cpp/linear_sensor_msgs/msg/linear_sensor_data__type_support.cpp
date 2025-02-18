// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from linear_sensor_msgs:msg/LinearSensorData.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "linear_sensor_msgs/msg/detail/linear_sensor_data__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace linear_sensor_msgs
{

namespace msg
{

namespace rosidl_typesupport_cpp
{

typedef struct _LinearSensorData_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LinearSensorData_type_support_ids_t;

static const _LinearSensorData_type_support_ids_t _LinearSensorData_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _LinearSensorData_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LinearSensorData_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LinearSensorData_type_support_symbol_names_t _LinearSensorData_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, linear_sensor_msgs, msg, LinearSensorData)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, linear_sensor_msgs, msg, LinearSensorData)),
  }
};

typedef struct _LinearSensorData_type_support_data_t
{
  void * data[2];
} _LinearSensorData_type_support_data_t;

static _LinearSensorData_type_support_data_t _LinearSensorData_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LinearSensorData_message_typesupport_map = {
  2,
  "linear_sensor_msgs",
  &_LinearSensorData_message_typesupport_ids.typesupport_identifier[0],
  &_LinearSensorData_message_typesupport_symbol_names.symbol_name[0],
  &_LinearSensorData_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LinearSensorData_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LinearSensorData_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace msg

}  // namespace linear_sensor_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<linear_sensor_msgs::msg::LinearSensorData>()
{
  return &::linear_sensor_msgs::msg::rosidl_typesupport_cpp::LinearSensorData_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, linear_sensor_msgs, msg, LinearSensorData)() {
  return get_message_type_support_handle<linear_sensor_msgs::msg::LinearSensorData>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp
