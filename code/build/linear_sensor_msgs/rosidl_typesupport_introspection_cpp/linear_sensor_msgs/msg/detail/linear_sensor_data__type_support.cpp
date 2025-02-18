// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from linear_sensor_msgs:msg/LinearSensorData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "linear_sensor_msgs/msg/detail/linear_sensor_data__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace linear_sensor_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void LinearSensorData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) linear_sensor_msgs::msg::LinearSensorData(_init);
}

void LinearSensorData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<linear_sensor_msgs::msg::LinearSensorData *>(message_memory);
  typed_message->~LinearSensorData();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember LinearSensorData_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(linear_sensor_msgs::msg::LinearSensorData, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(linear_sensor_msgs::msg::LinearSensorData, data),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers LinearSensorData_message_members = {
  "linear_sensor_msgs::msg",  // message namespace
  "LinearSensorData",  // message name
  2,  // number of fields
  sizeof(linear_sensor_msgs::msg::LinearSensorData),
  LinearSensorData_message_member_array,  // message members
  LinearSensorData_init_function,  // function to initialize message memory (memory has to be allocated)
  LinearSensorData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t LinearSensorData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &LinearSensorData_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace linear_sensor_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<linear_sensor_msgs::msg::LinearSensorData>()
{
  return &::linear_sensor_msgs::msg::rosidl_typesupport_introspection_cpp::LinearSensorData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, linear_sensor_msgs, msg, LinearSensorData)() {
  return &::linear_sensor_msgs::msg::rosidl_typesupport_introspection_cpp::LinearSensorData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
