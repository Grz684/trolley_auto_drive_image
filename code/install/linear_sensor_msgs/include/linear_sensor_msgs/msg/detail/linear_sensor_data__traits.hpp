// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from linear_sensor_msgs:msg/LinearSensorData.idl
// generated code does not contain a copyright notice

#ifndef LINEAR_SENSOR_MSGS__MSG__DETAIL__LINEAR_SENSOR_DATA__TRAITS_HPP_
#define LINEAR_SENSOR_MSGS__MSG__DETAIL__LINEAR_SENSOR_DATA__TRAITS_HPP_

#include "linear_sensor_msgs/msg/detail/linear_sensor_data__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<linear_sensor_msgs::msg::LinearSensorData>()
{
  return "linear_sensor_msgs::msg::LinearSensorData";
}

template<>
inline const char * name<linear_sensor_msgs::msg::LinearSensorData>()
{
  return "linear_sensor_msgs/msg/LinearSensorData";
}

template<>
struct has_fixed_size<linear_sensor_msgs::msg::LinearSensorData>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<linear_sensor_msgs::msg::LinearSensorData>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<linear_sensor_msgs::msg::LinearSensorData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LINEAR_SENSOR_MSGS__MSG__DETAIL__LINEAR_SENSOR_DATA__TRAITS_HPP_
