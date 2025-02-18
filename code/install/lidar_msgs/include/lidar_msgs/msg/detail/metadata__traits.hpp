// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lidar_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__MSG__DETAIL__METADATA__TRAITS_HPP_
#define LIDAR_MSGS__MSG__DETAIL__METADATA__TRAITS_HPP_

#include "lidar_msgs/msg/detail/metadata__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<lidar_msgs::msg::Metadata>()
{
  return "lidar_msgs::msg::Metadata";
}

template<>
inline const char * name<lidar_msgs::msg::Metadata>()
{
  return "lidar_msgs/msg/Metadata";
}

template<>
struct has_fixed_size<lidar_msgs::msg::Metadata>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<lidar_msgs::msg::Metadata>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<lidar_msgs::msg::Metadata>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LIDAR_MSGS__MSG__DETAIL__METADATA__TRAITS_HPP_
