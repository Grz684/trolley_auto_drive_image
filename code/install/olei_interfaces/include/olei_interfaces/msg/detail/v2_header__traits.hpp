// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from olei_interfaces:msg/V2Header.idl
// generated code does not contain a copyright notice

#ifndef OLEI_INTERFACES__MSG__DETAIL__V2_HEADER__TRAITS_HPP_
#define OLEI_INTERFACES__MSG__DETAIL__V2_HEADER__TRAITS_HPP_

#include "olei_interfaces/msg/detail/v2_header__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<olei_interfaces::msg::V2Header>()
{
  return "olei_interfaces::msg::V2Header";
}

template<>
inline const char * name<olei_interfaces::msg::V2Header>()
{
  return "olei_interfaces/msg/V2Header";
}

template<>
struct has_fixed_size<olei_interfaces::msg::V2Header>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<olei_interfaces::msg::V2Header>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<olei_interfaces::msg::V2Header>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // OLEI_INTERFACES__MSG__DETAIL__V2_HEADER__TRAITS_HPP_
