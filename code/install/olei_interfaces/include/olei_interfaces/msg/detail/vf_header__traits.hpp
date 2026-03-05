// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from olei_interfaces:msg/VFHeader.idl
// generated code does not contain a copyright notice

#ifndef OLEI_INTERFACES__MSG__DETAIL__VF_HEADER__TRAITS_HPP_
#define OLEI_INTERFACES__MSG__DETAIL__VF_HEADER__TRAITS_HPP_

#include "olei_interfaces/msg/detail/vf_header__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<olei_interfaces::msg::VFHeader>()
{
  return "olei_interfaces::msg::VFHeader";
}

template<>
inline const char * name<olei_interfaces::msg::VFHeader>()
{
  return "olei_interfaces/msg/VFHeader";
}

template<>
struct has_fixed_size<olei_interfaces::msg::VFHeader>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<olei_interfaces::msg::VFHeader>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<olei_interfaces::msg::VFHeader>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // OLEI_INTERFACES__MSG__DETAIL__VF_HEADER__TRAITS_HPP_
