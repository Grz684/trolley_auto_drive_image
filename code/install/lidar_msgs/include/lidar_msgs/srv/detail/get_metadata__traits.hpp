// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lidar_msgs:srv/GetMetadata.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__SRV__DETAIL__GET_METADATA__TRAITS_HPP_
#define LIDAR_MSGS__SRV__DETAIL__GET_METADATA__TRAITS_HPP_

#include "lidar_msgs/srv/detail/get_metadata__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<lidar_msgs::srv::GetMetadata_Request>()
{
  return "lidar_msgs::srv::GetMetadata_Request";
}

template<>
inline const char * name<lidar_msgs::srv::GetMetadata_Request>()
{
  return "lidar_msgs/srv/GetMetadata_Request";
}

template<>
struct has_fixed_size<lidar_msgs::srv::GetMetadata_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<lidar_msgs::srv::GetMetadata_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<lidar_msgs::srv::GetMetadata_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'metadata'
#include "lidar_msgs/msg/detail/metadata__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<lidar_msgs::srv::GetMetadata_Response>()
{
  return "lidar_msgs::srv::GetMetadata_Response";
}

template<>
inline const char * name<lidar_msgs::srv::GetMetadata_Response>()
{
  return "lidar_msgs/srv/GetMetadata_Response";
}

template<>
struct has_fixed_size<lidar_msgs::srv::GetMetadata_Response>
  : std::integral_constant<bool, has_fixed_size<lidar_msgs::msg::Metadata>::value> {};

template<>
struct has_bounded_size<lidar_msgs::srv::GetMetadata_Response>
  : std::integral_constant<bool, has_bounded_size<lidar_msgs::msg::Metadata>::value> {};

template<>
struct is_message<lidar_msgs::srv::GetMetadata_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<lidar_msgs::srv::GetMetadata>()
{
  return "lidar_msgs::srv::GetMetadata";
}

template<>
inline const char * name<lidar_msgs::srv::GetMetadata>()
{
  return "lidar_msgs/srv/GetMetadata";
}

template<>
struct has_fixed_size<lidar_msgs::srv::GetMetadata>
  : std::integral_constant<
    bool,
    has_fixed_size<lidar_msgs::srv::GetMetadata_Request>::value &&
    has_fixed_size<lidar_msgs::srv::GetMetadata_Response>::value
  >
{
};

template<>
struct has_bounded_size<lidar_msgs::srv::GetMetadata>
  : std::integral_constant<
    bool,
    has_bounded_size<lidar_msgs::srv::GetMetadata_Request>::value &&
    has_bounded_size<lidar_msgs::srv::GetMetadata_Response>::value
  >
{
};

template<>
struct is_service<lidar_msgs::srv::GetMetadata>
  : std::true_type
{
};

template<>
struct is_service_request<lidar_msgs::srv::GetMetadata_Request>
  : std::true_type
{
};

template<>
struct is_service_response<lidar_msgs::srv::GetMetadata_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // LIDAR_MSGS__SRV__DETAIL__GET_METADATA__TRAITS_HPP_
