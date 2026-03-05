// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from olei_interfaces:msg/V2Header.idl
// generated code does not contain a copyright notice

#ifndef OLEI_INTERFACES__MSG__DETAIL__V2_HEADER__BUILDER_HPP_
#define OLEI_INTERFACES__MSG__DETAIL__V2_HEADER__BUILDER_HPP_

#include "olei_interfaces/msg/detail/v2_header__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace olei_interfaces
{

namespace msg
{

namespace builder
{

class Init_V2Header_status_flags
{
public:
  explicit Init_V2Header_status_flags(::olei_interfaces::msg::V2Header & msg)
  : msg_(msg)
  {}
  ::olei_interfaces::msg::V2Header status_flags(::olei_interfaces::msg::V2Header::_status_flags_type arg)
  {
    msg_.status_flags = std::move(arg);
    return std::move(msg_);
  }

private:
  ::olei_interfaces::msg::V2Header msg_;
};

class Init_V2Header_error_status
{
public:
  explicit Init_V2Header_error_status(::olei_interfaces::msg::V2Header & msg)
  : msg_(msg)
  {}
  Init_V2Header_status_flags error_status(::olei_interfaces::msg::V2Header::_error_status_type arg)
  {
    msg_.error_status = std::move(arg);
    return Init_V2Header_status_flags(msg_);
  }

private:
  ::olei_interfaces::msg::V2Header msg_;
};

class Init_V2Header_safe_status
{
public:
  explicit Init_V2Header_safe_status(::olei_interfaces::msg::V2Header & msg)
  : msg_(msg)
  {}
  Init_V2Header_error_status safe_status(::olei_interfaces::msg::V2Header::_safe_status_type arg)
  {
    msg_.safe_status = std::move(arg);
    return Init_V2Header_error_status(msg_);
  }

private:
  ::olei_interfaces::msg::V2Header msg_;
};

class Init_V2Header_scan_frequency
{
public:
  explicit Init_V2Header_scan_frequency(::olei_interfaces::msg::V2Header & msg)
  : msg_(msg)
  {}
  Init_V2Header_safe_status scan_frequency(::olei_interfaces::msg::V2Header::_scan_frequency_type arg)
  {
    msg_.scan_frequency = std::move(arg);
    return Init_V2Header_safe_status(msg_);
  }

private:
  ::olei_interfaces::msg::V2Header msg_;
};

class Init_V2Header_timestamp
{
public:
  explicit Init_V2Header_timestamp(::olei_interfaces::msg::V2Header & msg)
  : msg_(msg)
  {}
  Init_V2Header_scan_frequency timestamp(::olei_interfaces::msg::V2Header::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_V2Header_scan_frequency(msg_);
  }

private:
  ::olei_interfaces::msg::V2Header msg_;
};

class Init_V2Header_software
{
public:
  explicit Init_V2Header_software(::olei_interfaces::msg::V2Header & msg)
  : msg_(msg)
  {}
  Init_V2Header_timestamp software(::olei_interfaces::msg::V2Header::_software_type arg)
  {
    msg_.software = std::move(arg);
    return Init_V2Header_timestamp(msg_);
  }

private:
  ::olei_interfaces::msg::V2Header msg_;
};

class Init_V2Header_hardware
{
public:
  explicit Init_V2Header_hardware(::olei_interfaces::msg::V2Header & msg)
  : msg_(msg)
  {}
  Init_V2Header_software hardware(::olei_interfaces::msg::V2Header::_hardware_type arg)
  {
    msg_.hardware = std::move(arg);
    return Init_V2Header_software(msg_);
  }

private:
  ::olei_interfaces::msg::V2Header msg_;
};

class Init_V2Header_internal
{
public:
  explicit Init_V2Header_internal(::olei_interfaces::msg::V2Header & msg)
  : msg_(msg)
  {}
  Init_V2Header_hardware internal(::olei_interfaces::msg::V2Header::_internal_type arg)
  {
    msg_.internal = std::move(arg);
    return Init_V2Header_hardware(msg_);
  }

private:
  ::olei_interfaces::msg::V2Header msg_;
};

class Init_V2Header_vommercial
{
public:
  explicit Init_V2Header_vommercial(::olei_interfaces::msg::V2Header & msg)
  : msg_(msg)
  {}
  Init_V2Header_internal vommercial(::olei_interfaces::msg::V2Header::_vommercial_type arg)
  {
    msg_.vommercial = std::move(arg);
    return Init_V2Header_internal(msg_);
  }

private:
  ::olei_interfaces::msg::V2Header msg_;
};

class Init_V2Header_brand
{
public:
  explicit Init_V2Header_brand(::olei_interfaces::msg::V2Header & msg)
  : msg_(msg)
  {}
  Init_V2Header_vommercial brand(::olei_interfaces::msg::V2Header::_brand_type arg)
  {
    msg_.brand = std::move(arg);
    return Init_V2Header_vommercial(msg_);
  }

private:
  ::olei_interfaces::msg::V2Header msg_;
};

class Init_V2Header_distance_ratio
{
public:
  explicit Init_V2Header_distance_ratio(::olei_interfaces::msg::V2Header & msg)
  : msg_(msg)
  {}
  Init_V2Header_brand distance_ratio(::olei_interfaces::msg::V2Header::_distance_ratio_type arg)
  {
    msg_.distance_ratio = std::move(arg);
    return Init_V2Header_brand(msg_);
  }

private:
  ::olei_interfaces::msg::V2Header msg_;
};

class Init_V2Header_version
{
public:
  explicit Init_V2Header_version(::olei_interfaces::msg::V2Header & msg)
  : msg_(msg)
  {}
  Init_V2Header_distance_ratio version(::olei_interfaces::msg::V2Header::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_V2Header_distance_ratio(msg_);
  }

private:
  ::olei_interfaces::msg::V2Header msg_;
};

class Init_V2Header_magic
{
public:
  Init_V2Header_magic()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_V2Header_version magic(::olei_interfaces::msg::V2Header::_magic_type arg)
  {
    msg_.magic = std::move(arg);
    return Init_V2Header_version(msg_);
  }

private:
  ::olei_interfaces::msg::V2Header msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::olei_interfaces::msg::V2Header>()
{
  return olei_interfaces::msg::builder::Init_V2Header_magic();
}

}  // namespace olei_interfaces

#endif  // OLEI_INTERFACES__MSG__DETAIL__V2_HEADER__BUILDER_HPP_
