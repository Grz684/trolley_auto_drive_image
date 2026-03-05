// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from olei_interfaces:msg/VFHeader.idl
// generated code does not contain a copyright notice

#ifndef OLEI_INTERFACES__MSG__DETAIL__VF_HEADER__BUILDER_HPP_
#define OLEI_INTERFACES__MSG__DETAIL__VF_HEADER__BUILDER_HPP_

#include "olei_interfaces/msg/detail/vf_header__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace olei_interfaces
{

namespace msg
{

namespace builder
{

class Init_VFHeader_status_flags
{
public:
  explicit Init_VFHeader_status_flags(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  ::olei_interfaces::msg::VFHeader status_flags(::olei_interfaces::msg::VFHeader::_status_flags_type arg)
  {
    msg_.status_flags = std::move(arg);
    return std::move(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_num_points_packet
{
public:
  explicit Init_VFHeader_num_points_packet(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_status_flags num_points_packet(::olei_interfaces::msg::VFHeader::_num_points_packet_type arg)
  {
    msg_.num_points_packet = std::move(arg);
    return Init_VFHeader_status_flags(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_first_index
{
public:
  explicit Init_VFHeader_first_index(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_num_points_packet first_index(::olei_interfaces::msg::VFHeader::_first_index_type arg)
  {
    msg_.first_index = std::move(arg);
    return Init_VFHeader_num_points_packet(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_end_index
{
public:
  explicit Init_VFHeader_end_index(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_first_index end_index(::olei_interfaces::msg::VFHeader::_end_index_type arg)
  {
    msg_.end_index = std::move(arg);
    return Init_VFHeader_first_index(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_start_index
{
public:
  explicit Init_VFHeader_start_index(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_end_index start_index(::olei_interfaces::msg::VFHeader::_start_index_type arg)
  {
    msg_.start_index = std::move(arg);
    return Init_VFHeader_end_index(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_field_status
{
public:
  explicit Init_VFHeader_field_status(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_start_index field_status(::olei_interfaces::msg::VFHeader::_field_status_type arg)
  {
    msg_.field_status = std::move(arg);
    return Init_VFHeader_start_index(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_output_status
{
public:
  explicit Init_VFHeader_output_status(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_field_status output_status(::olei_interfaces::msg::VFHeader::_output_status_type arg)
  {
    msg_.output_status = std::move(arg);
    return Init_VFHeader_field_status(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_iutput_status
{
public:
  explicit Init_VFHeader_iutput_status(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_output_status iutput_status(::olei_interfaces::msg::VFHeader::_iutput_status_type arg)
  {
    msg_.iutput_status = std::move(arg);
    return Init_VFHeader_output_status(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_num_points_scan
{
public:
  explicit Init_VFHeader_num_points_scan(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_iutput_status num_points_scan(::olei_interfaces::msg::VFHeader::_num_points_scan_type arg)
  {
    msg_.num_points_scan = std::move(arg);
    return Init_VFHeader_iutput_status(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_scan_frequency
{
public:
  explicit Init_VFHeader_scan_frequency(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_num_points_scan scan_frequency(::olei_interfaces::msg::VFHeader::_scan_frequency_type arg)
  {
    msg_.scan_frequency = std::move(arg);
    return Init_VFHeader_num_points_scan(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_timestamp_integer
{
public:
  explicit Init_VFHeader_timestamp_integer(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_scan_frequency timestamp_integer(::olei_interfaces::msg::VFHeader::_timestamp_integer_type arg)
  {
    msg_.timestamp_integer = std::move(arg);
    return Init_VFHeader_scan_frequency(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_timestamp_decimal
{
public:
  explicit Init_VFHeader_timestamp_decimal(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_timestamp_integer timestamp_decimal(::olei_interfaces::msg::VFHeader::_timestamp_decimal_type arg)
  {
    msg_.timestamp_decimal = std::move(arg);
    return Init_VFHeader_timestamp_integer(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_packet_number
{
public:
  explicit Init_VFHeader_packet_number(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_timestamp_decimal packet_number(::olei_interfaces::msg::VFHeader::_packet_number_type arg)
  {
    msg_.packet_number = std::move(arg);
    return Init_VFHeader_timestamp_decimal(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_scan_number
{
public:
  explicit Init_VFHeader_scan_number(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_packet_number scan_number(::olei_interfaces::msg::VFHeader::_scan_number_type arg)
  {
    msg_.scan_number = std::move(arg);
    return Init_VFHeader_packet_number(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_types
{
public:
  explicit Init_VFHeader_types(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_scan_number types(::olei_interfaces::msg::VFHeader::_types_type arg)
  {
    msg_.types = std::move(arg);
    return Init_VFHeader_scan_number(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_distance_ratio
{
public:
  explicit Init_VFHeader_distance_ratio(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_types distance_ratio(::olei_interfaces::msg::VFHeader::_distance_ratio_type arg)
  {
    msg_.distance_ratio = std::move(arg);
    return Init_VFHeader_types(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_header_size
{
public:
  explicit Init_VFHeader_header_size(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_distance_ratio header_size(::olei_interfaces::msg::VFHeader::_header_size_type arg)
  {
    msg_.header_size = std::move(arg);
    return Init_VFHeader_distance_ratio(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_packet_size
{
public:
  explicit Init_VFHeader_packet_size(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_header_size packet_size(::olei_interfaces::msg::VFHeader::_packet_size_type arg)
  {
    msg_.packet_size = std::move(arg);
    return Init_VFHeader_header_size(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_version
{
public:
  explicit Init_VFHeader_version(::olei_interfaces::msg::VFHeader & msg)
  : msg_(msg)
  {}
  Init_VFHeader_packet_size version(::olei_interfaces::msg::VFHeader::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_VFHeader_packet_size(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

class Init_VFHeader_magic
{
public:
  Init_VFHeader_magic()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VFHeader_version magic(::olei_interfaces::msg::VFHeader::_magic_type arg)
  {
    msg_.magic = std::move(arg);
    return Init_VFHeader_version(msg_);
  }

private:
  ::olei_interfaces::msg::VFHeader msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::olei_interfaces::msg::VFHeader>()
{
  return olei_interfaces::msg::builder::Init_VFHeader_magic();
}

}  // namespace olei_interfaces

#endif  // OLEI_INTERFACES__MSG__DETAIL__VF_HEADER__BUILDER_HPP_
