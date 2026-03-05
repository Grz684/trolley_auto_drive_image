// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from olei_interfaces:msg/VFHeader.idl
// generated code does not contain a copyright notice

#ifndef OLEI_INTERFACES__MSG__DETAIL__VF_HEADER__STRUCT_HPP_
#define OLEI_INTERFACES__MSG__DETAIL__VF_HEADER__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__olei_interfaces__msg__VFHeader __attribute__((deprecated))
#else
# define DEPRECATED__olei_interfaces__msg__VFHeader __declspec(deprecated)
#endif

namespace olei_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VFHeader_
{
  using Type = VFHeader_<ContainerAllocator>;

  explicit VFHeader_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->magic = 0;
      this->version = 0;
      this->packet_size = 0ul;
      this->header_size = 0;
      this->distance_ratio = 0;
      this->types = 0;
      this->scan_number = 0;
      this->packet_number = 0;
      this->timestamp_decimal = 0ul;
      this->timestamp_integer = 0ul;
      this->scan_frequency = 0;
      this->num_points_scan = 0;
      this->iutput_status = 0;
      this->output_status = 0;
      this->field_status = 0ul;
      this->start_index = 0;
      this->end_index = 0;
      this->first_index = 0;
      this->num_points_packet = 0;
      this->status_flags = 0ul;
    }
  }

  explicit VFHeader_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->magic = 0;
      this->version = 0;
      this->packet_size = 0ul;
      this->header_size = 0;
      this->distance_ratio = 0;
      this->types = 0;
      this->scan_number = 0;
      this->packet_number = 0;
      this->timestamp_decimal = 0ul;
      this->timestamp_integer = 0ul;
      this->scan_frequency = 0;
      this->num_points_scan = 0;
      this->iutput_status = 0;
      this->output_status = 0;
      this->field_status = 0ul;
      this->start_index = 0;
      this->end_index = 0;
      this->first_index = 0;
      this->num_points_packet = 0;
      this->status_flags = 0ul;
    }
  }

  // field types and members
  using _magic_type =
    uint16_t;
  _magic_type magic;
  using _version_type =
    uint16_t;
  _version_type version;
  using _packet_size_type =
    uint32_t;
  _packet_size_type packet_size;
  using _header_size_type =
    uint16_t;
  _header_size_type header_size;
  using _distance_ratio_type =
    uint8_t;
  _distance_ratio_type distance_ratio;
  using _types_type =
    uint8_t;
  _types_type types;
  using _scan_number_type =
    uint16_t;
  _scan_number_type scan_number;
  using _packet_number_type =
    uint16_t;
  _packet_number_type packet_number;
  using _timestamp_decimal_type =
    uint32_t;
  _timestamp_decimal_type timestamp_decimal;
  using _timestamp_integer_type =
    uint32_t;
  _timestamp_integer_type timestamp_integer;
  using _scan_frequency_type =
    uint16_t;
  _scan_frequency_type scan_frequency;
  using _num_points_scan_type =
    uint16_t;
  _num_points_scan_type num_points_scan;
  using _iutput_status_type =
    uint16_t;
  _iutput_status_type iutput_status;
  using _output_status_type =
    uint16_t;
  _output_status_type output_status;
  using _field_status_type =
    uint32_t;
  _field_status_type field_status;
  using _start_index_type =
    uint16_t;
  _start_index_type start_index;
  using _end_index_type =
    uint16_t;
  _end_index_type end_index;
  using _first_index_type =
    uint16_t;
  _first_index_type first_index;
  using _num_points_packet_type =
    uint16_t;
  _num_points_packet_type num_points_packet;
  using _status_flags_type =
    uint32_t;
  _status_flags_type status_flags;

  // setters for named parameter idiom
  Type & set__magic(
    const uint16_t & _arg)
  {
    this->magic = _arg;
    return *this;
  }
  Type & set__version(
    const uint16_t & _arg)
  {
    this->version = _arg;
    return *this;
  }
  Type & set__packet_size(
    const uint32_t & _arg)
  {
    this->packet_size = _arg;
    return *this;
  }
  Type & set__header_size(
    const uint16_t & _arg)
  {
    this->header_size = _arg;
    return *this;
  }
  Type & set__distance_ratio(
    const uint8_t & _arg)
  {
    this->distance_ratio = _arg;
    return *this;
  }
  Type & set__types(
    const uint8_t & _arg)
  {
    this->types = _arg;
    return *this;
  }
  Type & set__scan_number(
    const uint16_t & _arg)
  {
    this->scan_number = _arg;
    return *this;
  }
  Type & set__packet_number(
    const uint16_t & _arg)
  {
    this->packet_number = _arg;
    return *this;
  }
  Type & set__timestamp_decimal(
    const uint32_t & _arg)
  {
    this->timestamp_decimal = _arg;
    return *this;
  }
  Type & set__timestamp_integer(
    const uint32_t & _arg)
  {
    this->timestamp_integer = _arg;
    return *this;
  }
  Type & set__scan_frequency(
    const uint16_t & _arg)
  {
    this->scan_frequency = _arg;
    return *this;
  }
  Type & set__num_points_scan(
    const uint16_t & _arg)
  {
    this->num_points_scan = _arg;
    return *this;
  }
  Type & set__iutput_status(
    const uint16_t & _arg)
  {
    this->iutput_status = _arg;
    return *this;
  }
  Type & set__output_status(
    const uint16_t & _arg)
  {
    this->output_status = _arg;
    return *this;
  }
  Type & set__field_status(
    const uint32_t & _arg)
  {
    this->field_status = _arg;
    return *this;
  }
  Type & set__start_index(
    const uint16_t & _arg)
  {
    this->start_index = _arg;
    return *this;
  }
  Type & set__end_index(
    const uint16_t & _arg)
  {
    this->end_index = _arg;
    return *this;
  }
  Type & set__first_index(
    const uint16_t & _arg)
  {
    this->first_index = _arg;
    return *this;
  }
  Type & set__num_points_packet(
    const uint16_t & _arg)
  {
    this->num_points_packet = _arg;
    return *this;
  }
  Type & set__status_flags(
    const uint32_t & _arg)
  {
    this->status_flags = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    olei_interfaces::msg::VFHeader_<ContainerAllocator> *;
  using ConstRawPtr =
    const olei_interfaces::msg::VFHeader_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<olei_interfaces::msg::VFHeader_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<olei_interfaces::msg::VFHeader_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      olei_interfaces::msg::VFHeader_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<olei_interfaces::msg::VFHeader_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      olei_interfaces::msg::VFHeader_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<olei_interfaces::msg::VFHeader_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<olei_interfaces::msg::VFHeader_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<olei_interfaces::msg::VFHeader_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__olei_interfaces__msg__VFHeader
    std::shared_ptr<olei_interfaces::msg::VFHeader_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__olei_interfaces__msg__VFHeader
    std::shared_ptr<olei_interfaces::msg::VFHeader_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VFHeader_ & other) const
  {
    if (this->magic != other.magic) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->packet_size != other.packet_size) {
      return false;
    }
    if (this->header_size != other.header_size) {
      return false;
    }
    if (this->distance_ratio != other.distance_ratio) {
      return false;
    }
    if (this->types != other.types) {
      return false;
    }
    if (this->scan_number != other.scan_number) {
      return false;
    }
    if (this->packet_number != other.packet_number) {
      return false;
    }
    if (this->timestamp_decimal != other.timestamp_decimal) {
      return false;
    }
    if (this->timestamp_integer != other.timestamp_integer) {
      return false;
    }
    if (this->scan_frequency != other.scan_frequency) {
      return false;
    }
    if (this->num_points_scan != other.num_points_scan) {
      return false;
    }
    if (this->iutput_status != other.iutput_status) {
      return false;
    }
    if (this->output_status != other.output_status) {
      return false;
    }
    if (this->field_status != other.field_status) {
      return false;
    }
    if (this->start_index != other.start_index) {
      return false;
    }
    if (this->end_index != other.end_index) {
      return false;
    }
    if (this->first_index != other.first_index) {
      return false;
    }
    if (this->num_points_packet != other.num_points_packet) {
      return false;
    }
    if (this->status_flags != other.status_flags) {
      return false;
    }
    return true;
  }
  bool operator!=(const VFHeader_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VFHeader_

// alias to use template instance with default allocator
using VFHeader =
  olei_interfaces::msg::VFHeader_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace olei_interfaces

#endif  // OLEI_INTERFACES__MSG__DETAIL__VF_HEADER__STRUCT_HPP_
