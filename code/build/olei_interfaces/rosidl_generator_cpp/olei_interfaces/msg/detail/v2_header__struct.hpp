// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from olei_interfaces:msg/V2Header.idl
// generated code does not contain a copyright notice

#ifndef OLEI_INTERFACES__MSG__DETAIL__V2_HEADER__STRUCT_HPP_
#define OLEI_INTERFACES__MSG__DETAIL__V2_HEADER__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__olei_interfaces__msg__V2Header __attribute__((deprecated))
#else
# define DEPRECATED__olei_interfaces__msg__V2Header __declspec(deprecated)
#endif

namespace olei_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct V2Header_
{
  using Type = V2Header_<ContainerAllocator>;

  explicit V2Header_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->magic = 0ul;
      this->version = 0;
      this->distance_ratio = 0;
      std::fill<typename std::array<uint8_t, 3>::iterator, uint8_t>(this->brand.begin(), this->brand.end(), 0);
      std::fill<typename std::array<uint8_t, 12>::iterator, uint8_t>(this->vommercial.begin(), this->vommercial.end(), 0);
      this->internal = 0;
      this->hardware = 0;
      this->software = 0;
      this->timestamp = 0ul;
      this->scan_frequency = 0;
      this->safe_status = 0;
      this->error_status = 0;
      this->status_flags = 0ul;
    }
  }

  explicit V2Header_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : brand(_alloc),
    vommercial(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->magic = 0ul;
      this->version = 0;
      this->distance_ratio = 0;
      std::fill<typename std::array<uint8_t, 3>::iterator, uint8_t>(this->brand.begin(), this->brand.end(), 0);
      std::fill<typename std::array<uint8_t, 12>::iterator, uint8_t>(this->vommercial.begin(), this->vommercial.end(), 0);
      this->internal = 0;
      this->hardware = 0;
      this->software = 0;
      this->timestamp = 0ul;
      this->scan_frequency = 0;
      this->safe_status = 0;
      this->error_status = 0;
      this->status_flags = 0ul;
    }
  }

  // field types and members
  using _magic_type =
    uint32_t;
  _magic_type magic;
  using _version_type =
    uint16_t;
  _version_type version;
  using _distance_ratio_type =
    uint8_t;
  _distance_ratio_type distance_ratio;
  using _brand_type =
    std::array<uint8_t, 3>;
  _brand_type brand;
  using _vommercial_type =
    std::array<uint8_t, 12>;
  _vommercial_type vommercial;
  using _internal_type =
    uint16_t;
  _internal_type internal;
  using _hardware_type =
    uint16_t;
  _hardware_type hardware;
  using _software_type =
    uint16_t;
  _software_type software;
  using _timestamp_type =
    uint32_t;
  _timestamp_type timestamp;
  using _scan_frequency_type =
    uint16_t;
  _scan_frequency_type scan_frequency;
  using _safe_status_type =
    uint8_t;
  _safe_status_type safe_status;
  using _error_status_type =
    uint8_t;
  _error_status_type error_status;
  using _status_flags_type =
    uint32_t;
  _status_flags_type status_flags;

  // setters for named parameter idiom
  Type & set__magic(
    const uint32_t & _arg)
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
  Type & set__distance_ratio(
    const uint8_t & _arg)
  {
    this->distance_ratio = _arg;
    return *this;
  }
  Type & set__brand(
    const std::array<uint8_t, 3> & _arg)
  {
    this->brand = _arg;
    return *this;
  }
  Type & set__vommercial(
    const std::array<uint8_t, 12> & _arg)
  {
    this->vommercial = _arg;
    return *this;
  }
  Type & set__internal(
    const uint16_t & _arg)
  {
    this->internal = _arg;
    return *this;
  }
  Type & set__hardware(
    const uint16_t & _arg)
  {
    this->hardware = _arg;
    return *this;
  }
  Type & set__software(
    const uint16_t & _arg)
  {
    this->software = _arg;
    return *this;
  }
  Type & set__timestamp(
    const uint32_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__scan_frequency(
    const uint16_t & _arg)
  {
    this->scan_frequency = _arg;
    return *this;
  }
  Type & set__safe_status(
    const uint8_t & _arg)
  {
    this->safe_status = _arg;
    return *this;
  }
  Type & set__error_status(
    const uint8_t & _arg)
  {
    this->error_status = _arg;
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
    olei_interfaces::msg::V2Header_<ContainerAllocator> *;
  using ConstRawPtr =
    const olei_interfaces::msg::V2Header_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<olei_interfaces::msg::V2Header_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<olei_interfaces::msg::V2Header_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      olei_interfaces::msg::V2Header_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<olei_interfaces::msg::V2Header_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      olei_interfaces::msg::V2Header_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<olei_interfaces::msg::V2Header_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<olei_interfaces::msg::V2Header_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<olei_interfaces::msg::V2Header_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__olei_interfaces__msg__V2Header
    std::shared_ptr<olei_interfaces::msg::V2Header_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__olei_interfaces__msg__V2Header
    std::shared_ptr<olei_interfaces::msg::V2Header_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const V2Header_ & other) const
  {
    if (this->magic != other.magic) {
      return false;
    }
    if (this->version != other.version) {
      return false;
    }
    if (this->distance_ratio != other.distance_ratio) {
      return false;
    }
    if (this->brand != other.brand) {
      return false;
    }
    if (this->vommercial != other.vommercial) {
      return false;
    }
    if (this->internal != other.internal) {
      return false;
    }
    if (this->hardware != other.hardware) {
      return false;
    }
    if (this->software != other.software) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->scan_frequency != other.scan_frequency) {
      return false;
    }
    if (this->safe_status != other.safe_status) {
      return false;
    }
    if (this->error_status != other.error_status) {
      return false;
    }
    if (this->status_flags != other.status_flags) {
      return false;
    }
    return true;
  }
  bool operator!=(const V2Header_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct V2Header_

// alias to use template instance with default allocator
using V2Header =
  olei_interfaces::msg::V2Header_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace olei_interfaces

#endif  // OLEI_INTERFACES__MSG__DETAIL__V2_HEADER__STRUCT_HPP_
