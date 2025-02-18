// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from linear_sensor_msgs:msg/LinearSensorData.idl
// generated code does not contain a copyright notice

#ifndef LINEAR_SENSOR_MSGS__MSG__DETAIL__LINEAR_SENSOR_DATA__STRUCT_HPP_
#define LINEAR_SENSOR_MSGS__MSG__DETAIL__LINEAR_SENSOR_DATA__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__linear_sensor_msgs__msg__LinearSensorData __attribute__((deprecated))
#else
# define DEPRECATED__linear_sensor_msgs__msg__LinearSensorData __declspec(deprecated)
#endif

namespace linear_sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LinearSensorData_
{
  using Type = LinearSensorData_<ContainerAllocator>;

  explicit LinearSensorData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0.0f;
    }
  }

  explicit LinearSensorData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _data_type =
    float;
  _data_type data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__data(
    const float & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    linear_sensor_msgs::msg::LinearSensorData_<ContainerAllocator> *;
  using ConstRawPtr =
    const linear_sensor_msgs::msg::LinearSensorData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<linear_sensor_msgs::msg::LinearSensorData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<linear_sensor_msgs::msg::LinearSensorData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      linear_sensor_msgs::msg::LinearSensorData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<linear_sensor_msgs::msg::LinearSensorData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      linear_sensor_msgs::msg::LinearSensorData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<linear_sensor_msgs::msg::LinearSensorData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<linear_sensor_msgs::msg::LinearSensorData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<linear_sensor_msgs::msg::LinearSensorData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__linear_sensor_msgs__msg__LinearSensorData
    std::shared_ptr<linear_sensor_msgs::msg::LinearSensorData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__linear_sensor_msgs__msg__LinearSensorData
    std::shared_ptr<linear_sensor_msgs::msg::LinearSensorData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LinearSensorData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const LinearSensorData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LinearSensorData_

// alias to use template instance with default allocator
using LinearSensorData =
  linear_sensor_msgs::msg::LinearSensorData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace linear_sensor_msgs

#endif  // LINEAR_SENSOR_MSGS__MSG__DETAIL__LINEAR_SENSOR_DATA__STRUCT_HPP_
