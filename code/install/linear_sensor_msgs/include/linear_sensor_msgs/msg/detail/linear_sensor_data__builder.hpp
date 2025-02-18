// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from linear_sensor_msgs:msg/LinearSensorData.idl
// generated code does not contain a copyright notice

#ifndef LINEAR_SENSOR_MSGS__MSG__DETAIL__LINEAR_SENSOR_DATA__BUILDER_HPP_
#define LINEAR_SENSOR_MSGS__MSG__DETAIL__LINEAR_SENSOR_DATA__BUILDER_HPP_

#include "linear_sensor_msgs/msg/detail/linear_sensor_data__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace linear_sensor_msgs
{

namespace msg
{

namespace builder
{

class Init_LinearSensorData_data
{
public:
  explicit Init_LinearSensorData_data(::linear_sensor_msgs::msg::LinearSensorData & msg)
  : msg_(msg)
  {}
  ::linear_sensor_msgs::msg::LinearSensorData data(::linear_sensor_msgs::msg::LinearSensorData::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::linear_sensor_msgs::msg::LinearSensorData msg_;
};

class Init_LinearSensorData_header
{
public:
  Init_LinearSensorData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LinearSensorData_data header(::linear_sensor_msgs::msg::LinearSensorData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LinearSensorData_data(msg_);
  }

private:
  ::linear_sensor_msgs::msg::LinearSensorData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::linear_sensor_msgs::msg::LinearSensorData>()
{
  return linear_sensor_msgs::msg::builder::Init_LinearSensorData_header();
}

}  // namespace linear_sensor_msgs

#endif  // LINEAR_SENSOR_MSGS__MSG__DETAIL__LINEAR_SENSOR_DATA__BUILDER_HPP_
