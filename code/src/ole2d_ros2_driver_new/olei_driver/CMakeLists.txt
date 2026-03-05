cmake_minimum_required(VERSION 3.7)
project(olei_driver)

find_package(ament_cmake REQUIRED)
find_package(olei_interfaces)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(olei_interfaces REQUIRED)

find_package(Boost REQUIRED COMPONENTS  thread system)
if(NOT TARGET JsonCpp::JsonCpp)
  find_package(jsoncpp REQUIRED)
elseif(NOT TARGET jsoncpp_lib)
  add_library(jsoncpp_lib ALIAS JsonCpp::JsonCpp)
endif()
find_package(yaml-cpp REQUIRED)
get_target_property(jsoncpp_INCLUDE_DIRS jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)
find_package(PkgConfig REQUIRED)
find_package(ament_cmake REQUIRED)
pkg_check_modules(CURLPP REQUIRED curlpp)

include_directories(
  include
  ${jsoncpp_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_SOURCES
  src/olei/olei_interface.cpp
  src/olei/olei_packet_reader.cpp
  src/pipeline.cpp
  src/olei/olei_packet/olei_packet.cpp
  src/olei/olei_packet/olei_vf_packet.cpp
  src/olei/olei_packet/olei_vf_packet_a.cpp
  src/olei/olei_packet/olei_vf_packet_b.cpp
  src/olei/olei_packet/olei_vf_packet_c.cpp
  src/olei/olei_packet/olei_version2_a.cpp
  src/olei/olei_packet/olei_version2.cpp
  src/communication/udp_transport.cpp
  src/communication/tcp_transport.cpp
  src/ros/lidar_publish.cpp
)

add_executable(ros_main
  ${${PROJECT_NAME}_SOURCES}
  src/ros/ros_main.cpp
)
target_link_libraries(ros_main
  curlpp
  curl
  jsoncpp
  yaml-cpp
)
ament_target_dependencies(ros_main
  rclcpp
  laser_geometry
  tf2_ros
  pcl_ros
  olei_interfaces
)

install(TARGETS ros_main
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include)

ament_export_include_directories(include)

ament_package()
