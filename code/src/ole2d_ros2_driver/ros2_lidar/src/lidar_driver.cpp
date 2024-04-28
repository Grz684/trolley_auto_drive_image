#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/qos.hpp"
#include "ros2_lidar/lidar_driver.hpp"
#include "ros2_lidar/exception.hpp"
#include "ros2_lidar/interfaces/lifecycle_interface.hpp"
#include "ros2_lidar/interfaces/sensor_interface.hpp"
#include "ros2_lidar/OS1/processor_factories.hpp"

namespace ros2_lidar
{

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

lidarDriver::lidarDriver(
  std::unique_ptr<SensorInterface> sensor,
  const rclcpp::NodeOptions & options)
: LifecycleInterface("lidarDriver", options), _sensor{std::move(sensor)}
{
  // added by zyl for clang warning
  _use_system_default_qos = false;
  _use_ros_time =false;
  _os1_proc_mask = 0;

  // modified by zyl
  this->declare_parameter("lidar_ip");
  this->declare_parameter("computer_ip");
  this->declare_parameter("imu_port",rclcpp::ParameterValue(9866));
  this->declare_parameter("lidar_port",rclcpp::ParameterValue(2368));
  this->declare_parameter("imu_to_sensor_transform");
  this->declare_parameter("lidar_to_sensor_transform");

  this->declare_parameter("sensor_frame", rclcpp::ParameterValue(std::string("laser_sensor_frame")));
  this->declare_parameter("laser_frame", rclcpp::ParameterValue(std::string("laser_data_frame")));
  this->declare_parameter("imu_frame", rclcpp::ParameterValue(std::string("imu_data_frame")));
  this->declare_parameter("use_system_default_qos", rclcpp::ParameterValue(false));
  this->declare_parameter("timestamp_mode", rclcpp::ParameterValue(std::string("timestamp_mode")));
  // used to gen processor,
  this->declare_parameter("os1_proc_mask", rclcpp::ParameterValue(std::string("PCL")));

  // added by zyl
  this->declare_parameter("x_offset_array");
  this->declare_parameter("y_offset_array");
  this->declare_parameter("ah_offset_array");
  this->declare_parameter("av_offset_array");
  this->declare_parameter("laser_id_array");
  this->declare_parameter("num_lasers");
  this->declare_parameter("distance_resolution");
  this->declare_parameter("ring_scan");
  this->declare_parameter("lidar_vendor", rclcpp::ParameterValue(std::string("OLE_3D_V2")));

}

lidarDriver::~lidarDriver() = default;

void lidarDriver::onConfigure()
{
  std::cout << "get_parameter start" << std::endl;
  ros2_lidar::Configuration lidar_config;
  try {
    lidar_config.lidar_ip = get_parameter("lidar_ip").as_string();
    lidar_config.computer_ip = get_parameter("computer_ip").as_string();
  } catch (...) {
    RCLCPP_FATAL( this->get_logger(), "Failed to get lidar or IMU IP address or hostname. An IP address for both are required!");
    exit(-1);
  }

  lidar_config.imu_port = get_parameter("imu_port").as_int();
  lidar_config.lidar_port = get_parameter("lidar_port").as_int();
  std::cout << "get_parameter tp1" << std::endl;
  // vendor
  lidar_config.lidar_vendor = get_parameter("lidar_vendor").as_string();


  RCLCPP_INFO( this->get_logger(), "Connecting to sensor %s.", lidar_config.lidar_vendor.c_str());


  if(lidar_config.lidar_vendor == std::string("OLE_3D_V2")){
	  std::cout << "get_parameter tp2" << std::endl;
	  lidar_config.lidar_packet_size = 1206;
	  lidar_config.imu_packet_size = 842;
  }
  else{
	  std::cout << "get_parameter tp3" << std::endl;
      lidar_config.lidar_packet_size = 772;
      lidar_config.imu_packet_size = 842;     //rsv
  }

  //_sensor->reset(lidar_config);
  //lidar_config.timestamp_mode = get_parameter("timestamp_mode").as_string();
  _use_ros_time = true;
  //if (lidar_config.timestamp_mode == "TIME_FROM_ROS_RECEPTION") {
  //  RCLCPP_WARN(this->get_logger(), "Using TIME_FROM_ROS_RECEPTION to stamp data with ROS time on reception. This has unmodelled latency!");
  //  lidar_config.timestamp_mode = "TIME_FROM_INTERNAL_OSC";
   // _use_ros_time = true;
  //} else {
   // _use_ros_time = false;
  //}





  _laser_sensor_frame = get_parameter("sensor_frame").as_string();
  _laser_data_frame = get_parameter("laser_frame").as_string();
  _imu_data_frame = get_parameter("imu_frame").as_string();

  _use_system_default_qos = get_parameter("use_system_default_qos").as_bool();
  _os1_proc_mask = ros2_lidar::toProcMask(get_parameter("os1_proc_mask").as_string());


  RCLCPP_INFO( this->get_logger(), "Connecting to sensor at %s.", lidar_config.lidar_ip.c_str());
  RCLCPP_INFO( this->get_logger(), "Sending data from sensor to %s.", lidar_config.computer_ip.c_str());

  _reset_srv = this->create_service<std_srvs::srv::Empty>(
    "~/reset", std::bind(&lidarDriver::resetService, this, _1, _2, _3));
  _metadata_srv = this->create_service<lidar_msgs::srv::GetMetadata>(
    "~/get_metadata", std::bind(&lidarDriver::getMetadata, this, _1, _2, _3));

  //
  try {
    _sensor->configure(lidar_config);
  } catch (const lidarDriverException & e) {
    RCLCPP_FATAL(this->get_logger(), "Exception thrown: (%s)", e.what());
    exit(-1);
  }

  // added by zyl
  // ros2_lidar::Metadata mdata;

  mdata.computer_ip = get_parameter("computer_ip").as_string();
  mdata.lidar_ip = get_parameter("lidar_ip").as_string();
  mdata.imu_port = get_parameter("imu_port").as_int();
  mdata.lidar_port = get_parameter("lidar_port").as_int();
  mdata.imu_to_sensor_transform = get_parameter("imu_to_sensor_transform").as_double_array();
  mdata.lidar_to_sensor_transform = get_parameter("lidar_to_sensor_transform").as_double_array();

  // for correct
  mdata.ring_scan = get_parameter("ring_scan").as_int();
  mdata.num_lasers = get_parameter("num_lasers").as_int();
  mdata.distance_resolution = get_parameter("distance_resolution").as_double();
  mdata.x_offset_array = get_parameter("x_offset_array").as_double_array();
  mdata.y_offset_array = get_parameter("y_offset_array").as_double_array();
  mdata.ah_offset_array = get_parameter("ah_offset_array").as_double_array();
  mdata.av_offset_array = get_parameter("av_offset_array").as_double_array();
  mdata.laser_id_array = get_parameter("laser_id_array").as_integer_array();

  // vendor
  mdata.lidar_vendor = get_parameter("lidar_vendor").as_string();
  if(mdata.lidar_vendor == std::string("OLE_3D_V2")){
	  mdata.lidar_packet_size = 1206;
	  mdata.imu_packet_size = 842;
  }
  else{
	  mdata.lidar_packet_size = 1240;
	  mdata.imu_packet_size = 842;     //rsv
  }

  //ros2_lidar::Metadata mdata = _sensor->getMetadata();
  // end of added

  // create processors according _os1_proc_mask
  if (_use_system_default_qos) {
    RCLCPP_INFO(this->get_logger(), "Using system defaults QoS for sensor data");
    _data_processors = ros2_lidar::createProcessors(
      shared_from_this(), mdata, _imu_data_frame, _laser_data_frame,
      rclcpp::SystemDefaultsQoS(), _os1_proc_mask);
  } else {
    _data_processors = ros2_lidar::createProcessors(
      shared_from_this(), mdata, _imu_data_frame, _laser_data_frame,
      rclcpp::SensorDataQoS(), _os1_proc_mask);
  }

  // tf2 broadcast

  _tf_b = std::make_unique<tf2_ros::StaticTransformBroadcaster>(
    shared_from_this());
  broadcastStaticTransforms(mdata);

}

void lidarDriver::onActivate()
{
  DataProcessorMapIt it;
  for (it = _data_processors.begin(); it != _data_processors.end(); ++it) {
    it->second->onActivate();
  }

  _process_timer = this->create_wall_timer(
	1000000ns,
	std::bind(&lidarDriver::processData, this));
}

void lidarDriver::onError()
{
}

void lidarDriver::onDeactivate()
{
  _process_timer->cancel();
  _process_timer.reset();

  DataProcessorMapIt it;
  for (it = _data_processors.begin(); it != _data_processors.end(); ++it) {
    it->second->onDeactivate();
  }
}

void lidarDriver::onCleanup()
{
  _data_processors.clear();
  _tf_b.reset();
  _reset_srv.reset();
  _metadata_srv.reset();
}

void lidarDriver::onShutdown()
{
  _process_timer->cancel();
  _process_timer.reset();
  _tf_b.reset();

  DataProcessorMapIt it;
  for (it = _data_processors.begin(); it != _data_processors.end(); ++it) {
    delete it->second;
  }
  _data_processors.clear();
}

void lidarDriver::broadcastStaticTransforms(const ros2_lidar::Metadata & mdata)
{
	RCLCPP_INFO( this->get_logger(),"broadcastStaticTransformsCall");

  //if (_tf_b) {
   /// std::vector<geometry_msgs::msg::TransformStamped> transforms;
   // transforms.push_back( toMsg( mdata.imu_to_sensor_transform, _laser_sensor_frame, _imu_data_frame, this->now()));
   // transforms.push_back( toMsg( mdata.lidar_to_sensor_transform, _laser_sensor_frame, _laser_data_frame, this->now()));
   // _tf_b->sendTransform(transforms);
  //}
}

void lidarDriver::processData()
{
  try {
    ClientState state = _sensor->get();
    /*
    RCLCPP_DEBUG(
      this->get_logger(),
      "Packet with state: %s",
      ros2_lidar::toString(state).c_str());
    */
    uint8_t * packet_data = _sensor->readPacket(state);

    if (packet_data) {
    	uint32_t size = sizeof(packet_data);
      std::pair<DataProcessorMapIt, DataProcessorMapIt> key_its;
      key_its = _data_processors.equal_range(state);
      uint64_t override_ts = this->_use_ros_time ? this->now().nanoseconds() : 0;

      for (DataProcessorMapIt it = key_its.first; it != key_its.second; it++) {
        it->second->process(packet_data, override_ts);
      }
    }
  } catch (const lidarDriverException & e) {
    RCLCPP_WARN( this->get_logger(), "Failed to process packet with exception %s.", e.what());
  }
}

void lidarDriver::resetService(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  if (!this->isActive()) return; 

  ros2_lidar::Configuration lidar_config;
  lidar_config.lidar_ip = get_parameter("lidar_ip").as_string();
  // RCLCPP_INFO( this->get_logger(), "Connecting to sensor %s.", lidar_config.lidar_ip.c_str());
  lidar_config.computer_ip = get_parameter("computer_ip").as_string();
  lidar_config.imu_port = get_parameter("imu_port").as_int();
  lidar_config.lidar_port = get_parameter("lidar_port").as_int();
  lidar_config.lidar_mode = get_parameter("lidar_mode").as_string();
  lidar_config.timestamp_mode = get_parameter("timestamp_mode").as_string();
  _sensor->reset(lidar_config);
}

void lidarDriver::getMetadata(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<lidar_msgs::srv::GetMetadata::Request> request,
  std::shared_ptr<lidar_msgs::srv::GetMetadata::Response> response)
{
  if (!this->isActive()) return; 
  //response->metadata = toMsg(_sensor->getMetadata());
  response->metadata = toMsg(mdata);
}

}  // namespace ros2_lidar
