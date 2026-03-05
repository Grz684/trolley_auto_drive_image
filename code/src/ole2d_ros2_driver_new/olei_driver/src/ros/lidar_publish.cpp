#include "ros/lidar_publish.h"
#include"olei/olei_packet/olei_packet.h"
#include "olei/olei_packet/olei_vf_packet.h"
#include "olei/olei_packet/olei_vf_packet_a.h"
#include "olei/olei_packet/olei_vf_packet_b.h"
#include "olei/olei_packet/olei_vf_packet_c.h"
#include "olei/olei_packet/olei_version2.h"
#include "olei/olei_packet/olei_version2_a.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <algorithm>

LidarPublisher::LidarPublisher(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<ScanConfig> config,
    std::shared_ptr<ScanParameters> params, const std::string& scan_topic,
    const std::string& frame_id)
     : node_(node),config_(config),params_(params),frame_id_(frame_id)
{
  scan_publisher_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::SensorDataQoS());
  cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(scan_topic + "_cloud", rclcpp::SensorDataQoS());
  header_publisher_ = node_->create_publisher<olei_interfaces::msg::VFHeader>("/v3_header", 1);
  header_publisher_2 = node_->create_publisher<olei_interfaces::msg::V2Header>("/v2_header", 1);
  frame_id_ = frame_id;
}

template <typename T>
void LidarPublisher::to_msg_queue(T &packet, uint16_t layer_idx, int layer_inclination)
{
  sensor_msgs::msg::LaserScan::SharedPtr msg;
  int actual_num = 0;  
  actual_num = packet.header.end_index - packet.header.start_index;
  if (d_queue_.empty())
    d_queue_.emplace_back();
  else if (d_queue_.size() > 5)
    d_queue_.pop_front();
  if (current_id > packet.header.first_index)
  {
    uint16_t frequency = packet.header.scan_frequency & 32767;
    const auto scan_time = rclcpp::Duration::from_seconds((1.0 / frequency)*60);
    msg.reset(new sensor_msgs::msg::LaserScan());
    msg->header.frame_id.assign(frame_id_);
    // msg->header.seq = packet.header.header.scan_number;
    msg->header.stamp = packet.last_acquired_point_stamp - scan_time;
    msg->scan_time = static_cast<float>(scan_time.seconds());
    msg->angle_increment = (360.0/packet.header.num_points_scan) * M_PI / 180.0;
    {
      msg->time_increment = msg->scan_time/(float)packet.header.num_points_scan;
      msg->angle_min = packet.header.start_index * msg->angle_increment - M_PI;
      msg->angle_max = packet.header.end_index * msg->angle_increment - M_PI;
      msg->range_min = config_->range_min;
      msg->range_max = config_->range_max;
    }   
    msg->ranges.resize(actual_num);
    if (!packet.intensity.empty())
      msg->intensities.resize(actual_num);
    d_queue_.push_back(msg);
    idx = 0;
  }
  current_id = packet.header.first_index;
  msg = d_queue_.back();
  if (!msg)
    return;
  // errors in scan_number - not in sequence sometimes
  /*if (msg->header.seq != packet.header.header.scan_number)
    return;*/
  //int idx = packet.header.first_index;
  for (int i = 0; i < packet.header.num_points_packet; i++)
  {
    if(idx>=actual_num)
      break;
    float data;
    if (packet.distance[i] == 0xFFFFFFFF)
      data = std::numeric_limits<std::uint32_t>::quiet_NaN();
    else
      data = packet.distance[i] / 1000.0;
    msg->ranges[idx] = std::move(data);
    if (!packet.intensity.empty())
      msg->intensities[idx] = packet.intensity[i];
    idx++;
  }
  if (actual_num == idx)
  {
    if (msg)
    {     
      if(config_->ntp)
        msg->header.stamp = rclcpp::Time(packet.header.timestamp_integer, 
packet.header.timestamp_decimal);
      else
        msg->header.stamp = rclcpp::Clock().now();
      if(config_->inverted)
      {
        std::reverse(msg->ranges.begin(), msg->ranges.end());
        if (!msg->intensities.empty())
           std::reverse(msg->intensities.begin(), msg->intensities.end());
      }   
      publish_scan(msg);
      d_queue_.pop_back();
      idx = 0;
    }
  }
}

template <typename T>
void LidarPublisher::to_msg_queue2(T &packet, uint16_t layer_idx, int layer_inclination)
{
  sensor_msgs::msg::LaserScan::SharedPtr msg;
  if (d_queue_.empty())
    d_queue_.emplace_back();
  else if (d_queue_.size() > 5)
    d_queue_.pop_front(); 
  if (current_id > packet.angle[0])
  {  
    uint16_t frequency = packet.header.scan_frequency & 32767;
    const auto scan_time = rclcpp::Duration::from_seconds((1.0 / frequency)*60);
    msg.reset(new sensor_msgs::msg::LaserScan());
    msg->header.frame_id.assign(frame_id_);
    // msg->header.seq = packet.header.header.scan_number;
    msg->header.stamp = packet.last_acquired_point_stamp - scan_time;
    msg->scan_time = static_cast<float>(scan_time.seconds());
    msg->angle_increment = (packet.angle[10] - packet.angle[0]) * M_PI / 180000.0;
    {
      msg->time_increment = msg->scan_time / num;
      msg->angle_min = packet.angle[0]* M_PI / 18000.0 - M_PI;
      msg->angle_max = packet.angle[0]* M_PI / 18000.0 - M_PI + msg->angle_increment*num;
      msg->range_min = config_->range_min;
      msg->range_max = config_->range_max;
    } 
    msg->ranges.resize(num);
    current_num = num;
    if (!packet.intensity.empty())
      msg->intensities.resize(num);
    d_queue_.push_back(msg);
    num = 0;
    idx = 0;
  }
  current_id = packet.angle[0];
  num += packet.distance.size();
  msg = d_queue_.back();
  if (!msg)
    return;
  // errors in scan_number - not in sequence sometimes
  /*if (msg->header.seq != packet.header.header.scan_number)
    return;*/
  //int idx = packet.header.first_index;
  for (int i = 0; i < packet.distance.size(); i++)
  {
    if(idx>=msg->ranges.size())
      break;
    float data;
    if (packet.distance[i] == 0xFFFFFFFF)
      data = std::numeric_limits<std::uint32_t>::quiet_NaN();
    else
      data = packet.distance[i] / 1000.0;
    msg->ranges[idx] = std::move(data);
    if (!packet.intensity.empty())
      msg->intensities[idx] = packet.intensity[i];
    idx++;
  }
  if (current_num == idx)
  {
    if (msg)
    { 
      if(config_->ntp)
        msg->header.stamp = rclcpp::Time(packet.header.status_flags, packet.header.timestamp);
      else
        msg->header.stamp = rclcpp::Clock().now();
      if(config_->inverted)
      {
        std::reverse(msg->ranges.begin(), msg->ranges.end());
        if (!msg->intensities.empty())
           std::reverse(msg->intensities.begin(), msg->intensities.end());
      }   
      publish_scan(msg);
      d_queue_.pop_back();
      idx = 0;
      current_num = 0;
    }
  }
}

void LidarPublisher::read(OleiVFPacket_A &packet)
{
    publish_header(packet.header);
    to_msg_queue<OleiVFPacket_A>(packet);
}

void LidarPublisher::read(OleiVFPacket_B &packet)
{
    publish_header(packet.header);
    to_msg_queue<OleiVFPacket_B>(packet);
}

void LidarPublisher::read(OleiVFPacket_C &packet)
{
    publish_header(packet.header);
    to_msg_queue<OleiVFPacket_C>(packet);
}


void LidarPublisher::read(Olei2Packet_A &packet)
{
    header_publisher_2->publish(packet.header);
    to_msg_queue2<Olei2Packet_A>(packet);
}

bool LidarPublisher::start()
{
    return true;
}

bool LidarPublisher::stop()
{
    return true;
}

void LidarPublisher::publish_header(olei_interfaces::msg::VFHeader &header)
{
  header_publisher_->publish(header);
}

void LidarPublisher::publish_scan(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    scan_publisher_->publish(*msg);
    publish_pointcloud(msg);
}

void LidarPublisher::publish_pointcloud(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    auto cloud_msg = convertToPointCloud2(msg);
    cloud_publisher_->publish(cloud_msg);
}

sensor_msgs::msg::PointCloud2 LidarPublisher::convertToPointCloud2(sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = scan->header;
    cloud.height = 1;
    cloud.width = scan->ranges.size();
    cloud.is_bigendian = false;
    cloud.is_dense = false;
    
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(scan->ranges.size());
    
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");
    
    float angle = scan->angle_min;
    for (size_t i = 0; i < scan->ranges.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity)
    {
        const float range = scan->ranges[i];
        
        if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max)
        {
            *iter_x = range * cos(angle);
            *iter_y = range * sin(angle);
            *iter_z = 0.0;
        }
        else
        {
            *iter_x = std::numeric_limits<float>::quiet_NaN();
            *iter_y = std::numeric_limits<float>::quiet_NaN();
            *iter_z = std::numeric_limits<float>::quiet_NaN();
        }
        
        if (!scan->intensities.empty())
            *iter_intensity = scan->intensities[i];
        else
            *iter_intensity = 0.0;
        
        angle += scan->angle_increment;
    }
    
    return cloud;
}
