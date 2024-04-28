#ifndef ROS2_lidar__CONVERSIONS_HPP_
#define ROS2_lidar__CONVERSIONS_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros2_lidar/point_os.hpp"
#include "ros2_lidar/image_os.hpp"
#include "ros2_lidar/scan_os.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "lidar_msgs/msg/metadata.hpp"

#include "ros2_lidar/OS1/OS1.hpp"
#include "ros2_lidar/OS1/OS1_packet.hpp"

namespace ros2_lidar
{

/**
 * @brief Run-time big endian check
 *
 * Run-time initialize a constant determining if the underlying machine is big
 * endian. This is needed to properly stamp the `PointCloud2` ROS message
 * (below).
 */
static const bool IS_BIGENDIAN = []()
  {
    std::uint16_t dummy = 0x1;
    std::uint8_t * dummy_ptr = reinterpret_cast<std::uint8_t *>(&dummy);
    return dummy_ptr[0] == 0x1 ? false : true;
  } ();

/**
 * @brief Convert ClientState to string
 */
inline std::string toString(const ClientState & state)
{
  switch (state) {
    case TIMEOUT:
      return std::string("timeout");
    case ERROR:
      return std::string("error");
    case EXIT:
      return std::string("exit");
    case IMU_DATA:
      return std::string("imu data");
    case LIDAR_DATA:
      return std::string("lidar data");
    default:
      return std::string("unknown");
  }
}

/**
 * @brief Convert metadata to message format
 */
inline lidar_msgs::msg::Metadata toMsg(const ros2_lidar::Metadata & mdata)
{
  lidar_msgs::msg::Metadata msg;
  msg.computer_ip = mdata.computer_ip;
  msg.lidar_ip = mdata.lidar_ip;
  msg.imu_port = mdata.imu_port;
  msg.lidar_port = mdata.lidar_port;
  return msg;
}

/**
 * @brief Convert transformation to message format
 */
inline geometry_msgs::msg::TransformStamped toMsg(
  const std::vector<double> & mat, const std::string & frame,
  const std::string & child_frame, const rclcpp::Time & time)
{
  assert(mat.size() == 16);

  tf2::Transform tf;

  tf.setOrigin({mat[3] / 1e3, mat[7] / 1e3, mat[11] / 1e3});
  tf.setBasis({mat[0], mat[1], mat[2], mat[4], mat[5], mat[6], mat[8], mat[9], mat[10]});

  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame;
  msg.child_frame_id = child_frame;
  msg.transform = tf2::toMsg(tf);

  return msg;
}

/**
 * @brief Convert IMU to message format
 */
inline sensor_msgs::msg::Imu toMsg(
  const uint8_t * buf,
  const std::string & frame,
  uint64_t override_ts = 0)
{
  // rsv for later
  const double standard_g = 9.80665;
  sensor_msgs::msg::Imu m;
  m.orientation.x = 0;
  m.orientation.y = 0;
  m.orientation.z = 0;
  m.orientation.w = 1;
  /*
  m.header.stamp = override_ts == 0 ?
    rclcpp::Time(OS1::imu_gyro_ts(buf)) : rclcpp::Time(override_ts);
  m.header.frame_id = frame;

  m.linear_acceleration.x = OS1::imu_la_x(buf) * standard_g;
  m.linear_acceleration.y = OS1::imu_la_y(buf) * standard_g;
  m.linear_acceleration.z = OS1::imu_la_z(buf) * standard_g;

  m.angular_velocity.x = OS1::imu_av_x(buf) * M_PI / 180.0;
  m.angular_velocity.y = OS1::imu_av_y(buf) * M_PI / 180.0;
  m.angular_velocity.z = OS1::imu_av_z(buf) * M_PI / 180.0;
  */
  m.header.stamp = rclcpp::Time(0);
  m.header.frame_id = frame;

  m.linear_acceleration.x = 0;
  m.linear_acceleration.y = 0;
  m.linear_acceleration.z = 0;

  m.angular_velocity.x = 0;
  m.angular_velocity.y = 0;
  m.angular_velocity.z = 0;



  for (int i = 0; i < 9; i++) {
    m.orientation_covariance[i] = -1;
    m.angular_velocity_covariance[i] = 0;
    m.linear_acceleration_covariance[i] = 0;
  }
  for (int i = 0; i < 9; i += 4) {
    m.linear_acceleration_covariance[i] = 0.01;
    m.angular_velocity_covariance[i] = 6e-4;
  }

  return m;
}

/**
 * @brief Convert Pointcloud to ROS message format
 *
 * This function assumes the input `cloud` has its data in column major order
 * but shaped improperly according to the LiDAR resulting in tiled columns
 * across the rows of the in-memory data buffer. This data format is described
 * more closely here:
 * https://github.com/SteveMacenski/ros2_lidar_drivers/issues/52
 *
 * This function converts the PCL type to a ROS type and the resulting ROS type
 * will reorder the data in memory to be in row-major order consistent to the
 * rows x cols shape of the LiDAR receiver array.
 *
 * @param[in] cloud A PCL PointCloud containing lidar point data as described
 *                  above.
 * @param[in] timestamp The timestamp to put on the ROS message header
 * @param[in] frame The TF coordinate frame identifier to put on the ROS
 *                  message header.
 *
 * @return A ROS `PointCloud2` message of LiDAR data whose memory buffer is
 *         row-major ordered consistent to the shape of the LiDAR array.
 */
inline sensor_msgs::msg::PointCloud2 toMsg(
  const pcl::PointCloud<point_os::PointOS> & cloud,
  uint32_t & realwidth,
  std::chrono::nanoseconds timestamp,
  const std::string & frame)
{
//std::cout << "PointCloud2 toMsg start" <<std::endl;
  std::size_t pt_size = sizeof(point_os::PointOS);
  std::size_t data_size = pt_size * cloud.points.size();

  // here data_size = 1536000 = 48 * 16 * 2000? = pt_size * height * width?
  
  //std::cout << "PointCloud2 toMsg realwidth" << std::to_string(realwidth) <<std::endl;
  pcl::PCLPointCloud2 cloud2;
  cloud2.height = cloud.height;
  // modified by zyl
  // cloud2.width = (uint32_t)(data_size/(cloud.height * pt_size));
  cloud2.width = realwidth;
  cloud2.fields.clear();
  pcl::for_each_type<typename pcl::traits::fieldList<point_os::PointOS>::type>(
    pcl::detail::FieldAdder<point_os::PointOS>(cloud2.fields));
  cloud2.header = cloud.header;
  cloud2.point_step = pt_size;
  cloud2.row_step = static_cast<std::uint32_t>(pt_size * cloud2.width);
  cloud2.is_dense = cloud.is_dense;
  cloud2.is_bigendian = ros2_lidar::IS_BIGENDIAN;


  data_size = pt_size * cloud2.width * cloud2.height;

  cloud2.data.resize(data_size);

  if (data_size) {
    // column-major to row-major conversion
    for (int i = 0; i < cloud2.width; ++i) {
      for (int j = 0; j < cloud2.height; ++j) {
    	std::memcpy(
    	          &cloud2.data[(j * cloud2.width + i) * pt_size],
    	          &cloud.points[i * cloud2.height + j],
    	          pt_size);


      }
    }
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl_conversions::moveFromPCL(cloud2, msg);
  msg.header.frame_id = frame;
  rclcpp::Time t(timestamp.count());
  msg.header.stamp = t;
  return msg;
}

/**
 * @brief Convert Scan to message format
 */
inline sensor_msgs::msg::LaserScan toMsg(
  const std::vector<scan_os::ScanOS> & scans,
  uint32_t & realwidth,
  rclcpp::Time timestamp,
  const std::string & frame,
  const ros2_lidar::Metadata & mdata,
  const uint8_t ring_to_use)
{
  sensor_msgs::msg::LaserScan msg;
  //rclcpp::Time t(timestamp.count());
  // here maybe depend mdata.vendor,
  msg.header.stamp = timestamp;
  //msg.header.stamp = rclcpp::Time(0);
  msg.header.frame_id = frame;
  msg.angle_min = -M_PI;
  msg.angle_max = M_PI;

  //here keep same with pointcloud
  //msg.angle_min = 0;
  //msg.angle_max = 2 * M_PI;

  msg.range_min = 0.1;
  msg.range_max = 10.0;

  // modified by zyl
  double resolution;

  resolution = realwidth;
  //rate = 15.0;
  //std::cout << scans[0].frequency  <<std::endl;
  // unit:ns
  msg.scan_time = 1.0/ scans[0].frequency;
  msg.time_increment = msg.scan_time / (double)(resolution-1);
  msg.angle_increment = 2 * M_PI / resolution;
  //std::cout << scans[0].frequency  <<std::endl;
  //std::cout << "split frame and publish: "  <<std::endl;
  // scans.size() = 32000? = 16 * 2000;
  uint32_t real_size = mdata.num_lasers * realwidth;

  float min_angle = -90.0 * M_PI / 180.0;
  float max_angle = 90.0 * M_PI / 180.0;
  float max_range = 10.0;

  for (uint i = 0; i != real_size; i++) {
    if (scans[i].ring == ring_to_use) {
      float current_angle = msg.angle_min + i * msg.angle_increment;
      float current_range = scans[i].range * 1e-3;
      if (current_angle < min_angle || current_angle > max_angle || current_range > max_range) {
        msg.ranges.push_back(0 * 1e-3);
        msg.intensities.push_back(scans[i].intensity);
      }
      else {
        msg.ranges.push_back(scans[i].range * 1e-3);
        msg.intensities.push_back(scans[i].intensity);
      }
    }
  }

  return msg;
}

}  // namespace ros2_lidar

#endif  // ROS2_lidar__CONVERSIONS_HPP_
