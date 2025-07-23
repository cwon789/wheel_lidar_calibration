#ifndef WHEEL_LIDAR_CALIBRATION_DATA_SYNCHRONIZER_HPP_
#define WHEEL_LIDAR_CALIBRATION_DATA_SYNCHRONIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <deque>
#include <mutex>
#include <memory>

namespace wheel_lidar_calibration
{

struct OdometryPair
{
  nav_msgs::msg::Odometry wheel_odom;
  nav_msgs::msg::Odometry lidar_odom;
  double time_diff;
};

class DataSynchronizer
{
public:
  DataSynchronizer(double max_time_diff = 0.02, size_t buffer_size = 1000);
  
  void addWheelOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void addLidarOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  
  bool getSynchronizedPairs(std::vector<OdometryPair>& pairs, size_t min_pairs = 50);
  void clearBuffers();
  
  size_t getWheelBufferSize() const { return wheel_buffer_.size(); }
  size_t getLidarBufferSize() const { return lidar_buffer_.size(); }

private:
  nav_msgs::msg::Odometry interpolateOdometry(
    const nav_msgs::msg::Odometry& odom1,
    const nav_msgs::msg::Odometry& odom2,
    const rclcpp::Time& target_time);
  
  double max_time_diff_;
  size_t buffer_size_;
  
  std::deque<nav_msgs::msg::Odometry> wheel_buffer_;
  std::deque<nav_msgs::msg::Odometry> lidar_buffer_;
  
  mutable std::mutex wheel_mutex_;
  mutable std::mutex lidar_mutex_;
};

} // namespace wheel_lidar_calibration

#endif // WHEEL_LIDAR_CALIBRATION_DATA_SYNCHRONIZER_HPP_