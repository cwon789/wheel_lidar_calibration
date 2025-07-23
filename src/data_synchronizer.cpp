#include "wheel_lidar_calibration/data_synchronizer.hpp"
#include <algorithm>

namespace wheel_lidar_calibration
{

DataSynchronizer::DataSynchronizer(double max_time_diff, size_t buffer_size)
  : max_time_diff_(max_time_diff), buffer_size_(buffer_size)
{
}

void DataSynchronizer::addWheelOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(wheel_mutex_);
  wheel_buffer_.push_back(*msg);
  
  // Keep buffer size limited
  while (wheel_buffer_.size() > buffer_size_) {
    wheel_buffer_.pop_front();
  }
}

void DataSynchronizer::addLidarOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(lidar_mutex_);
  lidar_buffer_.push_back(*msg);
  
  // Keep buffer size limited
  while (lidar_buffer_.size() > buffer_size_) {
    lidar_buffer_.pop_front();
  }
}

bool DataSynchronizer::getSynchronizedPairs(std::vector<OdometryPair>& pairs, size_t min_pairs)
{
  std::lock_guard<std::mutex> wheel_lock(wheel_mutex_);
  std::lock_guard<std::mutex> lidar_lock(lidar_mutex_);
  
  pairs.clear();
  
  if (wheel_buffer_.size() < 2 || lidar_buffer_.size() < 2) {
    return false;
  }
  
  // For each lidar odometry, find the closest wheel odometry
  for (const auto& lidar_odom : lidar_buffer_) {
    rclcpp::Time lidar_time(lidar_odom.header.stamp);
    
    // Find wheel odometry messages that bracket the lidar time
    auto it = std::find_if(wheel_buffer_.begin(), wheel_buffer_.end(),
      [&lidar_time](const nav_msgs::msg::Odometry& wheel_odom) {
        return rclcpp::Time(wheel_odom.header.stamp) > lidar_time;
      });
    
    if (it != wheel_buffer_.begin() && it != wheel_buffer_.end()) {
      auto prev_it = std::prev(it);
      
      rclcpp::Time wheel_time1(prev_it->header.stamp);
      rclcpp::Time wheel_time2(it->header.stamp);
      
      // Check if lidar timestamp is within the bracket
      if (wheel_time1 <= lidar_time && lidar_time <= wheel_time2) {
        // Interpolate wheel odometry
        nav_msgs::msg::Odometry interpolated_wheel = 
          interpolateOdometry(*prev_it, *it, lidar_time);
        
        double time_diff = std::abs((lidar_time - rclcpp::Time(interpolated_wheel.header.stamp)).seconds());
        
        if (time_diff <= max_time_diff_) {
          OdometryPair pair;
          pair.wheel_odom = interpolated_wheel;
          pair.lidar_odom = lidar_odom;
          pair.time_diff = time_diff;
          pairs.push_back(pair);
        }
      }
    }
  }
  
  return pairs.size() >= min_pairs;
}

nav_msgs::msg::Odometry DataSynchronizer::interpolateOdometry(
  const nav_msgs::msg::Odometry& odom1,
  const nav_msgs::msg::Odometry& odom2,
  const rclcpp::Time& target_time)
{
  rclcpp::Time time1(odom1.header.stamp);
  rclcpp::Time time2(odom2.header.stamp);
  
  double dt = (time2 - time1).seconds();
  double alpha = (target_time - time1).seconds() / dt;
  
  nav_msgs::msg::Odometry result;
  result.header.stamp = target_time;
  result.header.frame_id = odom1.header.frame_id;
  result.child_frame_id = odom1.child_frame_id;
  
  // Interpolate position
  result.pose.pose.position.x = 
    odom1.pose.pose.position.x + alpha * (odom2.pose.pose.position.x - odom1.pose.pose.position.x);
  result.pose.pose.position.y = 
    odom1.pose.pose.position.y + alpha * (odom2.pose.pose.position.y - odom1.pose.pose.position.y);
  result.pose.pose.position.z = 
    odom1.pose.pose.position.z + alpha * (odom2.pose.pose.position.z - odom1.pose.pose.position.z);
  
  // Interpolate orientation using SLERP
  tf2::Quaternion q1, q2;
  tf2::fromMsg(odom1.pose.pose.orientation, q1);
  tf2::fromMsg(odom2.pose.pose.orientation, q2);
  tf2::Quaternion q_interp = q1.slerp(q2, alpha);
  result.pose.pose.orientation = tf2::toMsg(q_interp);
  
  // Interpolate velocities
  result.twist.twist.linear.x = 
    odom1.twist.twist.linear.x + alpha * (odom2.twist.twist.linear.x - odom1.twist.twist.linear.x);
  result.twist.twist.linear.y = 
    odom1.twist.twist.linear.y + alpha * (odom2.twist.twist.linear.y - odom1.twist.twist.linear.y);
  result.twist.twist.linear.z = 
    odom1.twist.twist.linear.z + alpha * (odom2.twist.twist.linear.z - odom1.twist.twist.linear.z);
  
  result.twist.twist.angular.x = 
    odom1.twist.twist.angular.x + alpha * (odom2.twist.twist.angular.x - odom1.twist.twist.angular.x);
  result.twist.twist.angular.y = 
    odom1.twist.twist.angular.y + alpha * (odom2.twist.twist.angular.y - odom1.twist.twist.angular.y);
  result.twist.twist.angular.z = 
    odom1.twist.twist.angular.z + alpha * (odom2.twist.twist.angular.z - odom1.twist.twist.angular.z);
  
  return result;
}

void DataSynchronizer::clearBuffers()
{
  std::lock_guard<std::mutex> wheel_lock(wheel_mutex_);
  std::lock_guard<std::mutex> lidar_lock(lidar_mutex_);
  
  wheel_buffer_.clear();
  lidar_buffer_.clear();
}

} // namespace wheel_lidar_calibration