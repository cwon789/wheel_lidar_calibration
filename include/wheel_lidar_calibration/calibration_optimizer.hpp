#ifndef WHEEL_LIDAR_CALIBRATION_CALIBRATION_OPTIMIZER_HPP_
#define WHEEL_LIDAR_CALIBRATION_CALIBRATION_OPTIMIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "wheel_lidar_calibration/data_synchronizer.hpp"

namespace wheel_lidar_calibration
{

struct CalibrationResult
{
  Eigen::Matrix4d transform_matrix;
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;
  double rmse;
  int iterations;
  bool converged;
};

class CalibrationOptimizer
{
public:
  CalibrationOptimizer();
  
  CalibrationResult optimize(const std::vector<OdometryPair>& odom_pairs);
  
  void setMaxIterations(int max_iter) { max_iterations_ = max_iter; }
  void setConvergenceThreshold(double threshold) { convergence_threshold_ = threshold; }
  void setInitialGuess(const Eigen::Matrix4d& initial_transform);
  
  geometry_msgs::msg::TransformStamped getTransformMsg(
    const std::string& parent_frame = "wheel_odom",
    const std::string& child_frame = "lidar_odom") const;

private:
  Eigen::Matrix4d computeTransformSVD(
    const std::vector<Eigen::Vector3d>& source_points,
    const std::vector<Eigen::Vector3d>& target_points);
  
  double computeRMSE(
    const std::vector<Eigen::Vector3d>& source_points,
    const std::vector<Eigen::Vector3d>& target_points,
    const Eigen::Matrix4d& transform);
  
  std::vector<Eigen::Vector3d> extractTrajectoryPoints(
    const std::vector<OdometryPair>& odom_pairs,
    bool use_wheel_odom);
  
  void refineWithLevenbergMarquardt(
    const std::vector<OdometryPair>& odom_pairs,
    Eigen::Matrix4d& transform);
  
  int max_iterations_;
  double convergence_threshold_;
  Eigen::Matrix4d initial_transform_;
  CalibrationResult result_;
  
  rclcpp::Logger logger_;
};

} // namespace wheel_lidar_calibration

#endif // WHEEL_LIDAR_CALIBRATION_CALIBRATION_OPTIMIZER_HPP_