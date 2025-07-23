#include "wheel_lidar_calibration/calibration_optimizer.hpp"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <tf2_eigen/tf2_eigen.h>
#include <cmath>

namespace wheel_lidar_calibration
{

CalibrationOptimizer::CalibrationOptimizer()
  : max_iterations_(100),
    convergence_threshold_(1e-6),
    initial_transform_(Eigen::Matrix4d::Identity()),
    logger_(rclcpp::get_logger("calibration_optimizer"))
{
}

CalibrationResult CalibrationOptimizer::optimize(const std::vector<OdometryPair>& odom_pairs)
{
  if (odom_pairs.size() < 10) {
    RCLCPP_ERROR(logger_, "Not enough odometry pairs for calibration. Need at least 10, got %zu", 
                 odom_pairs.size());
    result_.converged = false;
    return result_;
  }
  
  RCLCPP_INFO(logger_, "Starting calibration with %zu odometry pairs", odom_pairs.size());
  
  // Extract trajectory points
  std::vector<Eigen::Vector3d> wheel_points = extractTrajectoryPoints(odom_pairs, true);
  std::vector<Eigen::Vector3d> lidar_points = extractTrajectoryPoints(odom_pairs, false);
  
  // Initial estimate using SVD
  Eigen::Matrix4d transform = computeTransformSVD(lidar_points, wheel_points);
  
  // Refine with Levenberg-Marquardt if needed
  double prev_rmse = std::numeric_limits<double>::max();
  int iteration = 0;
  
  for (; iteration < max_iterations_; ++iteration) {
    double rmse = computeRMSE(lidar_points, wheel_points, transform);
    
    if (std::abs(prev_rmse - rmse) < convergence_threshold_) {
      RCLCPP_INFO(logger_, "Converged after %d iterations with RMSE: %f", iteration, rmse);
      result_.converged = true;
      break;
    }
    
    // Refine transformation
    refineWithLevenbergMarquardt(odom_pairs, transform);
    prev_rmse = rmse;
  }
  
  // Fill result
  result_.transform_matrix = transform;
  result_.translation = transform.block<3, 1>(0, 3);
  result_.rotation = Eigen::Quaterniond(transform.block<3, 3>(0, 0));
  result_.rmse = computeRMSE(lidar_points, wheel_points, transform);
  result_.iterations = iteration;
  
  RCLCPP_INFO(logger_, "Calibration completed:");
  RCLCPP_INFO(logger_, "  Translation: [%f, %f, %f]", 
              result_.translation.x(), result_.translation.y(), result_.translation.z());
  RCLCPP_INFO(logger_, "  Rotation (quaternion): [%f, %f, %f, %f]",
              result_.rotation.x(), result_.rotation.y(), result_.rotation.z(), result_.rotation.w());
  RCLCPP_INFO(logger_, "  Final RMSE: %f", result_.rmse);
  
  return result_;
}

Eigen::Matrix4d CalibrationOptimizer::computeTransformSVD(
  const std::vector<Eigen::Vector3d>& source_points,
  const std::vector<Eigen::Vector3d>& target_points)
{
  // Compute centroids
  Eigen::Vector3d source_centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_centroid = Eigen::Vector3d::Zero();
  
  for (size_t i = 0; i < source_points.size(); ++i) {
    source_centroid += source_points[i];
    target_centroid += target_points[i];
  }
  
  source_centroid /= source_points.size();
  target_centroid /= target_points.size();
  
  // Compute cross-covariance matrix
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  
  for (size_t i = 0; i < source_points.size(); ++i) {
    H += (source_points[i] - source_centroid) * (target_points[i] - target_centroid).transpose();
  }
  
  // SVD decomposition
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  
  // Compute rotation
  Eigen::Matrix3d R = V * U.transpose();
  
  // Handle reflection case
  if (R.determinant() < 0) {
    V.col(2) *= -1;
    R = V * U.transpose();
  }
  
  // Compute translation
  Eigen::Vector3d t = target_centroid - R * source_centroid;
  
  // Build transformation matrix
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = R;
  transform.block<3, 1>(0, 3) = t;
  
  return transform;
}

double CalibrationOptimizer::computeRMSE(
  const std::vector<Eigen::Vector3d>& source_points,
  const std::vector<Eigen::Vector3d>& target_points,
  const Eigen::Matrix4d& transform)
{
  double sum_squared_error = 0.0;
  
  for (size_t i = 0; i < source_points.size(); ++i) {
    Eigen::Vector4d src_homo(source_points[i].x(), source_points[i].y(), source_points[i].z(), 1.0);
    Eigen::Vector4d transformed = transform * src_homo;
    Eigen::Vector3d transformed_point = transformed.head<3>();
    
    double error = (transformed_point - target_points[i]).norm();
    sum_squared_error += error * error;
  }
  
  return std::sqrt(sum_squared_error / source_points.size());
}

std::vector<Eigen::Vector3d> CalibrationOptimizer::extractTrajectoryPoints(
  const std::vector<OdometryPair>& odom_pairs,
  bool use_wheel_odom)
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(odom_pairs.size());
  
  for (const auto& pair : odom_pairs) {
    const auto& odom = use_wheel_odom ? pair.wheel_odom : pair.lidar_odom;
    
    Eigen::Vector3d point(
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      odom.pose.pose.position.z
    );
    
    points.push_back(point);
  }
  
  return points;
}

void CalibrationOptimizer::refineWithLevenbergMarquardt(
  const std::vector<OdometryPair>& odom_pairs,
  Eigen::Matrix4d& transform)
{
  // Simple gradient descent refinement
  // For more sophisticated implementation, consider using Ceres Solver or similar
  
  const double learning_rate = 0.01;
  const double delta = 1e-5;
  
  // Extract current rotation and translation
  Eigen::Matrix3d R = transform.block<3, 3>(0, 0);
  Eigen::Vector3d t = transform.block<3, 1>(0, 3);
  
  // Convert to axis-angle for optimization
  Eigen::AngleAxisd aa(R);
  Eigen::Vector3d axis = aa.axis();
  double angle = aa.angle();
  
  // Parameters: [axis_x, axis_y, axis_z, angle, tx, ty, tz]
  Eigen::VectorXd params(7);
  params << axis.x(), axis.y(), axis.z(), angle, t.x(), t.y(), t.z();
  
  // Compute gradient numerically
  Eigen::VectorXd gradient(7);
  
  for (int i = 0; i < 7; ++i) {
    Eigen::VectorXd params_plus = params;
    Eigen::VectorXd params_minus = params;
    params_plus(i) += delta;
    params_minus(i) -= delta;
    
    // Reconstruct transforms
    Eigen::Matrix4d transform_plus = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d transform_minus = Eigen::Matrix4d::Identity();
    
    // Plus
    Eigen::Vector3d axis_plus(params_plus(0), params_plus(1), params_plus(2));
    axis_plus.normalize();
    Eigen::AngleAxisd aa_plus(params_plus(3), axis_plus);
    transform_plus.block<3, 3>(0, 0) = aa_plus.toRotationMatrix();
    transform_plus.block<3, 1>(0, 3) = params_plus.tail<3>();
    
    // Minus
    Eigen::Vector3d axis_minus(params_minus(0), params_minus(1), params_minus(2));
    axis_minus.normalize();
    Eigen::AngleAxisd aa_minus(params_minus(3), axis_minus);
    transform_minus.block<3, 3>(0, 0) = aa_minus.toRotationMatrix();
    transform_minus.block<3, 1>(0, 3) = params_minus.tail<3>();
    
    // Compute costs
    std::vector<Eigen::Vector3d> wheel_points = extractTrajectoryPoints(odom_pairs, true);
    std::vector<Eigen::Vector3d> lidar_points = extractTrajectoryPoints(odom_pairs, false);
    
    double cost_plus = computeRMSE(lidar_points, wheel_points, transform_plus);
    double cost_minus = computeRMSE(lidar_points, wheel_points, transform_minus);
    
    gradient(i) = (cost_plus - cost_minus) / (2.0 * delta);
  }
  
  // Update parameters
  params -= learning_rate * gradient;
  
  // Reconstruct transform
  Eigen::Vector3d new_axis(params(0), params(1), params(2));
  new_axis.normalize();
  Eigen::AngleAxisd new_aa(params(3), new_axis);
  transform.block<3, 3>(0, 0) = new_aa.toRotationMatrix();
  transform.block<3, 1>(0, 3) = params.tail<3>();
}

void CalibrationOptimizer::setInitialGuess(const Eigen::Matrix4d& initial_transform)
{
  initial_transform_ = initial_transform;
}

geometry_msgs::msg::TransformStamped CalibrationOptimizer::getTransformMsg(
  const std::string& parent_frame,
  const std::string& child_frame) const
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = rclcpp::Clock().now();
  tf_msg.header.frame_id = parent_frame;
  tf_msg.child_frame_id = child_frame;
  
  tf_msg.transform.translation.x = result_.translation.x();
  tf_msg.transform.translation.y = result_.translation.y();
  tf_msg.transform.translation.z = result_.translation.z();
  
  tf_msg.transform.rotation.x = result_.rotation.x();
  tf_msg.transform.rotation.y = result_.rotation.y();
  tf_msg.transform.rotation.z = result_.rotation.z();
  tf_msg.transform.rotation.w = result_.rotation.w();
  
  return tf_msg;
}

} // namespace wheel_lidar_calibration