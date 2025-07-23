#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/msg/bool.hpp>
#include "std_srvs/srv/trigger.hpp"
#include <fstream>
#include <iomanip>

#include "wheel_lidar_calibration/data_synchronizer.hpp"
#include "wheel_lidar_calibration/calibration_optimizer.hpp"

namespace wheel_lidar_calibration
{

class CalibrationNode : public rclcpp::Node
{
public:
  CalibrationNode() : Node("wheel_lidar_calibration_node")
  {
    // Declare parameters
    this->declare_parameter("wheel_odom_topic", "/wheel/odom");
    this->declare_parameter("lidar_odom_topic", "/lidar/odom");
    this->declare_parameter("max_time_diff", 0.02);
    this->declare_parameter("buffer_size", 1000);
    this->declare_parameter("min_pairs_for_calibration", 100);
    this->declare_parameter("max_iterations", 100);
    this->declare_parameter("convergence_threshold", 1e-6);
    this->declare_parameter("auto_start", false);
    this->declare_parameter("output_file", "calibration_result.txt");
    this->declare_parameter("publish_tf", true);
    
    // Get parameters
    wheel_odom_topic_ = this->get_parameter("wheel_odom_topic").as_string();
    lidar_odom_topic_ = this->get_parameter("lidar_odom_topic").as_string();
    max_time_diff_ = this->get_parameter("max_time_diff").as_double();
    buffer_size_ = this->get_parameter("buffer_size").as_int();
    min_pairs_ = this->get_parameter("min_pairs_for_calibration").as_int();
    max_iterations_ = this->get_parameter("max_iterations").as_int();
    convergence_threshold_ = this->get_parameter("convergence_threshold").as_double();
    auto_start_ = this->get_parameter("auto_start").as_bool();
    output_file_ = this->get_parameter("output_file").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    
    // Initialize components
    data_sync_ = std::make_unique<DataSynchronizer>(max_time_diff_, buffer_size_);
    optimizer_ = std::make_unique<CalibrationOptimizer>();
    optimizer_->setMaxIterations(max_iterations_);
    optimizer_->setConvergenceThreshold(convergence_threshold_);
    
    // Create subscriptions
    wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      wheel_odom_topic_, 10,
      std::bind(&CalibrationNode::wheelOdomCallback, this, std::placeholders::_1));
    
    lidar_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      lidar_odom_topic_, 10,
      std::bind(&CalibrationNode::lidarOdomCallback, this, std::placeholders::_1));
    
    // Create service for triggering calibration
    calibrate_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "calibrate",
      std::bind(&CalibrationNode::calibrateService, this, std::placeholders::_1, std::placeholders::_2));
    
    // Create TF broadcaster
    if (publish_tf_) {
      tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }
    
    // Create timer for status updates
    status_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&CalibrationNode::statusTimerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Calibration node initialized");
    RCLCPP_INFO(this->get_logger(), "Wheel odometry topic: %s", wheel_odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "LiDAR odometry topic: %s", lidar_odom_topic_.c_str());
    
    if (auto_start_) {
      RCLCPP_INFO(this->get_logger(), "Auto-start enabled. Will calibrate when enough data is collected.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Call the '/calibrate' service to start calibration.");
    }
  }

private:
  void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    data_sync_->addWheelOdometry(msg);
    
    if (auto_start_ && !calibration_done_) {
      checkAndCalibrate();
    }
  }
  
  void lidarOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    data_sync_->addLidarOdometry(msg);
  }
  
  void calibrateService(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    (void)request;  // Unused
    
    RCLCPP_INFO(this->get_logger(), "Calibration service called");
    
    bool success = performCalibration();
    
    response->success = success;
    if (success) {
      response->message = "Calibration completed successfully. Results saved to " + output_file_;
    } else {
      response->message = "Calibration failed. Check logs for details.";
    }
  }
  
  void checkAndCalibrate()
  {
    std::vector<OdometryPair> pairs;
    if (data_sync_->getSynchronizedPairs(pairs, min_pairs_)) {
      RCLCPP_INFO(this->get_logger(), "Collected enough data. Starting automatic calibration...");
      performCalibration();
      calibration_done_ = true;
    }
  }
  
  bool performCalibration()
  {
    std::vector<OdometryPair> pairs;
    
    if (!data_sync_->getSynchronizedPairs(pairs, min_pairs_)) {
      RCLCPP_ERROR(this->get_logger(), 
                   "Not enough synchronized pairs. Got %zu, need at least %d",
                   pairs.size(), min_pairs_);
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Starting calibration with %zu synchronized pairs", pairs.size());
    
    // Perform calibration
    CalibrationResult result = optimizer_->optimize(pairs);
    
    if (!result.converged) {
      RCLCPP_ERROR(this->get_logger(), "Calibration did not converge");
      return false;
    }
    
    // Save results
    saveCalibrationResult(result);
    
    // Publish TF if enabled
    if (publish_tf_) {
      publishTransform();
    }
    
    // Clear buffers after successful calibration
    data_sync_->clearBuffers();
    
    return true;
  }
  
  void saveCalibrationResult(const CalibrationResult& result)
  {
    std::ofstream file(output_file_);
    
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", output_file_.c_str());
      return;
    }
    
    file << std::fixed << std::setprecision(6);
    file << "# Wheel-LiDAR Extrinsic Calibration Result\n";
    file << "# Generated: " << rclcpp::Clock().now().seconds() << "\n\n";
    
    file << "# Translation (x, y, z) [meters]\n";
    file << "translation: [" 
         << result.translation.x() << ", " 
         << result.translation.y() << ", " 
         << result.translation.z() << "]\n\n";
    
    file << "# Rotation (quaternion: x, y, z, w)\n";
    file << "rotation: [" 
         << result.rotation.x() << ", " 
         << result.rotation.y() << ", " 
         << result.rotation.z() << ", " 
         << result.rotation.w() << "]\n\n";
    
    file << "# Transformation Matrix (4x4)\n";
    file << "transform_matrix:\n";
    for (int i = 0; i < 4; ++i) {
      file << "  - [";
      for (int j = 0; j < 4; ++j) {
        file << result.transform_matrix(i, j);
        if (j < 3) file << ", ";
      }
      file << "]\n";
    }
    
    file << "\n# Calibration Statistics\n";
    file << "rmse: " << result.rmse << "\n";
    file << "iterations: " << result.iterations << "\n";
    file << "converged: " << (result.converged ? "true" : "false") << "\n";
    
    file.close();
    
    RCLCPP_INFO(this->get_logger(), "Calibration results saved to: %s", output_file_.c_str());
  }
  
  void publishTransform()
  {
    geometry_msgs::msg::TransformStamped tf_msg = 
      optimizer_->getTransformMsg("wheel_odom", "lidar_odom");
    
    tf_broadcaster_->sendTransform(tf_msg);
    
    RCLCPP_INFO(this->get_logger(), "Published static transform from wheel_odom to lidar_odom");
  }
  
  void statusTimerCallback()
  {
    RCLCPP_INFO(this->get_logger(), 
                "Buffer status - Wheel: %zu, LiDAR: %zu messages",
                data_sync_->getWheelBufferSize(),
                data_sync_->getLidarBufferSize());
  }

  // ROS2 interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  
  // Core components
  std::unique_ptr<DataSynchronizer> data_sync_;
  std::unique_ptr<CalibrationOptimizer> optimizer_;
  
  // Parameters
  std::string wheel_odom_topic_;
  std::string lidar_odom_topic_;
  double max_time_diff_;
  size_t buffer_size_;
  int min_pairs_;
  int max_iterations_;
  double convergence_threshold_;
  bool auto_start_;
  std::string output_file_;
  bool publish_tf_;
  
  // State
  bool calibration_done_ = false;
};

} // namespace wheel_lidar_calibration

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheel_lidar_calibration::CalibrationNode>());
  rclcpp::shutdown();
  return 0;
}