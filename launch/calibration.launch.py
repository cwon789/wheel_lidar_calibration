import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('wheel_lidar_calibration')
    config_file = os.path.join(pkg_dir, 'config', 'calibration_params.yaml')
    
    # Declare launch arguments
    wheel_odom_topic_arg = DeclareLaunchArgument(
        'wheel_odom_topic',
        default_value='/wheel/odom',
        description='Topic name for wheel odometry'
    )
    
    lidar_odom_topic_arg = DeclareLaunchArgument(
        'lidar_odom_topic',
        default_value='/lidar/odom',
        description='Topic name for LiDAR odometry'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start calibration when enough data is collected'
    )
    
    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='calibration_result.txt',
        description='Output file for calibration results'
    )
    
    # Create the node
    calibration_node = Node(
        package='wheel_lidar_calibration',
        executable='calibration_node',
        name='wheel_lidar_calibration_node',
        output='screen',
        parameters=[
            config_file,
            {
                'wheel_odom_topic': LaunchConfiguration('wheel_odom_topic'),
                'lidar_odom_topic': LaunchConfiguration('lidar_odom_topic'),
                'auto_start': LaunchConfiguration('auto_start'),
                'output_file': LaunchConfiguration('output_file'),
            }
        ]
    )
    
    return LaunchDescription([
        wheel_odom_topic_arg,
        lidar_odom_topic_arg,
        auto_start_arg,
        output_file_arg,
        calibration_node
    ])