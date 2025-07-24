# 2d LiDAR Wheel extrinsic calibration project

bash# Launch the calibration node
ros2 launch wheel_lidar_calibration calibration.launch.py

# In another terminal, start your robot and collect diverse motion data
# The node will buffer odometry data from both sources

# Trigger calibration
ros2 service call /calibrate std_srvs/srv/Trigger
