# gps_odometry

This ROS pakage converts GNSS data received by a XSens IMU sensor and converts it into local positional data.

## Prequisites
- ROS Foxy
- bluespace_ai_xsens_ros_mti_driver (https://github.com/bluespace-ai/bluespace_ai_xsens_ros_mti_driver)

The variable wait_for_fix_time can be changed based on the time taken to get a GNSS fix by the IMU.

