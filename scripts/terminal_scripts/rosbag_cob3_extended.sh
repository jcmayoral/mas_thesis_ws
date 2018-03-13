#!/bin/bash

echo '$1 = ' $1

rosbag record  /audio /cam3d/rgb/image_raw /base/twist_mux/command_navigation /base/odometry_controller/odometry /scan_combined /camera/image_raw /phone1/android/imu /imu/data /tf  -o $1
