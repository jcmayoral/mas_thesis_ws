#!/bin/bash

echo '$1 = ' $1

rosbag record  /cam3d/rgb/image_raw /clock /diagnostics /base/twist_mux/command_navigation /base/odometry_controller/odometry /scan_combined /accel /tf /base/twist_mux/command_navigation -o $1
