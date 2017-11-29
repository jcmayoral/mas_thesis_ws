#!/bin/bash

echo '$1 = ' $1

rosbag record  /cam3d/rgb/image_raw /clock /diagnostics /cmd_vel /base/odometry_controller/odometry /scan_combined /accel /tf -o $1
