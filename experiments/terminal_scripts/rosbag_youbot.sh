#!/bin/bash

#echo '$1 = ' $1

rosbag record  /arm_cam3d/rgb/image_raw /clock /diagnostics /cmd_vel /odom /scan_combined /accel /tf
