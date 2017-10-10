/**
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 * base_collision_checker_node.cpp
 *
 * Created on: May 16th, 2016
 * Author: Jose
 */
#include <footprint_checker/base_collision_checker.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "base_collision_checker_server");

    ros::NodeHandle nh("~");

    double loop_rate;

    /**
     * Cycle rate in Hz.
     */
    ros::param::param<double>("~loop_rate", loop_rate, 10.0);
    BaseCollisionChecker base_collision_checker(nh);

    ros::ServiceServer service = nh.advertiseService("CollisionCheckerService",
        &BaseCollisionChecker::runService, &base_collision_checker);

    ROS_INFO("Ready to start...");

    ros::spin();
    return 0;
}
