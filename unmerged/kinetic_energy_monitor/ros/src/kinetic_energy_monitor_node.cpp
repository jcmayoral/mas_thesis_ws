/**
 * Created on: October 16th, 2017
 * Author: Jose Carlos Mayoral
 */
#include <kinetic_energy_monitor/kinetic_energy_monitor.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "base_collision_checker_server");

    ros::NodeHandle nh("~");

    double loop_rate;

    /**
     * Cycle rate in Hz.
     */
    ros::param::param<double>("~loop_rate", loop_rate, 10.0);
    KineticMonitor kinetic_monitor(nh);

    ros::ServiceServer service = nh.advertiseService("/collision_checker",
        &KineticMonitor::runService, &kinetic_monitor);

    ROS_INFO("Ready to start...");

    ros::spin();
    return 0;
}
