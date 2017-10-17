/**
 * Created on: October 16th, 2017
 * Author: Jose Carlos Mayoral
 */
#include <kinetic_energy_monitor/kinetic_energy_monitor.h>

KineticMonitor::KineticMonitor(ros::NodeHandle &nh):
        nh_(nh), mass_(20)
{
    //From Local_planner
    twist_sub_ = nh.subscribe("/cmd_vel",4, &KineticMonitor::twistCB, this);
    nh.param("/platform_mass", mass_,30.0);
    ROS_INFO("State: INIT");
}

KineticMonitor::~KineticMonitor()
{
}

bool KineticMonitor::runService(KineticEnergyMonitorMsg::Request  &req,
         KineticEnergyMonitorMsg::Response &resp)
{
    if (twist_historial_.size() > 1){
        ROS_INFO_STREAM("Request Received");
        resp.energy_lost = calculateDrop(req.collision_time);
        resp.success = true;
        return true;
    }
    else{
        ROS_WARN("Topics Missing Not Received");
        resp.success = false;
        return true;
    }
}

double KineticMonitor::calculateDrop(std_msgs::Header collision_time){

  double energy = 0.0;
  return energy;

}

void KineticMonitor::twistCB(const geometry_msgs::TwistConstPtr &msg){
    twist_historial_.push_back(*msg);
}
