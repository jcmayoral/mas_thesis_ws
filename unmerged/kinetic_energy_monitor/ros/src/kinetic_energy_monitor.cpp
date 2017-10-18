/**
 * Created on: October 16th, 2017
 * Author: Jose Carlos Mayoral
 */
#include <kinetic_energy_monitor/kinetic_energy_monitor.h>

KineticMonitor::KineticMonitor(ros::NodeHandle &nh):
        nh_(nh), mass_(20), request_received_(false), max_number_elements_(3)
{
    //From Local_planner
    twist_sub_ = nh.subscribe("/base/twist_mux/command_navigation",1, &KineticMonitor::twistCB, this);//TODO launch file with remap topic
    nh.param("/platform_mass", mass_,30.0);
    nh.param("/queue_size", max_number_elements_,3);
    ROS_INFO("State: INIT");
    ROS_INFO_STREAM("Subscribing Speed to "<< twist_sub_.getTopic());
}

KineticMonitor::~KineticMonitor()
{
  twist_sub_.shutdown();
}

bool KineticMonitor::runService(KineticEnergyMonitorMsg::Request  &req,
         KineticEnergyMonitorMsg::Response &resp)
{
    if (twist_historial_.size() > 1){
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

  for (std::list<geometry_msgs::TwistStamped>::iterator it = twist_historial_.begin() ; it != twist_historial_.end(); ++it){
    ROS_INFO_STREAM("Loooking for " << collision_time.stamp << ", " << " Stored" << it->header.stamp);
    if (collision_time.stamp >= it->header.stamp){
      ROS_WARN_STREAM("Collision Warning: " << collision_time.stamp << ", " << " in " << it->header.stamp);
    }
  }

  return energy;

}

void KineticMonitor::twistCB(const geometry_msgs::TwistConstPtr &msg){

    if (twist_historial_.size()==max_number_elements_)
    {
      twist_historial_.pop_front();
    }
    geometry_msgs::TwistStamped tmp;
    tmp.header.stamp = ros::Time::now();
    tmp.twist = *msg;
    twist_historial_.push_back(tmp);

}
