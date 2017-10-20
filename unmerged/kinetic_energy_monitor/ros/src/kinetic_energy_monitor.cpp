/**
 * Created on: October 16th, 2017
 * Author: Jose Carlos Mayoral
 */
#include <kinetic_energy_monitor/kinetic_energy_monitor.h>

KineticMonitor::KineticMonitor(ros::NodeHandle &nh):
        nh_(nh), mass_(20), radius_(1), request_received_(false), max_number_elements_(3)
{
    //From Local_planner
    open_loop_twist_sub_ = nh.subscribe("/base/twist_mux/command_navigation",1, &KineticMonitor::openLoopTwistCB, this);//TODO launch file with remap topic
    close_loop_twist_sub_ = nh.subscribe("/base/odometry_controller/odometry",1, &KineticMonitor::closeLoopTwistCB, this);//TODO launch file with remap topic

    nh.param("/platform_mass", mass_,30.0);
    nh.param("/rotation_radius", radius_,30.0);
    nh.param("/queue_size", max_number_elements_,3);
    ROS_INFO("State: INIT");
}

KineticMonitor::~KineticMonitor()
{
  open_loop_twist_sub_.shutdown();
  close_loop_twist_sub_.shutdown();
}

bool KineticMonitor::runService(KineticEnergyMonitorMsg::Request  &req,
         KineticEnergyMonitorMsg::Response &resp)
{
    if (twist_historial_open_loop_.size() > 1){
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

  std::list<geometry_msgs::TwistStamped>::iterator it = twist_historial_open_loop_.begin();
  std::list<geometry_msgs::TwistStamped>::iterator it2 = twist_historial_close_loop_.begin();

  if (twist_historial_open_loop_.size() != twist_historial_close_loop_.size()){
    ROS_ERROR("Missing Messages");
    return 0.0;
  }

  double diff_speed_x_last, diff_speed_x = 0;
  double diff_speed_y_last, diff_speed_y = 0;
  double diff_speed_z_last, diff_speed_z = 0;

  for ( ; it != twist_historial_open_loop_.end() || it2 != twist_historial_close_loop_.end(); ++it, ++it2){

    //ROS_INFO_STREAM("Loooking for " << collision_time.stamp << ", " << " Stored" << it->header.stamp);
    diff_speed_x = it->twist.linear.x - it2->twist.linear.x;
    diff_speed_y = it->twist.linear.y - it2->twist.linear.y;
    diff_speed_z = it->twist.angular.z - it2->twist.angular.z;
    //ROS_INFO_STREAM("differences " << diff_speed_x << ", " << diff_speed_y << ", " << diff_speed_z);

    if (collision_time.stamp >= it->header.stamp && collision_time.stamp >= it2 -> header.stamp){
      break;
    }
    diff_speed_x_last = diff_speed_x;
    diff_speed_y_last = diff_speed_y;
    diff_speed_z_last = diff_speed_z;
  }

  double speed_0 = sqrt(pow(diff_speed_x_last,2) + pow(diff_speed_y_last,2)) + radius_ * diff_speed_z_last;
  double speed_1 = sqrt(pow(diff_speed_x,2) + pow(diff_speed_y,2)) + radius_ * diff_speed_z;
  ROS_INFO_STREAM("coefficient of restitution" << speed_1/speed_0);

  double linear_speed = sqrt(pow(diff_speed_x - diff_speed_x_last,2) + pow(diff_speed_y - diff_speed_y_last,2));
  double angular_speed = radius_ * (diff_speed_z - diff_speed_z_last);
  double force = mass_ * (linear_speed + angular_speed);
  double energy = 0.5 * force;


  ROS_INFO_STREAM("Collision Force " << mass_ * (linear_speed + angular_speed));
  return energy;

}

void KineticMonitor::openLoopTwistCB(const geometry_msgs::TwistConstPtr &msg){

    if (twist_historial_open_loop_.size()==max_number_elements_)
    {
      twist_historial_open_loop_.pop_front();
    }
    geometry_msgs::TwistStamped tmp;
    tmp.header.stamp = ros::Time::now();
    tmp.twist = *msg;
    twist_historial_open_loop_.push_back(tmp);

}

void KineticMonitor::closeLoopTwistCB(const nav_msgs::OdometryConstPtr &msg){

    if (twist_historial_close_loop_.size()==max_number_elements_)
    {
      twist_historial_close_loop_.pop_front();
    }

    geometry_msgs::TwistStamped tmp;
    tmp.header.stamp = ros::Time::now();
    tmp.twist = msg->twist.twist;
    twist_historial_close_loop_.push_back(tmp);

}
