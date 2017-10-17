/**
 * Created on: October 16th, 2017
 * Author: Jose Carlos Mayoral
 */
#ifndef KINETIC_MONITOR_H_
#define KINETIC_MONITOR_H_

#include <ros/ros.h>
#include <vector>
#include <mutex>          // std::mutex
#include <geometry_msgs/Twist.h>
#include <std_msgs/Header.h>
#include <kinetic_energy_monitor/KineticEnergyMonitorMsg.h>

using namespace kinetic_energy_monitor;

class KineticMonitor
{
    public:
        /**
         * Ctor.
         *
         * @param nh An instance of the ROS node handle.
         */
        KineticMonitor(ros::NodeHandle &nh);

        /**
         * Dtor.
         */
        virtual ~KineticMonitor();

        /**
         * Performs state transitions and executes the
         * functionality related to current state.
         */
        bool runService(KineticEnergyMonitorMsg::Request  &req, KineticEnergyMonitorMsg::Response &resp);

    private:
        /**
        * Calculate Drop
        */
        double calculateDrop(std_msgs::Header collision_time);

        /**
        * Callback to set footprint
        */

        void twistCB(const geometry_msgs::TwistConstPtr &msg);

    private:

        /**
         * An instance of the ROS Node handle.
         */
        ros::NodeHandle nh_;

         /* Subscribers and Publishers
         */
        ros::Subscriber twist_sub_;

        std::vector <geometry_msgs::Twist> twist_historial_;
        double mass_;

};

#endif  // KINETIC_MONITOR_H_
