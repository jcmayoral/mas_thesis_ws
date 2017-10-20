/**
 * Created on: October 16th, 2017
 * Author: Jose Carlos Mayoral
 */
#ifndef KINETIC_MONITOR_H_
#define KINETIC_MONITOR_H_

#include <ros/ros.h>
#include <list>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
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
        * Callback to Twists
        */
        void openLoopTwistCB(const geometry_msgs::TwistConstPtr &msg);
        void closeLoopTwistCB(const nav_msgs::OdometryConstPtr &msg);

    private:

        /**
         * An instance of the ROS Node handle.
         */
        ros::NodeHandle nh_;

         /* Subscribers and Publishers
         */
        ros::Subscriber open_loop_twist_sub_;
        ros::Subscriber close_loop_twist_sub_;

        std::list <geometry_msgs::TwistStamped> twist_historial_open_loop_;
        std::list <geometry_msgs::TwistStamped> twist_historial_close_loop_;

        double mass_;
        double radius_;

        bool request_received_;
        int max_number_elements_;
};

#endif  // KINETIC_MONITOR_H_
