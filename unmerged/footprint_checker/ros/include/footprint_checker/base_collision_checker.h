/**
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 * base_collision_checker_node.cpp
 *
 * Created on: May 16th, 2016
 * Author: Jose
 */
#ifndef MCR_BASE_COLLISION_CHECKER_BASE_COLLISION_CHECKER_H_
#define MCR_BASE_COLLISION_CHECKER_BASE_COLLISION_CHECKER_H_

#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include <vector>
#include <footprint_checker/collision_checker.h>
#include <footprint_checker/CollisionCheckerMsg.h>

/**
 * This class provides ROS interface of the
 * base collision checker.
 */
class BaseCollisionChecker
{
    public:
        /**
         * Ctor.
         *
         * @param nh An instance of the ROS node handle.
         */
        BaseCollisionChecker(const ros::NodeHandle &nh);

        /**
         * Dtor.
         */
        virtual ~BaseCollisionChecker();

        /**
         * Performs state transitions and executes the
         * functionality related to current state.
         */
        bool runService(footprint_checker::CollisionCheckerMsg::Request  &req,
        footprint_checker::CollisionCheckerMsg::Response &resp);


    private:
        /**
         * Copy Ctor.
         */
        BaseCollisionChecker(const BaseCollisionChecker &other);

        /**
         * Assignment operator
         */
        BaseCollisionChecker &operator=(const BaseCollisionChecker &other);

        /**
        * Callback to set input costmap
        */
        void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    private:
        /**
         * An instance of the ROS Node handle.
         */
        ros::NodeHandle nh_;

        /**
         * Flag to check if costmap is received.
         */
        bool is_costmap_received_;

        /**
         * Subscribers and Publishers
         */
        ros::Subscriber costmap_sub_;
        ros::Publisher footprint_pub_;

        /**
         * Store costmap
         */
        nav_msgs::OccupancyGrid costmap_in_;

        /**
         * Store footprint of the base
         */

        std::vector<std::pair<double,double> > footprint_vector_;

        CollisionChecker collision_checker_;
};

#endif  // MCR_BASE_COLLISION_CHECKER_BASE_COLLISION_CHECKER_H_