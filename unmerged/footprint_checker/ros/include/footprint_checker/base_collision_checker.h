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
#include <pcl/point_cloud.h>

//TO update PointCloud MSGS
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <string>
#include <vector>
#include <mutex>          // std::mutex
#include <footprint_checker/footprint_extender.h>
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
        BaseCollisionChecker(ros::NodeHandle &nh);

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

        /**
        * CallBack to get PointCloud from LocalPlanner
        */

        void pointCloudCB(const sensor_msgs::PointCloud2ConstPtr &msg);

        /**
        * CallBack to get PointCloud from LocalPlanner
        */

        void localizationCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

        /**
        * Footprint to PointCloud
        */
        void updatePointCloud();

        /**
        * Callback to set footprint
        */

        void footprintCB(const geometry_msgs::PolygonStampedConstPtr &msg);


    private:

        /*
        * Store Base transformed Footprint
        */
        geometry_msgs::Polygon footprint_;
        /**
         * An instance of the ROS Node handle.
         */
        ros::NodeHandle nh_;

        /**
         * Flag to check if costmap and point_cloud are received.
         */
        bool is_point_cloud_received_;
        bool is_pose_received_;
        bool is_footprint_received;

        /**
         * Subscribers and Publishers
         */
        ros::Subscriber point_cloud_sub_;
        ros::Subscriber amcl_sub_;
        ros::Publisher footprint_pub_;
        ros::Publisher point_cloud_pub_;
        ros::Subscriber footprint_sub_;
        /*
        * Update for Storing AMCL messages
        */

        geometry_msgs::PoseWithCovarianceStamped current_pose_;

        /**
         * Store costmap
         */
        sensor_msgs::PointCloud2 point_cloud_;


        FootprintExtender footprint_extender_;

        double collision_threshold_;
        geometry_msgs::Quaternion collided_orientations_;

};

#endif  // MCR_BASE_COLLISION_CHECKER_BASE_COLLISION_CHECKER_H_
