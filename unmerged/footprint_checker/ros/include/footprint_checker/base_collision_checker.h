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
#include <geometry_msgs/PoseArray.h>
#include <string>
#include <vector>
#include <mutex>          // std::mutex
#include <tf/transform_listener.h>
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

        /*
        * transform Collided Point in goal_frame to footprint frame
        */

        void transformAndPublishPoints();

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
        * CallBack to get PointCloud from LocalPlanner
        */

        void pointCloudCB(const sensor_msgs::PointCloud2ConstPtr &msg);


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

         /* Subscribers and Publishers
         */
        ros::Subscriber point_cloud_sub_;
        ros::Subscriber footprint_sub_;
        ros::Publisher orientations_pub_;
        ros::Publisher point_cloud_pub_;

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
        std::vector <geometry_msgs::Pose> collided_pose_;

};

#endif  // MCR_BASE_COLLISION_CHECKER_BASE_COLLISION_CHECKER_H_
