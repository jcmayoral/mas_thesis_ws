/**
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 * base_collision_checker_node.cpp
 *
 * Created on: May 16th, 2016
 * Author: Jose
 */
#include <footprint_checker/base_collision_checker.h>
#include <ros/ros.h>

BaseCollisionChecker::BaseCollisionChecker(ros::NodeHandle &nh):
        nh_(nh), is_costmap_received_(false), is_point_cloud_received_(false),
        is_pose_received_(false)
{
    footprint_pub_ = nh_.advertise<geometry_msgs::Polygon>("footprint_out", 10);
    costmap_sub_ = nh_.subscribe("/move_base/global_costmap/costmap", 1, &BaseCollisionChecker::costMapCallback, this);
    amcl_sub_ = nh_.subscribe("/amcl_pose", 1, &BaseCollisionChecker::localizationCB, this);

    //From Local_planner
    point_cloud_sub_ = nh_.subscribe("/move_base/DWAPlannerROS/cost_cloud",1, &BaseCollisionChecker::pointCloudCB, this);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("overlap_costmap",2);

    collision_checker_ = CollisionChecker(nh);
    ROS_INFO("State: INIT");
}

BaseCollisionChecker::~BaseCollisionChecker()
{
}

bool BaseCollisionChecker::runService(footprint_checker::CollisionCheckerMsg::Request  &req,
         footprint_checker::CollisionCheckerMsg::Response &resp)
{

    if (is_costmap_received_ && is_point_cloud_received_ && is_pose_received_){
        ROS_INFO_STREAM("Request Received");

        collision_checker_.convertMap(costmap_in_);

        resp.success = collision_checker_.isBaseInCollision();
        resp.polygon_shapes = collision_checker_.getFootprint();
        updatePointCloud(resp.polygon_shapes);
        ROS_INFO("Service Finished Correctly");
        return true;
    }
    else{
        ROS_WARN("Subscribing Topics Missing Not Received");
        resp.success = false;
        return false;
    }
}

void BaseCollisionChecker::costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    costmap_in_ = *msg;
    is_costmap_received_ = true;
    ROS_INFO_ONCE("Costmap Received");
}


void BaseCollisionChecker::pointCloudCB(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    point_cloud_ = *msg;
    is_point_cloud_received_ = true;
    ROS_INFO_ONCE("PointCloud Received");
}

void BaseCollisionChecker::updatePointCloud(const geometry_msgs::Polygon footprint){

    /*for (int i=0; i< point_cloud_.fields.size(); i++){
      ROS_INFO_STREAM("field " << point_cloud_.fields[i]);
    }*/
    std::mutex mtx;           // mutex for critical section
    mtx.lock();

    //needed for add colors to the pointcloud
    sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_);
    pcd_modifier.resize(point_cloud_.height * point_cloud_.width);
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(point_cloud_, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(point_cloud_, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(point_cloud_, "b");

    for (; iter_z != iter_z.end(); ++iter_z, ++iter_r){
      ROS_INFO_STREAM(*iter_z << *iter_r);
      *iter_z = 10;
      *iter_r = 200;
    }

    point_cloud_pub_.publish(point_cloud_);
    mtx.unlock();
}

void BaseCollisionChecker::localizationCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    current_pose_ = *msg;
    is_pose_received_ = true;
    ROS_INFO_ONCE("Localization Received");
}
