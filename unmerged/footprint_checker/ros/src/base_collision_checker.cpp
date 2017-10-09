/**
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 * base_collision_checker_node.cpp
 *
 * Created on: May 16th, 2016
 * Author: Jose
 */
#include <footprint_checker/base_collision_checker.h>
#include <ros/ros.h>

BaseCollisionChecker::BaseCollisionChecker(const ros::NodeHandle &nh):
        nh_(nh), is_costmap_received_(false)
{
    XmlRpc::XmlRpcValue base_footprint;

    if (!nh.getParam("/move_base/trajectory_footprint",base_footprint)){
        ROS_WARN("Not footprint... consider robot as a point");
        footprint_vector_.push_back(std::make_pair(0,0));                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
    }
    else{
        ROS_INFO("Footprint Parameter found");
        ROS_ASSERT(base_footprint.getType() == XmlRpc::XmlRpcValue::TypeArray);
             
        for (int32_t i = 0; i < base_footprint.size(); ++i) 
        {
        ROS_ASSERT(base_footprint[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
        footprint_vector_.push_back(std::make_pair(static_cast<double> (base_footprint[i][0]),
                                                   static_cast<double> (base_footprint[i][1])));
        }
    }

    footprint_pub_ = nh_.advertise<geometry_msgs::Polygon>("footprint_out", 10);
    costmap_sub_ = nh_.subscribe("/move_base/global_costmap/costmap", 1, &BaseCollisionChecker::costMapCallback, this);

    collision_checker_ = CollisionChecker(footprint_vector_,nh);    
    ROS_INFO("State: INIT");
}

BaseCollisionChecker::~BaseCollisionChecker()
{
}

bool BaseCollisionChecker::runService(footprint_checker::CollisionCheckerMsg::Request  &req,
         footprint_checker::CollisionCheckerMsg::Response &resp)
{
    
    if (is_costmap_received_){
        ROS_INFO_STREAM("Request Received Size : " << req.inputPoses.size()<<
                        " from " << req.start << " to " << req.finish);

        collision_checker_.convertMap(costmap_in_);

        for (int i = req.start; i< req.finish;i++){
            ROS_INFO_STREAM(" Footprint "<< i << " Of " << req.finish);
            resp.success.push_back(collision_checker_.isBaseInCollision(req.inputPoses.at(i),
                                                                         resp));
        }
        ROS_INFO("Service Finished");
        return true;
    }
    else{
        ROS_WARN("Costmap Not Received");
        return false;
    }
}

void BaseCollisionChecker::costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    costmap_in_ = *msg;
    is_costmap_received_ = true;
    ROS_INFO_ONCE("Costmap Received");
}
