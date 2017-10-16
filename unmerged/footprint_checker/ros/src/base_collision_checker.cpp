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
        nh_(nh), is_point_cloud_received_(false), collision_threshold_(20.0),
        is_footprint_received(false)
{
    orientations_pub_ = nh_.advertise<geometry_msgs::PoseArray>("collisions_orientations", 1);
    //From Local_planner
    footprint_sub_ = nh.subscribe("/move_base/local_costmap/footprint",4, &BaseCollisionChecker::footprintCB, this);
    point_cloud_sub_ = nh_.subscribe("/move_base/DWAPlannerROS/cost_cloud",1, &BaseCollisionChecker::pointCloudCB, this);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("overlap_costmap",2);
    nh.param("collision_checker_threshold", collision_threshold_,30.0);

    footprint_extender_ = FootprintExtender(nh);
    ROS_INFO("State: INIT");
}

BaseCollisionChecker::~BaseCollisionChecker()
{
}

bool BaseCollisionChecker::runService(footprint_checker::CollisionCheckerMsg::Request  &req,
         footprint_checker::CollisionCheckerMsg::Response &resp)
{
    resp.success = false;

    if (is_point_cloud_received_ && is_footprint_received){
        ROS_INFO_STREAM("Request Received");
        footprint_extender_.getIntermediateFootprint(footprint_);
        updatePointCloud();
        ROS_INFO("Service Finished Correctly");
        resp.success = true;
        resp.potential_collisions = collided_poses_array_;
        return true;
    }
    else{
        ROS_WARN("Subscribing Topics Missing Not Received");
        return true;
    }
}

void BaseCollisionChecker::footprintCB(const geometry_msgs::PolygonStampedConstPtr &msg){
    geometry_msgs::PolygonStamped tmp = *msg;
    footprint_ = tmp.polygon;
    is_footprint_received = true;
    ROS_INFO_ONCE("FootprintCB Received");
}

void BaseCollisionChecker::pointCloudCB(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    point_cloud_ = *msg;
    is_point_cloud_received_ = true;
    ROS_INFO_ONCE("PointCloud Received");
}


void BaseCollisionChecker::updatePointCloud(){

    collided_poses_.clear();
    /*for (int i=0; i< point_cloud_.fields.size(); i++){
      ROS_INFO_STREAM("field " << point_cloud_.fields[i]);
    }*/
    std::mutex mtx;           // mutex for critical section
    mtx.lock();

    //needed for add colors to the pointcloud //Approach not used
    //sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_);
    //pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    //sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_, "x");
    //sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_, "y");
    //sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_, "z");
    //sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(point_cloud_, "r");
    //sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(point_cloud_, "g");
    //sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(point_cloud_, "b");
    sensor_msgs::PointCloud2Iterator<float> iter_tc(point_cloud_, "total_cost");

    //pcd_modifier.resize(point_cloud_.height * point_cloud_.width);


    //conversion for native pcl approach
    pcl::PCLPointCloud2 pcl_point_cloud;
    pcl_conversions::toPCL(point_cloud_,pcl_point_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_point_cloud,*temp_cloud);
    //end conversion

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud (temp_cloud);

    int K = 5;

    for (std::vector<std::pair<double,double> >::iterator it = footprint_extender_.footprint_extended_vector_.begin() ;
              it != footprint_extender_.footprint_extended_vector_.end(); ++it){
        //Search
        pcl::PointXYZRGB searchPoint;
        searchPoint.x = it->first;
        searchPoint.y = it->second;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        double partial_cost = 0.0;

        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
              //std::cout << temp_cloud->points[ pointIdxNKNSearch[i] ].x
              //          << " " << temp_cloud->points[ pointIdxNKNSearch[i] ].y
              //          << " " << temp_cloud->points[ pointIdxNKNSearch[i] ].z
              //          << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
              //*(iter_z+pointIdxNKNSearch[i]) = 1;
              partial_cost += *(iter_tc + pointIdxNKNSearch[i]);
              //temp_cloud->points[ pointIdxNKNSearch[i] ].z = 1;
              temp_cloud->points[ pointIdxNKNSearch[i] ].r = 255;
              //ROS_INFO("a");
            }

            partial_cost /= pointIdxNKNSearch.size();
            ROS_DEBUG_STREAM("costs " << partial_cost);
            //if(partial_cost<min_cost){
            if(partial_cost>= collision_threshold_){
              ROS_DEBUG_STREAM("Collision Found in " << searchPoint.x << " , " << searchPoint.y);
              //transformPoint(searchPoint);
              geometry_msgs::Pose tmp_pose;
              tmp_pose.position.x = searchPoint.x;
              tmp_pose.position.y = searchPoint.y;
              tmp_pose.orientation.w = 1.0;
              collided_poses_.push_back(tmp_pose);

              for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
                temp_cloud->points[ pointIdxNKNSearch[i] ].b = 255;
                temp_cloud->points[ pointIdxNKNSearch[i] ].g = 255;
              }
            }
        }
        // End search
    }
    transformAndPublishPoints();
    pcl::toROSMsg(*temp_cloud, point_cloud_);
    point_cloud_pub_.publish(point_cloud_);
    mtx.unlock();
}

void BaseCollisionChecker::transformAndPublishPoints(){

  collided_poses_array_.poses.clear();
  collided_poses_array_.header.frame_id = footprint_extender_.goal_frame_;

  tf::TransformListener tf_listener;
  tf::StampedTransform transform;
  tf_listener.waitForTransform(footprint_extender_.goal_frame_, footprint_extender_.base_frame_, ros::Time(), ros::Duration(0.5));
  //tf_listener.lookupTransform(footprint_extender_.goal_frame_, footprint_extender_.base_frame_,ros::Time(),transform);

  for (std::vector<geometry_msgs::Pose>::iterator it = collided_poses_.begin() ; it != collided_poses_.end(); ++it){
    geometry_msgs::PoseStamped pose_in, pose_out;
    pose_in.header.frame_id = footprint_extender_.base_frame_;
    pose_in.pose = *it;
    tf_listener.transformPose (footprint_extender_.goal_frame_, ros::Time(), pose_in, footprint_extender_.base_frame_, pose_out);

    tf::Quaternion quat = tf::createQuaternionFromYaw(atan2(pose_out.pose.position.y,pose_out.pose.position.x));
    tf::quaternionTFToMsg(quat,pose_out.pose.orientation);
    collided_poses_array_.poses.push_back(pose_out.pose);
    ROS_DEBUG_STREAM("Collision in base_footprint " << pose_out);
  }

  orientations_pub_.publish(collided_poses_array_);

  //transformPose (const std::string &target_frame, const ros::Time &target_time, const geometry_msgs::PoseStamped &pin, const std::string &fixed_frame, geometry_msgs::PoseStamped &pout) const Transform a Stamped Pose Message into the target frame and time This can throw all that lookupTransform can throw as well as tf::InvalidTransform.
  //transformPose (const std::string &target_frame, const geometry_msgs::PoseStamped &stamped_in, geometry_msgs::PoseStamped &stamped_out)

}
