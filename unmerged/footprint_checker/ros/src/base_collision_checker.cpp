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
    footprint_sub_ = nh.subscribe("/move_base/local_costmap/footprint",4, &BaseCollisionChecker::footprintCB, this);
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

    if (is_costmap_received_ && is_point_cloud_received_ && is_pose_received_ && is_footprint_received){
        ROS_INFO_STREAM("Request Received");

        collision_checker_.convertMap(costmap_in_);

        resp.success = collision_checker_.isBaseInCollision(footprint_);
        updatePointCloud();
        ROS_INFO("Service Finished Correctly");
        return true;
    }
    else{
        ROS_WARN("Subscribing Topics Missing Not Received");
        resp.success = false;
        return false;
    }
}

void BaseCollisionChecker::footprintCB(const geometry_msgs::PolygonStampedConstPtr &msg){
    geometry_msgs::PolygonStamped tmp = *msg;
    footprint_ = tmp.polygon;
    is_footprint_received = true;
    ROS_INFO_ONCE("FootprintCB Received");
}

void BaseCollisionChecker::costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    costmap_in_ = *msg;
    is_costmap_received_ = true;
    ROS_INFO_ONCE("Costmap Received");
}


void BaseCollisionChecker::pointCloudCB(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  ROS_INFO_ONCE("PointCloud Received1");

    point_cloud_ = *msg;
    is_point_cloud_received_ = true;
    ROS_INFO_ONCE("PointCloud Received");
}


void BaseCollisionChecker::updatePointCloud(){

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

    //Search
    //conversion
    pcl::PCLPointCloud2 pcl_point_cloud;
    pcl_conversions::toPCL(point_cloud_,pcl_point_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_point_cloud,*temp_cloud);
    //end conversion

    pcl::PointXYZ searchPoint;
    searchPoint.x = 10.0;// collision_checker_.footprint_extended_vector_[0].first;
    searchPoint.y = 10.0;//collision_checker_.footprint_extended_vector_[0].second;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (temp_cloud);

    int K = 10;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
      {
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
          std::cout << temp_cloud->points[ pointIdxNKNSearch[i] ].x
                    << " " << temp_cloud->points[ pointIdxNKNSearch[i] ].y
                    << " " << temp_cloud->points[ pointIdxNKNSearch[i] ].z
                    << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
      }
      // End search

    ROS_INFO_STREAM(collision_checker_.footprint_extended_vector_.size ());
    for (std::vector<std::pair<double,double> >::iterator it = collision_checker_.footprint_extended_vector_.begin() ;
              it != collision_checker_.footprint_extended_vector_.end(); ++it){
        pcl::PointXYZ searchPoint;
        searchPoint.x = it->first;
        searchPoint.y = it->second;
        ROS_INFO_STREAM(it->first);
        for (; iter_z != iter_z.end(); ++iter_z, ++iter_r){
            //IF condition
            //footprint_extended_vector_[a].first;
            //footprint_extended_vector_[a].second;
            *iter_z = 10;
            *iter_r = 200;
      }
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
