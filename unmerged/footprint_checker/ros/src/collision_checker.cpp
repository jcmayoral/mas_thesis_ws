/**
 * Copyright [2016] <Bonn-Rhein-Sieg University>
 * base_collision_checker_node.cpp
 *
 * Created on: May 16th, 2016
 * Author: Jose Carlos Mayoral
 */
#include <footprint_checker/collision_checker.h>
#include <ros/ros.h>
#include <math.h>
using namespace std;

CollisionChecker::CollisionChecker(){

}


CollisionChecker::CollisionChecker(const std::vector<std::pair<double,double> > footprint_vector,
                                   const ros::NodeHandle &nh) :
convert_map_(false)
{
    footprint_vector_ = footprint_vector;
    nh.param("base_frame", base_frame_, std::string("map"));
    nh.param("scalling_factor", scaling_factor_new_footprint_,3);
    nh.param("max_collisions", max_number_of_vertices_in_collision_,2);
    nh.param("collision_threshold", threshold_,180.0);

}


CollisionChecker::~CollisionChecker()
{

}

geometry_msgs::Polygon CollisionChecker::getFootprint(){
  return base_transformed_footprint_;
}

bool CollisionChecker::isBaseInCollision( )
{
	bool success = false;
	int targetCell = 0;

	//Get center of footprint
	bool center = CollisionChecker::getFootprintCenter();

	if (center){
        CollisionChecker::getTransformedFootprint();
        CollisionChecker::getIntermediateFootprint();
        success = CollisionChecker::checkCells();
	}

  //
	return success;
}

void CollisionChecker::convertMap(const nav_msgs::OccupancyGrid& costmap){

    costmap_2d::Costmap2D temporal(costmap.info.height, costmap.info.width,
        costmap.info.resolution, costmap.info.origin.position.x,
        costmap.info.origin.position.y,0);


    for (int i=0; i< costmap.info.height;i++)
        for (int j=0; j< costmap.info.width;j++) {
            unsigned char costmap_value;

            if (costmap.data[i + costmap.info.height*j] == -1){
                costmap_value = 255;
            }
            else{
                costmap_value = (costmap.data[i + costmap.info.height*j]*254) / 100;
            }

            //cout << "new  val " << (int)costmap_value;
            temporal.setCost(j,i,costmap_value);
        }

    costmap_ = temporal;
    convert_map_ = true;
}


bool CollisionChecker::checkCells (){

  int vertices_in_collision = 0;

	for (int a=0; a < footprint_extended_vector_.size();a++)
    {
        unsigned int mx, my;
        double wx,wy;
        int cost;
        int index = 0;

        wx = footprint_extended_vector_[a].first;
        wy = footprint_extended_vector_[a].second;

        //Get Cell Number of the Footprint point
        costmap_.worldToMap(wx,wy, mx, my);
        index = costmap_.getIndex(mx,my);
        cost = (int) costmap_.getCost(mx, my);

        //ROS_INFO_STREAM("cell number " << a << " of " << footprint_extended_vector_.size());
    	if (cost > threshold_)
        {
            if(vertices_in_collision > max_number_of_vertices_in_collision_) {
               //ROS_INFO_STREAM("Cost value in 1D " << (int)(costmap.data[index]) << " at index "  << index );
               //ROS_INFO("Footprint is in collision.");
               return false;
            }
            vertices_in_collision++;
    	}
    }

    return true;
}

void CollisionChecker::getTransformedFootprint(){

	//footprint_translation constains relative translations of the footprint point to the base_link frame
	int i = 0 ;
	double yaw = 0.0;
	double yaw_center = 0.0;

	// Get Angle of rotation
	getTargetYaw(footprintCenter_, &yaw_center);

	// setting new footprint
  base_transformed_footprint_.points.clear();
	geometry_msgs::Point32 point;

	while( i < footprint_vector_.size())
    {

    	// Homogeneous transformation multiplied by the vectors of the footprint
    	//temporal
    	double x,y;
      x = footprint_vector_[i].first;
      y = footprint_vector_[i].second;

		  i ++;

      base_transformed_footprint_.points.push_back(point);
    }
}

bool CollisionChecker::getFootprintCenter()
{
	tf::StampedTransform transform;
    tf::TransformListener* tf_listener = new tf::TransformListener();

    try
        {
        	ros::Time now = ros::Time(0);
            tf_listener->waitForTransform(base_frame_, "base_footprint",
                                          now, ros::Duration(1.0));
            tf_listener->lookupTransform(base_frame_, "base_footprint",
                                          now, transform);
        }
        catch (std::exception &e)
        {
            ROS_ERROR_STREAM("Could not lookup transform to base link frame: " << e.what());
            return 0;
        }
    //Storing center of footprint
    footprintCenter_.position.x = transform.getOrigin().x();
	footprintCenter_.position.y = transform.getOrigin().y();

	//Copy orientation
	tf::quaternionTFToMsg(transform.getRotation().normalize(), footprintCenter_.orientation);
  return 1;

}

void CollisionChecker::getTargetYaw(geometry_msgs::Pose pose_in, double *yaw)
{

    try
        {
            (*yaw) = tf::getYaw(pose_in.orientation);
        }

    catch (std::exception &e)
        {
            ROS_ERROR_STREAM("Could not get angle form pose: " << e.what());
        }

}

void CollisionChecker::getIntermediateFootprint()
{

	int i = 0;
    int initial_points = footprint_vector_.size();

    footprint_extended_vector_.clear();

    //foot_print_vertices_pub_
    geometry_msgs::PoseArray footprint_vertices;
    footprint_vertices.header.frame_id = base_frame_;

    while( i < (initial_points))
    {
        double diffx = 0.0;
        double diffy = 0.0;
        int cycle = 1;

        if (i < initial_points-1){
            diffx = (base_transformed_footprint_.points[i+1].x -
                base_transformed_footprint_.points[i].x)/scaling_factor_new_footprint_;

            diffy = (base_transformed_footprint_.points[i+1].y -
                base_transformed_footprint_.points[i].y)/scaling_factor_new_footprint_;
        }
        else{
            diffx = (base_transformed_footprint_.points[0].x -
                base_transformed_footprint_.points[i].x)/scaling_factor_new_footprint_;
            diffy = (base_transformed_footprint_.points[0].y -
                base_transformed_footprint_.points[i].y)/scaling_factor_new_footprint_;
        }

        while (cycle <= scaling_factor_new_footprint_){
            double x = base_transformed_footprint_.points[i].x + (diffx*cycle);
            double y = base_transformed_footprint_.points[i].y + (diffy*cycle);
            footprint_extended_vector_.push_back(std::make_pair(x,y));
            cycle++;
        }

    	// add third row 1
        i += 1;
    }
}
