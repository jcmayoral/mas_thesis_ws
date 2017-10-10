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


CollisionChecker::CollisionChecker(ros::NodeHandle &nh) :
convert_map_(false), is_footprint_received(false)
{
    nh.param("base_frame", base_frame_, std::string("map"));
    nh.param("scalling_factor", scaling_factor_new_footprint_,3);
    nh.param("max_collisions", max_number_of_vertices_in_collision_,2);
    nh.param("collision_threshold", threshold_,180.0);
    footprint_sub_ = nh.subscribe("/move_base/local_costmap/footprint",1, &CollisionChecker::footprintCB, this);

}


CollisionChecker::~CollisionChecker()
{

}

void CollisionChecker::footprintCB(const geometry_msgs::PolygonStampedConstPtr &msg){
    ROS_WARN_ONCE("TODO footprintCB");
    base_transformed_footprint_.polygon.points.clear();

    for (int i=0; i<msg->polygon.points.size();i++){
        //ROS_INFO_STREAM(msg->polygon.points[i]);
        base_transformed_footprint_.polygon.points.push_back(msg->polygon.points[i]);
    }
    //base_transformed_footprint_ = *msg;
    //ROS_WARN_ONCE("TODO footprintCB");
    is_footprint_received = true;
}

geometry_msgs::Polygon CollisionChecker::getFootprint(){
    return base_transformed_footprint_.polygon;
}

bool CollisionChecker::isBaseInCollision( )
{
	  bool success = false;

	  if (is_footprint_received){
        CollisionChecker::getIntermediateFootprint();
        success = CollisionChecker::checkCells();
	  }

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

void CollisionChecker::getIntermediateFootprint()
{

	  int i = 0;
    int initial_points = base_transformed_footprint_.polygon.points.size();

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
            diffx = (base_transformed_footprint_.polygon.points[i+1].x -
                base_transformed_footprint_.polygon.points[i].x)/scaling_factor_new_footprint_;

            diffy = (base_transformed_footprint_.polygon.points[i+1].y -
                base_transformed_footprint_.polygon.points[i].y)/scaling_factor_new_footprint_;
        }
        else{
            diffx = (base_transformed_footprint_.polygon.points[0].x -
                base_transformed_footprint_.polygon.points[i].x)/scaling_factor_new_footprint_;
            diffy = (base_transformed_footprint_.polygon.points[0].y -
                base_transformed_footprint_.polygon.points[i].y)/scaling_factor_new_footprint_;
        }

        while (cycle <= scaling_factor_new_footprint_){
            double x = base_transformed_footprint_.polygon.points[i].x + (diffx*cycle);
            double y = base_transformed_footprint_.polygon.points[i].y + (diffy*cycle);
            footprint_extended_vector_.push_back(std::make_pair(x,y));
            cycle++;
        }

    	// add third row 1
        i += 1;
    }
}
