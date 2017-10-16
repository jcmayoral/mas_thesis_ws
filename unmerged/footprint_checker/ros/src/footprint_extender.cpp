/**
 * Copyright [2016] <Bonn-Rhein-Sieg University>
 * base_collision_checker_node.cpp
 *
 * Created on: May 16th, 2016
 * Author: Jose Carlos Mayoral
 */
#include <footprint_checker/footprint_extender.h>
#include <ros/ros.h>
#include <math.h>
using namespace std;

FootprintExtender::FootprintExtender(){

}


FootprintExtender::FootprintExtender(ros::NodeHandle &nh)
{
    nh.param("base_frame", base_frame_, std::string("map"));
    nh.param("goal_frame", goal_frame_, std::string("base_footprint"));
    nh.param("scalling_factor", scaling_factor_new_footprint_,3);
}


FootprintExtender::~FootprintExtender()
{

}

void FootprintExtender::getIntermediateFootprint(geometry_msgs::Polygon polygon){

	  int i = 0;
    int initial_points = polygon.points.size();

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
            diffx = (polygon.points[i+1].x -
                polygon.points[i].x)/scaling_factor_new_footprint_;

            diffy = (polygon.points[i+1].y -
                polygon.points[i].y)/scaling_factor_new_footprint_;
        }
        else{
            diffx = (polygon.points[0].x -
                polygon.points[i].x)/scaling_factor_new_footprint_;
            diffy = (polygon.points[0].y -
                polygon.points[i].y)/scaling_factor_new_footprint_;
        }

        while (cycle <= scaling_factor_new_footprint_){
            double x = polygon.points[i].x + (diffx*cycle);
            double y = polygon.points[i].y + (diffy*cycle);
            footprint_extended_vector_.push_back(std::make_pair(x,y));
            cycle++;
        }

    	// add third row 1
        i += 1;
    }
}
