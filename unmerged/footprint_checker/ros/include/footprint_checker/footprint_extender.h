/**
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 * collision_checker.cpp
 *
 * Created on: May 16th, 2016
 * Author: Jose
 */
#ifndef MCR_BASE_COLLISION_CHECKER_COLLISION_CHECKER_H_
#define MCR_BASE_COLLISION_CHECKER_COLLISION_CHECKER_H_

#include <vector>
#include <string>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_listener.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
#include <footprint_checker/CollisionCheckerMsg.h>
#include <costmap_2d/costmap_2d.h>

class FootprintExtender
{
    public:
        /**
         * Ctor.
         * The constructor takes base footprint
         *
         * @param base_footprint vector containing vertices of the footprint
         */
        FootprintExtender();

        FootprintExtender(ros::NodeHandle &nh);
        /**
         * Dtor.
         */
        virtual ~FootprintExtender();

        /*
        * Calculte intermediate footprints between vertices
        */
        void getIntermediateFootprint(geometry_msgs::Polygon polygon);


        /*
        * Store frames
        */
        std::string base_frame_;
        std::string goal_frame_;

        /*
        * Store footprint extended with intermediate points between vertices
        */
        std::vector<std::pair<double,double> > footprint_extended_vector_;


    private:

        /*
        * Store Scalling factor for number of footprint point
        */
        int scaling_factor_new_footprint_;

};

#endif  // MCR_BASE_COLLISION_CHECKER_COLLISION_CHECKER_H_
