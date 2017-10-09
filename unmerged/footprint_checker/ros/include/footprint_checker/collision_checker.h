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


class CollisionChecker
{
    public:
        /**
         * Ctor.
         * The constructor takes base footprint
         *
         * @param base_footprint vector containing vertices of the footprint
         */
        CollisionChecker();

        CollisionChecker(const std::vector<std::pair<double,double> > footprint_vector,
                                  ros::NodeHandle &nh);
        /**
         * Dtor.
         */
        virtual ~CollisionChecker();

        /**
         * This method performs time parameterization of a given trajectory.
         *
         * @param costmap stores current costmap
         * @param target_pose_in stores given target pose
         * @return True if base does not in collision
         */
        bool isBaseInCollision();

        /*
        * Calculate the center of the Footprint
        * @return true if center was found
        */
        bool getFootprintCenter();

        /*
        * Calculte intermediate footprints between vertices
        */
        void getIntermediateFootprint();

        /*
        * Get the new footprint according to the rotation and translation of the final pose
        * @param target_pose_in stores given target pose
        */
        void getTransformedFootprint();

        /*
        * Check target footprint cell
        * @return true if not collision detected
        */
        bool checkCells();

        /*
        * get yaw angle
        * @param target_pose_in stores given target pose
        * @return true if transformation is completed
        */
        void getTargetYaw(geometry_msgs::Pose pose_in, double *yaw);

        /*
        * Convert Map from Ooccupancy Grid to Costmap_2d class
        */

        void convertMap(const nav_msgs::OccupancyGrid& costmap);


        /**
         * Copy ctor.
         */
        CollisionChecker(const CollisionChecker &other);

        /*
        * Store base_frame
        */
        std::string base_frame_;

        /*
        * get Polygon
        */
        geometry_msgs::Polygon getFootprint();

        /**
        * Callback to set footprint
        */

        void footprintCB(const geometry_msgs::PolygonStampedConstPtr &msg);

    private:

        /*
        true is map is already converted
        */
        bool convert_map_;

        /**
        * Store Costmap
        */
        costmap_2d::Costmap2D costmap_;


        /*
         * Store Base transformed Footprint
         */
        geometry_msgs::Polygon base_transformed_footprint_;
        /*
        * Stores Base Footprint Center Coordinates
        */
        geometry_msgs::Pose footprintCenter_;

        /**
         * Store Base Footprint
         */
        std::vector<std::pair<double,double> > footprint_vector_;
        /*
        * Store footprint extended with intermediate points between vertices
        */
        std::vector<std::pair<double,double> > footprint_extended_vector_;

        /*
        * Select the probability for an occupied cell
        */
        double threshold_;

        /*
        * Store Scalling factor for number of footprint point
        */
        int scaling_factor_new_footprint_;

        /*
        * Store maximum number of collisions
        */
        int max_number_of_vertices_in_collision_;
        ros::Subscriber footprint_sub_;


};

#endif  // MCR_BASE_COLLISION_CHECKER_COLLISION_CHECKER_H_
