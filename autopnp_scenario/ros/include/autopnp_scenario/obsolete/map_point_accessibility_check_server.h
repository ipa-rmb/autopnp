#ifndef CLEANING_POSITION_HH
#define CLEANING_POSITION_HH

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/GridCells.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <autopnp_scenario/CheckPointAccessibility.h>
#include <autopnp_scenario/CheckPerimeterAccessibility.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>


class MapPointAccessibilityCheck
{
public:
	MapPointAccessibilityCheck(ros::NodeHandle nh);

protected:

	// original map initializer
	void mapInit(ros::NodeHandle& nh_map);

	// dynamic obstacles map initializer
	void inflationInit(ros::NodeHandle& nh);

	// map data call-back function to get the original map and also to write original inflated map for the obstacles
	void mapDataCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg_data);

	// to create dynamic obstacles map
	void obstacleDataCallback(const nav_msgs::GridCells::ConstPtr& obstacles_data, const nav_msgs::GridCells::ConstPtr& inflated_obstacles_data);

	// callback for service checking the accessibility of a vector of points
	bool checkPose2DArrayCallback(autopnp_scenario::CheckPointAccessibility::Request &req, autopnp_scenario::CheckPointAccessibility::Response &res);

	// callback for service checking the accessibility of a perimeter around a center point
	bool checkPerimeterCallback(autopnp_scenario::CheckPerimeterAccessibility::Request &req, autopnp_scenario::CheckPerimeterAccessibility::Response &res);

	ros::NodeHandle node_handle_;

	ros::Subscriber map_msg_sub_;		// subscriber to the map topic
	std::string map_topic_name_;		// name of the map topic
	bool map_data_recieved_;			// flag whether the map has already been received by the node

	message_filters::Subscriber<nav_msgs::GridCells> obstacles_sub_;
	message_filters::Subscriber<nav_msgs::GridCells> inflated_obstacles_sub_;
	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::GridCells, nav_msgs::GridCells> InflatedObstaclesSyncPolicy;
	boost::shared_ptr<message_filters::Synchronizer<InflatedObstaclesSyncPolicy> > inflated_obstacles_sub_sync_; //< Synchronizer
	std::string obstacles_topic_name_;				// name of obstacle topic
	std::string inflated_obstacles_topic_name_;		// name of inflated obstacles topic

	ros::ServiceServer map_points_accessibility_check_server_;	// server handling requests for checking the accessibility of a set of points
	ros::ServiceServer map_perimeter_accessibility_check_server_;	// server handling requests for checking the accessibility of any point on the perimeter of a given position

	//Resolution,robot radius,origin of the map
	double inverse_map_resolution_; // in [cell/m]
	double robot_radius_; // in [m]
	cv::Point2d map_origin_; // in [m]

	// maps
	cv::Mat original_map_;
	cv::Mat inflated_original_map_;		// contains only the inflated static obstacles
	cv::Mat inflated_map_;				// contains inflated static and dynamic obstacles

	boost::mutex mutex_inflated_map_;		// mutex for access on inflated_map

};

#endif	//CLEANING_POSITION_HH
