#include "ros/ros.h"
#include <iostream>
#include "autopnp_scenario/CheckPointAccessibility.h"
#include "autopnp_scenario/move_base_location_on_perimeter.h"


bool search(autopnp_scenario::CheckPointAccessibility::Request &req,
			autopnp_scenario::CheckPointAccessibility::Response &res)
	{
		Clean_Pose CP_obj;
		ROS_INFO("Result: ");
		ROS_INFO("Goal Point x : %f",req.points_to_check[0].x);
		ROS_INFO("Goal Point y : %f",req.points_to_check[0].y);
		std::vector<bool> temp;
		temp.clear();
		temp = CP_obj.get_available_points_( req.points_to_check );
		for(uchar i = 0 ; i < temp.size(); i++)
			{
				res.accessibility_flag[i] = temp [i];
			}
		ROS_INFO( "Result : %d",res.accessibility_flag[0]);
		return true;
	}


int main(int argc, char **argv)
	{
		ros::init(argc, argv, "Get_obstacle_Free_Point_Server");
		ros::NodeHandle n;

		ros::ServiceServer service = n.advertiseService("search_obstacle_free_point", search);
		ROS_INFO("Ready to search for a obstacle free point.");
		ros::spin();

		return 0;
	}
