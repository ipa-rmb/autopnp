#include "ros/ros.h"
#include "autopnp_scenario/CheckPointAccessibility.h"

int main(int argc, char **argv)
	{
		ros::init(argc, argv, "Get_obstacle_Free_Point_client");

		ros::NodeHandle n;
		ros::ServiceClient client = n.serviceClient<autopnp_scenario::CheckPointAccessibility>("search_obstacle_free_point");
		autopnp_scenario::CheckPointAccessibility srv;
		srv.request.points_to_check[0].x = 493;
		srv.request.points_to_check[0].y = 346;

		if (client.call(srv))
			{
				ROS_INFO("Sum: %d", srv.response.accessibility_flag[0]);
			}
		else
			{
				ROS_ERROR("Failed to call service search_obstacle_free_point");
				return 1;
			}

		return 0;
	}
