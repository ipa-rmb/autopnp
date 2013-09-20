#include "ros/ros.h"

#include "opencv/cv.h"

// services - here you have to include the header file with exactly the same name as your message in the /srv folder (the Message.h is automatically generated from your Message.srv file during compilation)
#include <autopnp_scenario/CheckPointAccessibility.h>
#include <autopnp_scenario/CheckPerimeterAccessibility.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line. For programmatic
	* remappings you can use a different version of init() which takes remappings
	* directly, but for most command-line programs, passing argc and argv is the easiest
	* way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "map_point_accessibility_check_client");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

	std::string points_service_name = "/map_points_accessibility_check";
	std::string perimeter_service_name = "/map_perimeter_accessibility_check";

	// here we wait until the service is available; please use the same service name as the one in the server; you may define a timeout if the service does not show up
	std::cout << "Waiting for service server to become available..." << std::endl;
	bool serviceAvailable = ros::service::waitForService(points_service_name, 5000);
	serviceAvailable &= ros::service::waitForService(perimeter_service_name, 5000);

	// only proceed if the service is available
	if (serviceAvailable == false)
	{
		std::cout << "The services could not be found.\n" << std::endl;
		return -1;
	}
	std::cout << "The service servers are advertised.\n" << std::endl;

	// prepare the request and response messages
	autopnp_scenario::CheckPointAccessibility::Request req_points;
	autopnp_scenario::CheckPointAccessibility::Response res_points;
	autopnp_scenario::CheckPerimeterAccessibility::Request req_perimeter;
	autopnp_scenario::CheckPerimeterAccessibility::Response res_perimeter;

	geometry_msgs::Pose2D point;
	point.x = 1.;
	point.y = 0;
	req_points.points_to_check.push_back(point);
	point.x = -2.;
	point.y = 0;
	req_points.points_to_check.push_back(point);

	// this calls the service server to process our request message and put the result into the response message
	// this call is blocking, i.e. this program will not proceed until the service server sends the response
	bool success = ros::service::call(points_service_name, req_points, res_points);

	if (success == true)
	{
		printf("Points request successful, results:\n");
		for (unsigned int i=0; i<res_points.accessibility_flags.size(); ++i)
			printf(" - (xy)=(%f, %f), accessible=%d\n", req_points.points_to_check[i].x, req_points.points_to_check[i].y, res_points.accessibility_flags[i]);
	}
	else
		std::cout << "The service call for points was not successful.\n" << std::endl;


	req_perimeter.center.x = 0.;
	req_perimeter.center.y = 0.;
	req_perimeter.center.theta = 0.;
	req_perimeter.radius = 1.0;
	req_perimeter.rotational_sampling_step = 10./180. * CV_PI;

	// this calls the service server to process our request message and put the result into the response message
	// this call is blocking, i.e. this program will not proceed until the service server sends the response
	success = ros::service::call(perimeter_service_name, req_perimeter, res_perimeter);

	if (success == true)
	{
		printf("Accessible points on perimeter:\n");
		for (unsigned int i=0; i<res_perimeter.accessible_poses_on_perimeter.size(); ++i)
			printf(" - (xyt)=(%f, %f, %f)\n", res_perimeter.accessible_poses_on_perimeter[i].x, res_perimeter.accessible_poses_on_perimeter[i].y, res_perimeter.accessible_poses_on_perimeter[i].theta);
	}
	else
		std::cout << "The service call for perimeter points was not successful.\n" << std::endl;


	return 0;
}
