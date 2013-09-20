#include <autopnp_scenario/random_location_finder.h>

/* If you want to run this software smoothly,please->
 * set the approach_path_accessibility_check to false
 * in map_accessibility_analysis_params.yaml from
 * package cob_map_accessibility_analysis and also
 * set the parameter in node_handle_.param("map_accessibility_analysis/approach_path_accessibility_check", approach_path_accessibility_check_, false);
 * to false in map_accessibility_analysis_server.cpp
 * */

//#define __DEBUG_DISPLAYS__

random_location_finder::random_location_finder(std::string name_of_the_action) :
		random_location_action_server_(nh_, name_of_the_action, boost::bind(&random_location_finder::execute_random_location_finder_action_server_, this, _1), false), action_name_(
				name_of_the_action)
{
	//Start action server
	random_location_action_server_.start();
}

cv::Mat random_location_finder::find_random_location_(cv::Mat &original_map_from_goal_definition)
{
	ROS_INFO("444444444444 random location action server 444444444444");
	cv::Mat random_location_finder_map = original_map_from_goal_definition.clone();
	cv::Mat Random_Location_mutable = original_map_from_goal_definition;

	random_location_point_.y =
			rand()
					% (maximum_x_coordinate_value_of_the_room_[room_Number_.back()]
							- minimum_x_coordinate_value_of_the_room_[room_Number_.back()])
					+ minimum_x_coordinate_value_of_the_room_[room_Number_.back()];
	random_location_point_.x =
			rand()
					% (maximum_y_coordinate_value_of_the_room_[room_Number_.back()]
							- minimum_y_coordinate_value_of_the_room_[room_Number_.back()])
					+ minimum_y_coordinate_value_of_the_room_[room_Number_.back()];

	bool loop_checker = true;
	while (loop_checker)
	{
#ifdef __DEBUG_DISPLAYS__
	cv::Mat Debug_image = original_map_from_goal_definition.clone();
	cv::circle( Debug_image , random_location_point_ , 3 , cv::Scalar(255), -1 );
	cv::imshow( "find random location", Debug_image );
	cv::waitKey(100);
#endif
		/* To check the dynamic obstacles the map
		 * accessibility analysis service server
		 * is called
		 * */
		std::string points_service_name_ = "/map_accessibility_analysis/map_points_accessibility_check";
		cob_map_accessibility_analysis::CheckPointAccessibility::Request req_points_;
		cob_map_accessibility_analysis::CheckPointAccessibility::Response res_points_;
		geometry_msgs::Pose2D point_to_be_observed_;
		point_to_be_observed_.x = (random_location_point_.x * map_resolution_) + map_origin_.x;
		point_to_be_observed_.y = (random_location_point_.y * map_resolution_) + map_origin_.y;
		req_points_.points_to_check.push_back(point_to_be_observed_);

		// this calls the service server to process our request message and put the result into the response message
		// this call is blocking, i.e. this program will not proceed until the service server sends the response
		ros::service::call(points_service_name_, req_points_, res_points_);

		if (random_location_finder_map.at<unsigned char>(random_location_point_) != 0 && res_points_.accessibility_flags[0] == true)
		{
			loop_checker = false;
		}

		else
		{
			random_location_point_.y =
					rand()
							% (maximum_x_coordinate_value_of_the_room_[room_Number_.back()]
									- minimum_x_coordinate_value_of_the_room_[room_Number_.back()])
							+ minimum_x_coordinate_value_of_the_room_[room_Number_.back()];
			random_location_point_.x =
					rand()
							% (maximum_y_coordinate_value_of_the_room_[room_Number_.back()]
									- minimum_y_coordinate_value_of_the_room_[room_Number_.back()])
							+ minimum_y_coordinate_value_of_the_room_[room_Number_.back()];
			loop_checker = true;
		}
	}

	cv::Point center_of_room;
	center_of_room.y = (maximum_x_coordinate_value_of_the_room_[room_Number_.back()] + minimum_x_coordinate_value_of_the_room_[room_Number_.back()]) / 2;
	center_of_room.x = (maximum_y_coordinate_value_of_the_room_[room_Number_.back()] + minimum_y_coordinate_value_of_the_room_[room_Number_.back()]) / 2;

	if (unsuccessful_times_ == 6)
	{
		ROS_INFO("didn't find a random location to go");
		cv::circle(Random_Location_mutable, center_of_room, 3, cv::Scalar(255), -1);
	}
	else
		ROS_INFO("found a random location to go");

	ROS_INFO("444444444444 random location action server 444444444444\n");

	return Random_Location_mutable;
}

void random_location_finder::execute_random_location_finder_action_server_(const autopnp_scenario::RandomLocationFinderGoalConstPtr &goal)
{
	ros::Rate loop_counter(1);

	//get the necessary room information from goal definition from client
	map_resolution_ = goal->map_resolution;
	map_origin_.x = goal->map_origin_x;
	map_origin_.y = goal->map_origin_y;
	unsuccessful_times_ = goal->unsuccess_counter;

	//Initializing the necessary container value
	room_Number_.clear();
	minimum_x_coordinate_value_of_the_room_.clear();
	maximum_x_coordinate_value_of_the_room_.clear();
	minimum_y_coordinate_value_of_the_room_.clear();
	maximum_y_coordinate_value_of_the_room_.clear();

	room_Number_ = goal->room_number;
	minimum_x_coordinate_value_of_the_room_ = goal->room_min_x;
	maximum_x_coordinate_value_of_the_room_ = goal->room_max_x;
	minimum_y_coordinate_value_of_the_room_ = goal->room_min_y;
	maximum_y_coordinate_value_of_the_room_ = goal->room_max_y;

	//converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(goal->input_img, sensor_msgs::image_encodings::MONO8);
	cv::Mat original_img;
	original_img = cv_ptr->image;

	cv::Mat output_img_ = find_random_location_(original_img);

	//converting the cv format in map msg format
	cv_bridge::CvImage cv_image;
	cv_image.header.stamp = ros::Time::now();
	cv_image.encoding = "mono8";
	cv_image.image = output_img_;
	cv_image.toImageMsg(result_.output_img);

	//setting value to the action msgs to publish
	result_.random_location_x = random_location_point_.x;
	result_.random_location_y = random_location_point_.y;

	loop_counter.sleep();

	random_location_action_server_.setSucceeded(result_);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "random_location_finder");
	random_location_finder random_location_finder_object(ros::this_node::getName());
	ROS_INFO("random location action server is initialized.....");
	ros::spin();
	return 0;
}

