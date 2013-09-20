#include <autopnp_scenario/find_next_unprocessed_room.h>

//#define __DEBUG_DISPLAYS__

unprocessed_room_finder::unprocessed_room_finder(std::string name_of_the_action) :
		next_unprocessed_room_action_server_(nh_, name_of_the_action, boost::bind(&unprocessed_room_finder::execute_find_next_unprocessed_room_action_server, this, _1), false), action_name_(
				name_of_the_action)
{
	//Start action server
	next_unprocessed_room_action_server_.start();
}

void unprocessed_room_finder::find_next_room_(const cv::Mat &map_from_goal_definition)
{
	double robot_distance_From_Center = 10000;
	x_coordinate_of_room_center_position_ = 0, y_coordinate_of_room_center_position_ = 0;
	cv::Point robot_location_in_pixel;

	ROS_INFO("222222222222 find next unprocessed room action server 222222222222");

	//Get the current position of the robot
	listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform_);
	Pose robot_location_in_meter(transform_.getOrigin().x(), transform_.getOrigin().y(), transform_.getRotation().z());
	robot_location_in_pixel = convert_from_meter_to_pixel_coordinates_<cv::Point>(robot_location_in_meter);
	int Pixel_Value = (int)map_from_goal_definition.at<unsigned char>(robot_location_in_pixel);

#ifdef __DEBUG_DISPLAYS__
	cv::Mat debug_image = map_from_goal_definition.clone();
	cv::circle( debug_image , robot_location_in_pixel , 3 , cv::Scalar(255), -1 );
	cv::imshow( "robot location", debug_image );
	cv::waitKey(100);
#endif

	/*
	 * comment: If Pixel_value gives the same value then that means
	 * it is traveling on the same room still
	 */
	ROS_INFO("pixel value in robot location point: %d",Pixel_Value);

	//get the distances between the room center point
	for (unsigned int n = 0; n < center_of_room_.size(); n++)
	{
		if (map_from_goal_definition.at<unsigned char>(center_of_room_[n]) != 255 && map_from_goal_definition.at<unsigned char>(center_of_room_[n]) != 0)
		{
			robot_distance_From_Center = std::min(robot_distance_From_Center,
					std::sqrt(
							((x_coordinate_of_room_center_[n] - robot_location_in_pixel.x) * (x_coordinate_of_room_center_[n] - robot_location_in_pixel.x))
									+ ((y_coordinate_of_room_center_[n] - robot_location_in_pixel.y) * (y_coordinate_of_room_center_[n] - robot_location_in_pixel.y))));
		}
	}

	ROS_INFO("Robot Distance from Nearest Room Location: %f",robot_distance_From_Center);

	//get the nearest room center point from current robot location
	for (unsigned int n = 0; n < center_of_room_.size(); n++)
	{
		if (std::sqrt(
				((x_coordinate_of_room_center_[n] - robot_location_in_pixel.x) * (x_coordinate_of_room_center_[n] - robot_location_in_pixel.x))
						+ ((y_coordinate_of_room_center_[n] - robot_location_in_pixel.y) * (y_coordinate_of_room_center_[n] - robot_location_in_pixel.y)))
				== robot_distance_From_Center)
		{
			x_coordinate_of_room_center_position_ = x_coordinate_of_room_center_[n];
			y_coordinate_of_room_center_position_ = y_coordinate_of_room_center_[n];
			room_number_.push_back(n);
			ROS_INFO("Next room to Visit: %d",room_number_.back());
		}
	}

#ifdef __DEBUG_DISPLAYS__
	cv::Mat debug_image_for_next_room = map_from_goal_definition.clone();
	cv::circle( debug_image_for_next_room , center_of_room_[room_number_.back()] , 3 , cv::Scalar(255), -1 );
	cv::imshow( "Next nearest unprocessed room", debug_image_for_next_room );
	cv::waitKey(100);
#endif

	ROS_INFO("222222222222 find next unprocessed room action server 222222222222\n");
}

void unprocessed_room_finder::execute_find_next_unprocessed_room_action_server(const autopnp_scenario::FindNextUnprocessedRoomGoalConstPtr &goal)
{
	ros::Rate r(1);

	//get the necessary room information from goal definition from client
	map_resolution_ = goal->map_resolution;
	map_origin_.x = goal->map_origin_x;
	map_origin_.y = goal->map_origin_y;

	//initialize the container
	x_coordinate_of_room_center_ = goal->room_center_x;
	y_coordinate_of_room_center_ = goal->room_center_y;

	//set the center of room from room coordinates value
	cv::Point temp_center_point;
	for (unsigned int loop_counter = 0; loop_counter < x_coordinate_of_room_center_.size(); loop_counter++)
	{
		temp_center_point.x = x_coordinate_of_room_center_[loop_counter];
		temp_center_point.y = y_coordinate_of_room_center_[loop_counter];
		center_of_room_.push_back(temp_center_point);
	}

	//convert the image msg to cv mat format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat original_img;
	original_img = cv_ptr_obj->image;

	//run the find next unprocessed room function to get the next unprocessed room
	find_next_room_(original_img);

	r.sleep();

	//set the resultant value for the action server
	result_.room_number = room_number_;
	result_.center_position_x = x_coordinate_of_room_center_position_;
	result_.center_position_y = y_coordinate_of_room_center_position_;

	//when no unprocessed room is left,then clear the room_number and initialize for the new task
	if (room_number_.size() == center_of_room_.size())
	{
		room_number_.clear();
	}

	//publish the action server result
	next_unprocessed_room_action_server_.setSucceeded(result_);

	//clearing the memory container used
	x_coordinate_of_room_center_.clear();
	y_coordinate_of_room_center_.clear();
	center_of_room_.clear();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "find_next_unprocessed_room");
	unprocessed_room_finder unprocessed_room_finder_obj(ros::this_node::getName());
	ROS_INFO("find next unprocessed room action server is initialized.....");
	ros::spin();
	return 0;
}

