#include <autopnp_scenario/room_inspection.h>

//#define __DEBUG_DISPLAYS__

room_inspection::room_inspection(std::string name_of_the_action) :
		inspect_room_action_server_object_(nh_, name_of_the_action, boost::bind(&room_inspection::execute_inspect_room_action_server_, this, _1), false), action_name_(name_of_the_action)
{
	//Start action server
	inspect_room_action_server_object_.start();
}

//execute move_base action and receives the goal in pixel
move_base_msgs::MoveBaseGoal room_inspection::move_in_pixel_(int x_coordinate_value_in_meter, int y_coordinate_value_in_meter)
{
	listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform_);

	move_base_msgs::MoveBaseGoal move_base_msgs_obj;
	geometry_msgs::PoseStamped goal_to_move_base_action_server;

	double x_coordinate_value_in_pixel = (x_coordinate_value_in_meter * map_resolution_) + map_origin_.x;
	double y_coordinate_value_in_pixel = (y_coordinate_value_in_meter * map_resolution_) + map_origin_.y;

	double current_x_coordinate_value_in_pixel = transform_.getOrigin().x();
	double current_y_coordinate_value_in_pixel = transform_.getOrigin().y();

	double angel_around_z_axis = atan2((y_coordinate_value_in_pixel - current_y_coordinate_value_in_pixel), (x_coordinate_value_in_pixel - current_x_coordinate_value_in_pixel));

	goal_to_move_base_action_server.header.frame_id = "map";
	goal_to_move_base_action_server.header.stamp = ros::Time::now();
	goal_to_move_base_action_server.pose.position.x = x_coordinate_value_in_pixel;
	goal_to_move_base_action_server.pose.position.y = y_coordinate_value_in_pixel;
	tf::Quaternion quat = tf::createQuaternionFromYaw(angel_around_z_axis);
	tf::quaternionTFToMsg(quat, goal_to_move_base_action_server.pose.orientation);

	move_base_msgs_obj.target_pose = goal_to_move_base_action_server;

	return move_base_msgs_obj;
}

//always keep the robot in front in moving direction and receives the goal in pixel
move_base_msgs::MoveBaseGoal room_inspection::stay_forward_(int x_coordinate_value_in_meter, int y_coordinate_value_in_meter)
{
	listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform_);

	move_base_msgs::MoveBaseGoal move_base_msgs_obj;
	geometry_msgs::PoseStamped goal_to_move_base_action_server;

	double x_coordinate_value_in_pixel = (x_coordinate_value_in_meter * map_resolution_) + map_origin_.x;
	double y_coordinate_value_in_pixel = (y_coordinate_value_in_meter * map_resolution_) + map_origin_.y;

	double current_x_coordinate_value_in_pixel = transform_.getOrigin().x();
	double current_y_coordinate_value_in_pixel = transform_.getOrigin().y();

	double angel_around_z_axis = atan2((y_coordinate_value_in_pixel - current_y_coordinate_value_in_pixel), (x_coordinate_value_in_pixel - current_x_coordinate_value_in_pixel));

	goal_to_move_base_action_server.header.frame_id = "map";
	goal_to_move_base_action_server.header.stamp = ros::Time::now();
	goal_to_move_base_action_server.pose.position.x = current_x_coordinate_value_in_pixel;
	goal_to_move_base_action_server.pose.position.y = current_y_coordinate_value_in_pixel;
	tf::Quaternion quat = tf::createQuaternionFromYaw(angel_around_z_axis);
	tf::quaternionTFToMsg(quat, goal_to_move_base_action_server.pose.orientation);

	move_base_msgs_obj.target_pose = goal_to_move_base_action_server;

	return move_base_msgs_obj;
}

cv::Mat room_inspection::room_inspection_method_(cv::Mat &original_map_from_goal_definiton)
{
	ROS_INFO("555555555555 inspect room action server 555555555555");
	cv::Point pixel_point_next;
	int pixel_value_of_the_room = original_map_from_goal_definiton.at<unsigned char>(center_of_room_[room_number_.back()]);
	int even_odd_loop_checker = 0;
	ros::param::param("/inspect_room_parameter/step_size_to_find_accessible_points_of_the_room_check_", step_size_to_find_accessible_points_of_the_room_, 1);
	int step_size_to_find_accessible_points_of_the_room_in_pixel = (step_size_to_find_accessible_points_of_the_room_/map_resolution_);
	move_base_client_identifier_ move_base_client_object("move_base", true);

	for (int map_column_value = room_min_x_[room_number_.back()]; map_column_value < room_max_x_[room_number_.back()];
			map_column_value = map_column_value + step_size_to_find_accessible_points_of_the_room_in_pixel)
	{
		if (even_odd_loop_checker % 2 == 0)
		{
			for (int map_row_value = room_min_y_[room_number_.back()]; map_row_value < room_max_y_[room_number_.back()];
					map_row_value = map_row_value + step_size_to_find_accessible_points_of_the_room_in_pixel)
			{
				if (original_map_from_goal_definiton.at<unsigned char>(map_column_value, map_row_value) == pixel_value_of_the_room)
				{
					cob_map_accessibility_analysis::CheckPointAccessibility::Request req_points;
					cob_map_accessibility_analysis::CheckPointAccessibility::Response res_points;
					std::string points_service_name = "/map_accessibility_analysis/map_points_accessibility_check";

					geometry_msgs::Pose2D accessible_point;
					accessible_point.x = (map_row_value * map_resolution_) + map_origin_.x;
					accessible_point.y = (map_column_value * map_resolution_) + map_origin_.y;
					req_points.points_to_check.push_back(accessible_point);

					// this calls the service server to process our request message and put the result into the response message
					// this call is blocking, i.e. this program will not proceed until the service server sends the response
//					bool accessible_point_flag = ros::service::call(points_service_name, req_points, res_points);
					ros::service::call(points_service_name, req_points, res_points);

					if (res_points.accessibility_flags[0] == true)
					{
						pixel_point_next.x = map_row_value;
						pixel_point_next.y = map_column_value;

#ifdef __DEBUG_DISPLAYS__
	cv::circle( original_map_from_goal_definiton , pixel_point_next , 3 , cv::Scalar(255), -1 );
	cv::imshow( "Room Inspection", original_map_from_goal_definiton );
	cv::waitKey(100);
#endif

#ifndef __DEBUG_DISPLAYS__
	bool finished_before_timeout = false;
	move_base_client_object.waitForServer();
	move_base_client_object.sendGoal(stay_forward_(map_row_value, map_column_value));
	move_base_client_object.waitForResult();

	move_base_client_object.waitForServer();
	move_base_client_object.sendGoalAndWait(move_in_pixel_(map_row_value, map_column_value),ros::Duration(30.0));
	finished_before_timeout = move_base_client_object.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = move_base_client_object.getState();
		ROS_INFO("move base action for room inspection finished: %s", state.toString().c_str());
	}
	else
	{
		ROS_INFO("move base action for room inspection did not finish before the time out");
	}
#endif
					}
				}
			}

		}

		else
		{
			for (int map_row_value = room_max_y_[room_number_.back()]; map_row_value > room_min_y_[room_number_.back()];
					map_row_value = map_row_value - step_size_to_find_accessible_points_of_the_room_in_pixel)
			{
				if (original_map_from_goal_definiton.at<unsigned char>(map_column_value, map_row_value) == pixel_value_of_the_room)
				{
					cob_map_accessibility_analysis::CheckPointAccessibility::Request req_points;
					cob_map_accessibility_analysis::CheckPointAccessibility::Response res_points;
					std::string points_service_name = "/map_accessibility_analysis/map_points_accessibility_check";

					geometry_msgs::Pose2D accessible_point;
					accessible_point.x = (map_row_value * map_resolution_) + map_origin_.x;
					accessible_point.y = (map_column_value * map_resolution_) + map_origin_.y;
					req_points.points_to_check.push_back(accessible_point);

					// this calls the service server to process our request message and put the result into the response message
					// this call is blocking, i.e. this program will not proceed until the service server sends the response
//					bool accessible_point_flag = ros::service::call(points_service_name, req_points, res_points);

					ros::service::call(points_service_name, req_points, res_points);
					if (res_points.accessibility_flags[0] == true)
					{
						pixel_point_next.x = map_row_value;
						pixel_point_next.y = map_column_value;

#ifdef __DEBUG_DISPLAYS__
	cv::circle( original_map_from_goal_definiton , pixel_point_next , 3 , cv::Scalar(255), -1 );
	cv::imshow( "Room Inspection", original_map_from_goal_definiton );
	cv::waitKey(100);
#endif

#ifndef __DEBUG_DISPLAYS__
	bool finished_before_timeout = false;
	move_base_client_object.waitForServer();
	move_base_client_object.sendGoal(stay_forward_(map_row_value, map_column_value));
	move_base_client_object.waitForResult();

	move_base_client_object.waitForServer();
	move_base_client_object.sendGoalAndWait(move_in_pixel_(map_row_value, map_column_value),ros::Duration(30.0));
	finished_before_timeout = move_base_client_object.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
	actionlib::SimpleClientGoalState state = move_base_client_object.getState();
	ROS_INFO("move base action for room inspection finished: %s", state.toString().c_str());
	}

	else
	{
	ROS_INFO("move base action for room inspection did not finish before the time out");
	}
#endif
					}
				}
			}
		}
		even_odd_loop_checker++;
	}

	cv::circle(original_map_from_goal_definiton, center_of_room_[room_number_.back()], 3, cv::Scalar(255), -1);

	move_base_client_object.waitForServer();
	move_base_client_object.sendGoal(move_in_pixel_(room_center_x_[room_number_.back()], room_center_y_[room_number_.back()]));
	bool timeout = move_base_client_object.waitForResult();
	if (timeout)
	{
		actionlib::SimpleClientGoalState state = move_base_client_object.getState();
		ROS_INFO("move base action for room inspection finished: %s", state.toString().c_str());
	}
	else
		ROS_INFO("move base action for room inspection did not finish before the time out.");
	ROS_INFO("555555555555 inspect room action server 555555555555\n");
	return original_map_from_goal_definiton;
}

void room_inspection::execute_inspect_room_action_server_(const autopnp_scenario::InspectRoomGoalConstPtr &goal)
{
	ros::Rate loop_counter(1);

	//get the necessary room information from goal definition from client
	map_resolution_ = goal->map_resolution;
	map_origin_.x = goal->map_origin_x;
	map_origin_.y = goal->map_origin_y;

	//clearing the memory container used
	room_number_.clear();
	room_center_x_.clear();
	room_center_y_.clear();
	room_min_x_.clear();
	room_max_x_.clear();
	room_min_y_.clear();
	room_max_y_.clear();
	center_of_room_.clear();

	//initialize the container
	room_number_ = goal->room_number;
	room_center_x_ = goal->room_center_x;
	room_center_y_ = goal->room_center_y;
	room_min_x_ = goal->room_min_x;
	room_max_x_ = goal->room_max_x;
	room_min_y_ = goal->room_min_y;
	room_max_y_ = goal->room_max_y;

	cv::Point centroid;
	for( unsigned int i = 0 ; i < room_center_x_.size() ; i++ )
		{
			centroid.x = room_center_x_[i];
			centroid.y = room_center_y_[i];
			center_of_room_.push_back(centroid);
		}

	//converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(goal->input_img, sensor_msgs::image_encodings::MONO8);
	cv::Mat original_img;
	original_img = cv_ptr->image;

	cv::Mat InspectRoomMap;

	InspectRoomMap = room_inspection_method_(original_img);

	loop_counter.sleep();

	//converting the cv format in map msg format
	cv_bridge::CvImage cv_image;
	cv_image.header.stamp = ros::Time::now();
	cv_image.encoding = "mono8";
	cv_image.image = InspectRoomMap;
	cv_image.toImageMsg(result_.output_img);

	//publish result
	inspect_room_action_server_object_.setSucceeded(result_);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "inspect_room");
	room_inspection room_inspection_obj(ros::this_node::getName());
	ROS_INFO("inspect room action server is initialized.....");
	ros::spin();
	return 0;
}

