/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2013 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: care-o-bot
 * \note
 * ROS stack name: autopnp
 * \note
 * ROS package name: autopnp_scenario
 *
 * \author
 * Author: Mohammad Muinul Islam(email:mohammad.islam@ipa.fraunhofer.de)
 *
 * \author
 * Supervised by: Richard Bormann(email:richard.bormann@ipa.fraunhofer.de)
 *
 * \date Date of creation: August 2013
 *
 * \brief
 * go to room location provides header definition for go to
 * room location action server which receives the goal definition
 * via go to room location action client(in smach-state_machine)
 * from NextUnprocessedRoom state. So, it tries to go to the center
 * position of that room and send a success feedback,if it reaches
 * the center of the room successfully. But, if it can't be able to
 * reach room center,then it checks the current robot location.If the
 * robot is inside that room, it also then sends the success feedback.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef GO_TO_ROOM_LOCATION_HH
#define GO_TO_ROOM_LOCATION_HH

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <autopnp_scenario/GoToRoomLocationAction.h>

struct pose
{
	float x_coordinate_value;
	float y_coordinate_value;
	float orientation_around_z_axis; //in degree

	pose()
	{
		x_coordinate_value = 0;
		y_coordinate_value = 0;
		orientation_around_z_axis = 0;
	}

	pose(float x_, float y_, float orientation_)
	{
		x_coordinate_value = x_;
		y_coordinate_value = y_;
		orientation_around_z_axis = orientation_;
	}
};

class go_inside_of_the_room
{
public:
	//redefine a short name for move base client data-type
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_client_datatype;

	double map_resolution_;			// in [m/cell]
	double robot_radius_;			// in [m]
	cv::Point2d map_origin_;		// in [m]
	std::string feedback_about_robot_location_;
	cv::Point robot_location_in_pixel_;
	int goal_room_center_x_, goal_room_center_y_;

	tf::TransformListener listener_;
	tf::StampedTransform transform_;

	template< class T >
	T convert_from_meter_to_pixel_coordinates_(const pose& temp_pose_obj)
	{
		T value;
		value.x = (temp_pose_obj.x_coordinate_value - map_origin_.x) / map_resolution_;
		value.y = (temp_pose_obj.y_coordinate_value - map_origin_.y) / map_resolution_;
		return value;
	}

	//this function will send the robot to room center position
	std::string go_to_room_center_location_(const cv::Mat &original_map_from_goal_definition);

	//Move Base Action Server Methods
	move_base_msgs::MoveBaseGoal move_in_pixel_(int x_coordinate_value_in_meter, int y_coordinate_value_in_meter);
	move_base_msgs::MoveBaseGoal stay_forward_(int x_coordinate_value_in_meter, int y_coordinate_value_in_meter);

	//This is the execution function used by action server
	void execute_go_to_room_location_action_server_(const autopnp_scenario::GoToRoomLocationGoalConstPtr &goal);

protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<autopnp_scenario::GoToRoomLocationAction> go_inside_of_the_room_action_server_;
	std::string action_name_;
	autopnp_scenario::GoToRoomLocationFeedback feedback_;
	autopnp_scenario::GoToRoomLocationResult result_;

public:
	//Start the map segmentation action server
	go_inside_of_the_room(std::string name_of_the_action);

	~go_inside_of_the_room(void)
	{
	}
};

#endif

