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
 * inspect room provides header definition for inspect room action server
 * which check every unprocessed room for available points or accessible
 * areas where the robot can visit
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

#ifndef INSPECT_ROOM_HH
#define INSPECT_ROOM_HH

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <autopnp_scenario/InspectRoomAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <cob_map_accessibility_analysis/CheckPointAccessibility.h>

class room_inspection
{
private:
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_identifier_;

	double map_resolution_;			// in [m/cell]
	double robot_radius_;			// in [m]
	cv::Point2d map_origin_;		// in [m]

	std::vector<cv::Point> center_of_room_;
	std::vector<int> room_number_;
	std::vector<int> room_center_x_;
	std::vector<int> room_center_y_;
	std::vector<int> room_min_x_;
	std::vector<int> room_max_x_;
	std::vector<int> room_min_y_;
	std::vector<int> room_max_y_;

	tf::TransformListener listener_;
	tf::StampedTransform transform_;

	/* step_size_to_find_accessible_points_of_the_room_->
	 * 1. This variable is depends on choice of the user
	 * 2. Lower step size will provide more frequent points
	 * 	  to observe inside a room.Lowest Limit should be
	 * 	  greater or equal to (robot_radius_/map_resolution_).
	 * 	  If the lower limit is < (robot_radius_/map_resolution_)
	 * 	  then you may get some extra points which the robot
	 * 	  has already visited.
	 * 3. Higher step size will give you less observation
	 * 	  points.
	 * 4. good selection of value will be 2*(robot_radius_/map_resolution_)
	 * */
	int step_size_to_find_accessible_points_of_the_room_;

	/* room_inspection_method_->
	 * 1. It finds the available points from the unprocessed room.
	 * 2. check the points against the dynamic obstacles
	 * 3. call move_base action to move the base to the accessible points
	 * */
	cv::Mat room_inspection_method_(cv::Mat &original_map_from_goal_definiton);

	//Move Base Action Server Methods
	move_base_msgs::MoveBaseGoal move_in_pixel_(int x_coordinate_value_in_meter, int y_coordinate_value_in_meter);
	move_base_msgs::MoveBaseGoal stay_forward_(int x_coordinate_value_in_meter, int y_coordinate_value_in_meter);

	//This is the execution function used by action server
	void execute_inspect_room_action_server_(const autopnp_scenario::InspectRoomGoalConstPtr &goal);

protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<autopnp_scenario::InspectRoomAction> inspect_room_action_server_object_;
	std::string action_name_;
	autopnp_scenario::InspectRoomFeedback feedback_;
	autopnp_scenario::InspectRoomResult result_;

public:
	room_inspection(std::string name_of_the_action);
	~room_inspection(void){}
};

#endif

