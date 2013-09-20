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
 * find next unprocessed room provides header definition for find next
 * unprocessed room action server which receives a segmented map and by
 * extracting data from segmented map it evaluates the nearest room from
 * the current robot location which is not yet processed
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

#ifndef FIND_NEXT_UNPROCESSED_ROOM_HH
#define FIND_NEXT_UNPROCESSED_ROOM_HH

#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>

#include <actionlib/server/simple_action_server.h>
#include <autopnp_scenario/FindNextUnprocessedRoomAction.h>

struct Pose
{
	float x;
	float y;
	float orientation; //in degree

	Pose()
	{
		x = 0;
		y = 0;
		orientation = 0;
	}

	Pose(float x_, float y_, float orientation_)
	{
		x = x_;
		y = y_;
		orientation = orientation_;
	}
};

class unprocessed_room_finder
{
private:

	double map_resolution_;		// in [m/cell]
	cv::Point2d map_origin_;	// in [m]

	int x_coordinate_of_room_center_position_;
	int y_coordinate_of_room_center_position_;

	std::vector<int> room_number_;
	std::vector<cv::Point> center_of_room_;
	std::vector<int> x_coordinate_of_room_center_;
	std::vector<int> y_coordinate_of_room_center_;

	tf::TransformListener listener_;
	tf::StampedTransform transform_;

	template< class T >
	T convert_from_meter_to_pixel_coordinates_(const Pose& pose)
	{
		T value;
		value.x = (pose.x - map_origin_.x) / map_resolution_;
		value.y = (pose.y - map_origin_.y) / map_resolution_;
		return value;
	}

	//The function calculates the nearest unprocessed room from current robot location
	void find_next_room_(const cv::Mat &map_from_goal_definition);

	//This is the execution function used by action server
	void execute_find_next_unprocessed_room_action_server(const autopnp_scenario::FindNextUnprocessedRoomGoalConstPtr &goal);

protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<autopnp_scenario::FindNextUnprocessedRoomAction> next_unprocessed_room_action_server_;
	std::string action_name_;
	autopnp_scenario::FindNextUnprocessedRoomFeedback feedback_;
	autopnp_scenario::FindNextUnprocessedRoomResult result_;

public:
	//Start the map segmentation action server
	unprocessed_room_finder(std::string name_of_the_action);

	//Default destructor for the class
	~unprocessed_room_finder(void)
	{
	}

};
#endif
