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
 * random location finder provides header definition for random location
 * finder action server which receives the goal definition via random location
 * finder action client(in smach-state_machine) from NextUnprocessedRoom
 * state to go to that room when go to room location action server returns
 * a failure message. So, it tries to go to any pose of that room.
 * Depending on the obstacles message, it tries upto 5 times to enter
 * into the room and in the 6th time it stops trying to reach that
 * room and set the room as processed room.
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

#ifndef RANDOM_LOCATION_FINDER_HH
#define RANDOM_LOCATION_FINDER_HH

#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <actionlib/server/simple_action_server.h>
#include <autopnp_scenario/RandomLocationFinderAction.h>
#include <cob_map_accessibility_analysis/CheckPointAccessibility.h>

class random_location_finder
{
private:
	double map_resolution_;			// in [m/cell]
	cv::Point2d map_origin_;		// in [m]

	/* The following information is about the input map
	 * which is necessary to find the next random obstacle free
	 * pose in inside the room.
	 *
	 * room_number-> the last value of the container provides
	 * 				 the info about next room to visit.
	 *
	 * according to the room number other containers value are
	 * evaluated and all has the value in pixel.
	 * */
	std::vector<int> room_Number_;
	std::vector<int> minimum_x_coordinate_value_of_the_room_;
	std::vector<int> maximum_x_coordinate_value_of_the_room_;
	std::vector<int> minimum_y_coordinate_value_of_the_room_;
	std::vector<int> maximum_y_coordinate_value_of_the_room_;

	/* random location pose inside the room which
	 * provides the goal definition(through smach-stat_machine)
	 * for the go to room location action server
	 * */
	cv::Point random_location_point_;

	/* unsuccessful times checks the
	 * unsuccess times. if the value is greater
	 * than five then it change the image
	 * received by the action server to
	 * set the next room as processed room
	 * */
	int unsuccessful_times_;

	/* This is the function which finds
	 * the random location for action server
	 * is the core of action server
	 * */
	cv::Mat find_random_location_(cv::Mat &original_map_from_goal_definition);

	//This is the execution function used by action server
	void execute_random_location_finder_action_server_(const autopnp_scenario::RandomLocationFinderGoalConstPtr &goal);

protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<autopnp_scenario::RandomLocationFinderAction> random_location_action_server_;
	std::string action_name_;
	autopnp_scenario::RandomLocationFinderFeedback feedback_;
	autopnp_scenario::RandomLocationFinderResult result_;

public:
	//Start the random location finder action server
	random_location_finder(std::string name_of_the_action);

	//Default destructor for the class
	~random_location_finder(void)
	{
	}
};

#endif
