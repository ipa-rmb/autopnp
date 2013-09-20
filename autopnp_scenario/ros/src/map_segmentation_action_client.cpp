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
 * map segmentation action client will provide you a demo version about how
 * to use map segmentation action server.it does the following:
 * 1. first it subscribed to the map topic to get the original map massage.
 * 2. then it converts the image into cv image format to send it as a input
 * 	  to the map segmentation algorithm in map segmentation action server to
 * 	  get the segmented map.
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

#include <iostream>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <autopnp_scenario/MapSegmentationAction.h>
#include <nav_msgs/OccupancyGrid.h>

void updateMapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg)
{
	//map resolution in [m/cell]
	double map_resolution_;

	//map origin in meter
	cv::Point2d map_origin_;	// in [m]
	cv::Mat map_;

	map_resolution_ = map_msg->info.resolution;
	map_origin_ = cv::Point2d(map_msg->info.origin.position.x, map_msg->info.origin.position.y);

	// create empty copy of map
	map_ = 255 * cv::Mat::ones(map_msg->info.height, map_msg->info.width, CV_8UC1);

	// copy real static map into cv::Mat element-wise
	for (unsigned int v = 0, i = 0; v < map_msg->info.height; v++)
	{
		for (unsigned int u = 0; u < map_msg->info.width; u++, i++)
		{
			if (map_msg->data[i] != 0)
				map_.at<unsigned char>(v, u) = 0;
		}
	}

	//instantiation of action goal
	autopnp_scenario::MapSegmentationGoal goal;

	//instantiation of action client object
	actionlib::SimpleActionClient<autopnp_scenario::MapSegmentationAction> ac("segment_map", true);

	// wait for the action server to start
	ROS_INFO("Waiting for action server to start.....");

	//will wait for infinite time
	ac.waitForServer();
	ROS_INFO("Action server started, sending goal.....");

	// send a goal to the action
	//converting cv image to msg format
	cv_bridge::CvImage cv_image;
	cv_image.header.stamp = ros::Time::now();
	cv_image.encoding = "mono8";
	cv_image.image = map_;
	cv_image.toImageMsg(goal.input_map);

	goal.map_resolution = map_resolution_;
	goal.map_origin_x = map_origin_.x;
	goal.map_origin_y = map_origin_.y;
	goal.return_format_in_meter = true;
	goal.return_format_in_pixel = true;

	ac.sendGoal(goal);
	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
	}
	else
		ROS_INFO("Action did not finish before the time out.");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_segment_map");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/map", 1000, updateMapCallback);
	ros::spin();
	return 0;
}

