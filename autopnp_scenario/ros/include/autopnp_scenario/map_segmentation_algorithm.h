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
 * map segmentation algorithm provides provides header definition for
 * map segmentation action server receives the map from navigation
 * and does the images segmentation and return a segmented map with
 * the following room information:
 * 1. Return Segmented Map
 * 2. Return Original Map Resolution used in Navigation
 * 3. Return Original Map Origin X Coordinate used in Navigation
 * 4. Return Original Map Origin Y Coordinate used in Navigation
 * 5. Return Room Corner information:
 * 		1. Minimum X-Coordinate.
 * 		2. Maximum X-Coordinate.
 * 		3. Minimum Y-Coordinate.
 * 		4. Maximum Y-Coordinate.
 * 6. Return Room Center Information:
 * 		1. Center X-Coordinate.
 * 		2. Center Y_Coordinate.
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

#ifndef MAP_SEGMENTATION_ALGORITHM_HH
#define MAP_SEGMENTATION_ALGORITHM_HH

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include <actionlib/server/simple_action_server.h>

#include <autopnp_scenario/MapSegmentationAction.h>

class segmentation_algorithm
{
private:
	double map_resolution_;		// in [m/cell]

	/* map sampling factor-> The chance of getting proper room configuration depends on this value ------> in [m]
	 * 1. choice of value is very important
	 * 2. slightly large value will give better sampled map
	 * 3. Extra large value will cost more computation time
	 * 4. large value is needed for big map
	 * 5. small value is needed for small map
	 * */
	double map_sampling_factor_;

	double room_area_;

	/* room area factor-> Set the limitation of area of the room -------> in [m]
	 * 1. choice of value is very important
	 * 2. smaller value increase the chance to get every room
	 * */
	double room_area_factor_lower_limit_, room_area_factor_upper_limit_;

	//""""""""""""""""""""Basic room information"""""""""""""
	std::vector<int> room_number_;
	std::vector<int> minimum_x_coordinate_value_of_the_room_;
	std::vector<int> maximum_x_coordinate_value_of_the_room_;
	std::vector<int> minimum_y_coordinate_value_of_the_room_;
	std::vector<int> maximum_y_coordinate_value_of_the_room_;

	std::vector<cv::Point> center_of_room_;
	std::vector<int> x_coordinate_value_of_the_room_center_;
	std::vector<int> y_coordinate_value_of_the_room_center_;
	//"""The evaluated values are in cell[pixel value]"""""""

	std::vector<std::vector<cv::Point> > temporary_contours_;
	std::vector<std::vector<cv::Point> > saved_contours_;
	std::vector<cv::Vec4i> hierarchy_;
	std::vector<cv::Point> black_pixel_;
	std::vector<cv::Point> neighbourhood_pixel_;

	cv::Mat temporary_map_to_get_the_contours_of_the_room_;
	cv::Mat new_map_to_draw_the_saved_contours_of_the_room_;
	cv::Mat expanded_map_;
	cv::Mat contour_map_;
	cv::Mat new_map_with_obstacles_info_;
	cv::Mat complete_map_after_contour_extraction_and_labelling;
	cv::Mat temporary_map_for_replica_padding_purpose;
	cv::Mat bounding_box_map_to_extract_room_info;

	cv::Point2d map_origin_;	// in [m]

	cv::Mat segmented_map_;

	//converter-> Pixel to meter for X coordinate
	double convert_pixel_to_meter_for_x_coordinate_(int pixel_valued_object_x)
	{
		double meter_value_obj_x = (pixel_valued_object_x * map_resolution_) + map_origin_.x;
		return meter_value_obj_x;
	}

	//converter-> Pixel to meter for Y coordinate
	double convert_pixel_to_meter_for_y_coordinate_(int pixel_valued_object_y)
	{
		double meter_value_obj_y = (pixel_valued_object_y * map_resolution_) + map_origin_.y;
		return meter_value_obj_y;
	}

	//This function takes the navigation data and produce segmented map and necessary information regarding room
	cv::Mat Image_Segmentation_method(cv::Mat &Original_Map_from_subscription, double map_resolution_data_from_subscription);

	//This is the execution function used by action server
	void execute_map_segmentation_server(const autopnp_scenario::MapSegmentationGoalConstPtr &goal);

protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<autopnp_scenario::MapSegmentationAction> map_segmentation_action_server_;
	std::string action_name_;
	autopnp_scenario::MapSegmentationFeedback action_feedback_;
	autopnp_scenario::MapSegmentationResult action_result_;

public:
	//Start the map segmentation action server
	segmentation_algorithm(std::string name_of_the_action);

	//Default destructor for the class
	~segmentation_algorithm(void)
	{
	}
};

#endif
