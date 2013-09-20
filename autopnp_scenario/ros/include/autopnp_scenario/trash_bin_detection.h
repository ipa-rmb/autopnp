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
 * \date Date of creation: September 2013
 *
 * \brief
 * Trash Bin Detection-> The program subscribes to the topic
 * /fiducials/detect_fiducials to get the data from trash bin marker
 * detection.Then it process the data to get the Trash Bin position
 * with respect to map coordinate system
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

#ifndef TRASH_BIN_DETECTION_HH
#define TRASH_BIN_DETECTION_HH

#include <cmath>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <autopnp_scenario/DetectFiducials.h>
#include <autopnp_scenario/TrashBinDetection.h>
#include <cob_object_detection_msgs/Detection.h>
#include <cob_object_detection_msgs/DetectionArray.h>
#include <autopnp_scenario/ActivateTrashBinDetection.h>
#include <autopnp_scenario/DeactivateTrashBinDetection.h>

class trash_bin_detection
{
private:
	bool trash_bin_detection_active_;

	std::string tag_label_name_;
	geometry_msgs::PoseStamped fiducials_pose_;

	std::string fiducials_frame_id_;
	std::string image_detection_label_;

	ros::Subscriber fiducials_msg_sub_;
	ros::Publisher trash_bin_location_publisher_;
	ros::ServiceServer detect_trash_bin_again_server_;
	ros::ServiceServer activate_trash_bin_detection_server_;
	ros::ServiceServer deactivate_trash_bin_detection_server_;

	tf::TransformListener listener_;

    geometry_msgs::PoseStamped pose_with_respect_to_fiducials_frame_id_;
	geometry_msgs::PoseStamped pose_with_respect_to_map_;

	autopnp_scenario::TrashBinDetection trash_bin_location_storage_;

	void fiducials_data_callback_(const cob_object_detection_msgs::DetectionArray::ConstPtr& fiducials_msg_data);
	void trash_bin_pose_estimator_(geometry_msgs::PoseStamped& pose_from_fiducials_frame_id, std::string& frame_id);
	bool detect_trash_bin_again_callback_(autopnp_scenario::DetectFiducials::Request &req, autopnp_scenario::DetectFiducials::Response &res);
	bool activate_trash_bin_detection_callback_(autopnp_scenario::ActivateTrashBinDetection::Request &req, autopnp_scenario::ActivateTrashBinDetection::Response &res);
	bool deactivate_trash_bin_detection_callback_(autopnp_scenario::DeactivateTrashBinDetection::Request &req, autopnp_scenario::DeactivateTrashBinDetection::Response &res);
	bool similarity_checker_(geometry_msgs::PoseStamped &present_value, geometry_msgs::PoseStamped &past_value, double difference_value );
	geometry_msgs::PoseStamped average_calculator_(geometry_msgs::PoseStamped &present_value, geometry_msgs::PoseStamped &past_value);

public:
	trash_bin_detection(ros::NodeHandle& nh);
	void fiducials_init_(ros::NodeHandle& nh);
};

#endif
