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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <autopnp_scenario/DetectFiducials.h>
#include <cob_object_detection_msgs/Detection.h>
#include <cob_object_detection_msgs/DetectionArray.h>

class detect_trash_bin_again
{
private:
	std::string tag_label_name_;
	geometry_msgs::PoseStamped fiducials_pose_;

	ros::Subscriber fiducials_msg_sub_;
	ros::Publisher trash_bin_location_publisher_;
	ros::NodeHandle node_handle_;
	ros::ServiceServer detect_trash_bin_again_server_;

	autopnp_scenario::DetectFiducials trash_bin_pose_;

	void fiducials_data_callback_(const cob_object_detection_msgs::DetectionArray::ConstPtr& fiducials_msg_data);
	bool detect_trash_bin_again_callback_(autopnp_scenario::DetectFiducials::Request &req, autopnp_scenario::DetectFiducials::Response &res);

public:
	void fiducials_init_(ros::NodeHandle& nh);
};

void detect_trash_bin_again::fiducials_init_(ros::NodeHandle& nh)
{
	ROS_INFO("FiducialsDetectionServer: Waiting to receive data.....");
	fiducials_msg_sub_ = nh.subscribe<cob_object_detection_msgs::DetectionArray>("/fiducials/detect_fiducials", 1, &detect_trash_bin_again::fiducials_data_callback_,this);
	ROS_INFO("FiducialsDetectionCheck: data received.");
	detect_trash_bin_again_server_ = node_handle_.advertiseService("detect_trash_bin_again_service", &detect_trash_bin_again::detect_trash_bin_again_callback_,this);
}

void detect_trash_bin_again::fiducials_data_callback_(const cob_object_detection_msgs::DetectionArray::ConstPtr& fiducials_msg_data)
{
	for (unsigned int loop_counter=0; loop_counter<fiducials_msg_data->detections.size(); loop_counter++)
	{
		if (fiducials_msg_data->detections.size() == 0)
		{
			ROS_INFO("No markers detected.\n");
			return;
		}
		else
		{
			tag_label_name_ = fiducials_msg_data->detections[loop_counter].label;
			fiducials_pose_= fiducials_msg_data->detections[loop_counter].pose;
		}
	}
}

bool detect_trash_bin_again::detect_trash_bin_again_callback_(
		autopnp_scenario::DetectFiducials::Request &req,
		autopnp_scenario::DetectFiducials::Response &res) {

	ROS_INFO("Received request to detect trash bin.....");

	if((tag_label_name_ == req.tag_name))
	{
		res.waste_bin_location = fiducials_pose_ ;
	}

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "detect_trash_bin_again");
	ros::NodeHandle nh;
	detect_trash_bin_again detect_trash_bin_again_obj;
	detect_trash_bin_again_obj.fiducials_init_(nh);
	ROS_INFO("Detect trash Bin again service server initialized.....");
	ros::spin();
	return 0;
}







