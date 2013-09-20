#include <autopnp_scenario/trash_bin_detection.h>

//#define __DEBUG_DISPLAYS__

//constructor Initialization
trash_bin_detection::trash_bin_detection(ros::NodeHandle& nh)
: listener_(nh, ros::Duration(40.0))
{
	trash_bin_detection_active_ = false ;
}

//Initialize Trash bin detection to receive necessary raw data
void trash_bin_detection::fiducials_init_(ros::NodeHandle& nh)
{
	ROS_INFO("FiducialsDetectionServer: Waiting to receive data.....");
	fiducials_msg_sub_ = nh.subscribe<cob_object_detection_msgs::DetectionArray>("/fiducials/detect_fiducials", 1, &trash_bin_detection::fiducials_data_callback_,this);
	ROS_INFO("FiducialsDetectionCheck: data received.");
	detect_trash_bin_again_server_ = nh.advertiseService("detect_trash_bin_again_service", &trash_bin_detection::detect_trash_bin_again_callback_,this);
	activate_trash_bin_detection_server_ = nh.advertiseService("activate_trash_bin_detection_service", &trash_bin_detection::activate_trash_bin_detection_callback_, this);
	deactivate_trash_bin_detection_server_ = nh.advertiseService("deactivate_trash_bin_detection_service", &trash_bin_detection::deactivate_trash_bin_detection_callback_, this);
	//trash_bin_poses-> So you have to take the data from this topic
	trash_bin_location_publisher_ = nh.advertise<autopnp_scenario::TrashBinDetection>("trash_bin_poses", 1000 , this);
}

//fiducials topic data call-back
void trash_bin_detection::fiducials_data_callback_(const cob_object_detection_msgs::DetectionArray::ConstPtr& fiducials_msg_data)
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

			if(fiducials_msg_data->detections[loop_counter].label == "tag_0" )
			{
				fiducials_frame_id_ = fiducials_msg_data->detections[loop_counter].header.frame_id;
				pose_with_respect_to_fiducials_frame_id_= fiducials_msg_data->detections[loop_counter].pose;
				trash_bin_pose_estimator_(pose_with_respect_to_fiducials_frame_id_ , fiducials_frame_id_ );
			}
		}
	}
}

//This function process the data to calculate the trash bin location
void trash_bin_detection::trash_bin_pose_estimator_(geometry_msgs::PoseStamped& pose_from_fiducials_frame_id, std::string& frame_id)
{
	if (trash_bin_detection_active_ == false)
	{
		return;
	}
	else
	{
		tf::Stamped<tf::Pose> original_pose(
		tf::Pose(
				tf::Quaternion(
						pose_from_fiducials_frame_id.pose.orientation.x,
						pose_from_fiducials_frame_id.pose.orientation.y,
						pose_from_fiducials_frame_id.pose.orientation.z,
						pose_from_fiducials_frame_id.pose.orientation.w),
				tf::Vector3(
						pose_from_fiducials_frame_id.pose.position.x,
						pose_from_fiducials_frame_id.pose.position.y,
						pose_from_fiducials_frame_id.pose.position.z)),
				ros::Time(0), frame_id);
		tf::Stamped<tf::Pose> transformed_pose;
		try
		{
			listener_.waitForTransform("/map", original_pose.frame_id_, original_pose.stamp_, ros::Duration(5.0));
			listener_.transformPose("/map", original_pose, transformed_pose);
		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("%s",ex.what());
			return;
		}
		pose_with_respect_to_map_.pose.position.x = transformed_pose.getOrigin().x();
		pose_with_respect_to_map_.pose.position.y = transformed_pose.getOrigin().y();
		pose_with_respect_to_map_.pose.position.z = transformed_pose.getOrigin().z();
		pose_with_respect_to_map_.pose.orientation.x = transformed_pose.getRotation().x();
		pose_with_respect_to_map_.pose.orientation.y = transformed_pose.getRotation().y();
		pose_with_respect_to_map_.pose.orientation.z = transformed_pose.getRotation().z();
		pose_with_respect_to_map_.pose.orientation.w = transformed_pose.getRotation().w();
		pose_with_respect_to_map_.header.frame_id = transformed_pose.frame_id_;
		pose_with_respect_to_map_.header.stamp = transformed_pose.stamp_;
		pose_with_respect_to_map_.header.seq = pose_from_fiducials_frame_id.header.seq;

		if (trash_bin_location_storage_.trash_bin_locations.size() > 0)
		{
			for ( unsigned int loop_counter = 0 ; loop_counter < trash_bin_location_storage_.trash_bin_locations.size() ; loop_counter++ )
			{
				if (similarity_checker_(pose_with_respect_to_map_ , trash_bin_location_storage_.trash_bin_locations[loop_counter] , 0.3 ) == true)
				{
					trash_bin_location_storage_.trash_bin_locations[loop_counter] = average_calculator_(pose_with_respect_to_map_,trash_bin_location_storage_.trash_bin_locations[loop_counter]);
				}
				else if(loop_counter == (trash_bin_location_storage_.trash_bin_locations.size() - 1 ))
				{
					trash_bin_location_storage_.trash_bin_locations.push_back(pose_with_respect_to_map_);
				}

			}
		}
		else
		{
			trash_bin_location_storage_.trash_bin_locations.push_back(pose_with_respect_to_map_);
		}

		trash_bin_location_publisher_.publish(trash_bin_location_storage_);
	}
}

bool trash_bin_detection::detect_trash_bin_again_callback_(
		autopnp_scenario::DetectFiducials::Request &req,
		autopnp_scenario::DetectFiducials::Response &res) {

	ROS_INFO("Received request to detect trash bin.....");

	if((tag_label_name_ == req.tag_name))
	{
		res.waste_bin_location = fiducials_pose_ ;
	}

	return true;
}

//This function activate the trash bin detection service
bool trash_bin_detection::activate_trash_bin_detection_callback_(
		autopnp_scenario::ActivateTrashBinDetection::Request &req,
		autopnp_scenario::ActivateTrashBinDetection::Response &res) {

	ROS_INFO("Received request to turn-on trash bin detection.....");
	ROS_INFO("Trash bin detection is turned-on.");

	trash_bin_location_storage_.trash_bin_locations.clear();

	trash_bin_detection_active_ = true;

	return true;
}

//This function deactivate the trash bin detection service
bool trash_bin_detection::deactivate_trash_bin_detection_callback_(
		autopnp_scenario::DeactivateTrashBinDetection::Request &req,
		autopnp_scenario::DeactivateTrashBinDetection::Response &res) {

	ROS_INFO("Received request to turn-off trash bin detection.....");
	ROS_INFO("Trash bin detection is turned-off.");

	trash_bin_detection_active_ = false;

	cob_object_detection_msgs::Detection temp_detection_obj;

	for(unsigned int loop_counter = 0 ; loop_counter < trash_bin_location_storage_.trash_bin_locations.size() ; loop_counter++)
	{
		temp_detection_obj.pose.pose = trash_bin_location_storage_.trash_bin_locations[loop_counter].pose;
		temp_detection_obj.header =  trash_bin_location_storage_.trash_bin_locations[loop_counter].header;
		res.detected_trash_bin_poses.detections.push_back(temp_detection_obj);
	}



	return true;
}

//This function checks the similarity between the two consecutive pose
bool trash_bin_detection::similarity_checker_(geometry_msgs::PoseStamped &present_value, geometry_msgs::PoseStamped &past_value , double difference_value )
{
	if (sqrt((present_value.pose.position.x - past_value.pose.position.x)*(present_value.pose.position.x - past_value.pose.position.x)
			+ (present_value.pose.position.y - past_value.pose.position.y)*(present_value.pose.position.y - past_value.pose.position.y)
			+ (present_value.pose.position.z - past_value.pose.position.z)*(present_value.pose.position.z - past_value.pose.position.z))
			< difference_value )
	{
		return true;
	}
	else
		return false;
}

//This function calculate the average between the two pose
geometry_msgs::PoseStamped trash_bin_detection::average_calculator_(geometry_msgs::PoseStamped &present_value, geometry_msgs::PoseStamped &past_value)
{
	geometry_msgs::PoseStamped average_value;
	average_value.pose.position.x = (present_value.pose.position.x
			+ past_value.pose.position.x) / 2;
	average_value.pose.position.y = (present_value.pose.position.y
			+ past_value.pose.position.y) / 2;
	average_value.pose.position.z = (present_value.pose.position.z
			+ past_value.pose.position.z) / 2;
	average_value.pose.orientation.x = (present_value.pose.orientation.x
			+ past_value.pose.orientation.x) / 2;
	average_value.pose.orientation.y = (present_value.pose.orientation.y
			+ past_value.pose.orientation.y) / 2;
	average_value.pose.orientation.z = (present_value.pose.orientation.z
			+ past_value.pose.orientation.z) / 2;
	average_value.pose.orientation.w = (present_value.pose.orientation.w
			+ past_value.pose.orientation.w) / 2;

	average_value.header = present_value.header;

	return average_value;
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "trash_bin_detection_server");
	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle nh;
	trash_bin_detection bin_detection_object(nh);
	bin_detection_object.fiducials_init_(nh);
	ROS_INFO("Trash Bin detection Server initialized.....");
	ros::spin();
	return 0;
}
