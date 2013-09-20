#include <iostream>

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <actionlib/server/simple_action_server.h>
#include <autopnp_scenario/InspectRoomAction.h>

#include <autopnp_scenario/exploration_algorithm_action_server_library.h>



class Inspect_Room_Server
	{
		protected:
			ros::NodeHandle nh_;
			actionlib::SimpleActionServer<autopnp_scenario::InspectRoomAction> irs_;
			std::string action_name_;
			autopnp_scenario::InspectRoomFeedback feedback_;
			autopnp_scenario::InspectRoomResult result_;

		public:
			Exploration exploration_obj;

			Inspect_Room_Server(std::string name):
				irs_(nh_,name,boost::bind(&Inspect_Room_Server::executeCB,this,_1),false),action_name_(name)
				{
					irs_.start();
				}

			~Inspect_Room_Server(void){}

			void executeCB(const autopnp_scenario::InspectRoomGoalConstPtr &goal)
				{
					ros::Rate r(1);

					cv_bridge::CvImagePtr cv_ptr;
					cv_ptr = cv_bridge::toCvCopy(goal->input_img, sensor_msgs::image_encodings::MONO8);
					cv::Mat original_img;
					original_img = cv_ptr->image;

					cv::Point2d map_origin_;
					map_origin_.x = goal->Map_Origin_x;
					map_origin_.y = goal->Map_Origin_y;

					exploration_obj.Set_Map_Resoloution_and_Origin( goal->map_resolution , map_origin_ );

					std::vector < int > Center_of_Room_x;
					std::vector < int > Center_of_Room_y;

					Center_of_Room_x = goal->room_center_x;
					Center_of_Room_y = goal->room_center_y;
					std::vector <cv::Point> Center_of_Room;
					cv::Point centroid;

					for( unsigned int i = 0 ; i < Center_of_Room_x.size() ; i++ )
						{
							centroid.x = Center_of_Room_x[i];
							centroid.y = Center_of_Room_y[i];
							Center_of_Room.push_back(centroid);
						}

					cv::Mat InspectRoomMap;

					InspectRoomMap = exploration_obj.Room_Inspection(original_img,
																	Center_of_Room,
																	goal->room_number,
																	Center_of_Room_x,
																	Center_of_Room_y,
																	goal->room_min_x,
																	goal->room_max_x,
																	goal->room_min_y,
																	goal->room_max_y);

					r.sleep();

					cv_bridge::CvImage cv_image;
					cv_image.header.stamp = ros::Time::now();
					cv_image.encoding = "mono8";
					cv_image.image = InspectRoomMap;
					cv_image.toImageMsg(result_.output_img);

					irs_.setSucceeded(result_);
				}
	};



int main(int argc,char** argv)
	{
		ros::init(argc,argv,"Inspect_Room");

		Inspect_Room_Server IRS(ros::this_node::getName());

		ros::spin();

		return 0;
	}
