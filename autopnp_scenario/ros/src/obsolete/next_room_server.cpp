#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <actionlib/server/simple_action_server.h>
#include <autopnp_scenario/NextRoomAction.h>

#include <autopnp_scenario/exploration_algorithm_action_server_library.h>


class Next_Room_Server
	{
		protected:
			ros::NodeHandle nh_;
			actionlib::SimpleActionServer<autopnp_scenario::NextRoomAction> nrs;
			std::string action_name_;
			autopnp_scenario::NextRoomFeedback feedback_;
			autopnp_scenario::NextRoomResult result_;

		public:
			Exploration exploration_obj;

			Next_Room_Server(std::string name):
				nrs(nh_,name,boost::bind(&Next_Room_Server::executeCB,this,_1),false),action_name_(name)
				{
					nrs.start();
				}

			~Next_Room_Server(void){}

			void executeCB(const autopnp_scenario::NextRoomGoalConstPtr &goal)
				{
					ros::Rate r(1);

					cv_bridge::CvImagePtr cv_ptr;
					cv_ptr = cv_bridge::toCvCopy(goal->input_img, sensor_msgs::image_encodings::MONO8);
					cv::Mat original_img;
					original_img = cv_ptr->image;

					//double map_resolution_ = goal->map_resolution;
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


					exploration_obj.Find_Next_Room( original_img , Center_of_Room , Center_of_Room_x , Center_of_Room_y );

					r.sleep();

					result_.room_number = exploration_obj.get_room_number();
					result_.CenterPositionX = exploration_obj.get_center_position_x();
					result_.CenterPositionY = exploration_obj.get_center_position_y();

					if( exploration_obj.get_room_number().size() == Center_of_Room.size() )
						{
							exploration_obj.get_room_number().clear();
						}

					nrs.setSucceeded(result_);
				}
	};



int main(int argc,char** argv)
	{
		ros::init(argc,argv,"Find_Next_Room");

		Next_Room_Server NRS(ros::this_node::getName());

		ros::spin();

		return 0;
	}

