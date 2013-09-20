#include <iostream>
#include <string>

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <actionlib/server/simple_action_server.h>
#include <autopnp_scenario/ToLocationAction.h>

#include <autopnp_scenario/exploration_algorithm_action_server_library.h>


class to_location_Server
	{
		protected:
			ros::NodeHandle nh_;
			actionlib::SimpleActionServer<autopnp_scenario::ToLocationAction> tl;
			std::string action_name_;
			autopnp_scenario::ToLocationFeedback feedback_;
			autopnp_scenario::ToLocationResult result_;

		public:
			Exploration exploration_obj;

			to_location_Server(std::string name):
				tl(nh_,name,boost::bind(&to_location_Server::executeCB,this,_1),false),action_name_(name)
				{
					tl.start();
				}

			~to_location_Server(void){}

			void executeCB(const autopnp_scenario::ToLocationGoalConstPtr &goal)
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

					std::string Result = exploration_obj.go_to_destination( original_img , goal->CenterPositionX , goal->CenterPositionY );

					result_.resultant = Result ;

					r.sleep();

					tl.setSucceeded(result_);
				}
	};



int main(int argc,char** argv)
	{
		ros::init(argc,argv,"go_to_location");

		to_location_Server TL(ros::this_node::getName());

		ros::spin();

		return 0;
	}

