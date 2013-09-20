#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <actionlib/server/simple_action_server.h>
#include <autopnp_scenario/RandomLocationAction.h>

#include <autopnp_scenario/exploration_algorithm_action_server_library.h>


class Random_Location_Server
	{
		protected:
			ros::NodeHandle nh_;
			actionlib::SimpleActionServer<autopnp_scenario::RandomLocationAction> rl;
			std::string action_name_;
			autopnp_scenario::RandomLocationFeedback feedback_;
			autopnp_scenario::RandomLocationResult result_;

		public:
			Exploration exploration_obj;

			Random_Location_Server(std::string name):
				rl(nh_,name,boost::bind(&Random_Location_Server::executeCB,this,_1),false),action_name_(name)
				{
					rl.start();
				}

			~Random_Location_Server(void){}

			void executeCB(const autopnp_scenario::RandomLocationGoalConstPtr &goal)
				{
					ros::Rate r(1);

					cv_bridge::CvImagePtr cv_ptr;
					cv_ptr = cv_bridge::toCvCopy(goal->input_img, sensor_msgs::image_encodings::MONO8);
					cv::Mat original_img;
					original_img = cv_ptr->image;

					cv::Mat RandomLocation_img = exploration_obj.random_location(original_img,
																				goal->room_number,
																				goal->room_min_x,
																				goal->room_max_x,
																				goal->room_min_y,
																				goal->room_max_y,
																				goal->Unsuccess_time);

					cv_bridge::CvImage cv_image;
					cv_image.header.stamp = ros::Time::now();
					cv_image.encoding = "mono8";
					cv_image.image = RandomLocation_img;
					cv_image.toImageMsg(result_.output_img);

					result_.random_location_x = exploration_obj.get_random_location().x;
					result_.random_location_y = exploration_obj.get_random_location().y;

					r.sleep();

					rl.setSucceeded(result_);
				}
	};



int main(int argc,char** argv)
	{
		ros::init(argc,argv,"Find_Random_Location");

		Random_Location_Server RLS(ros::this_node::getName());

		ros::spin();

		return 0;
	}

