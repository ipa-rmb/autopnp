#ifndef EXPLORATION_ALGORITHM_HH
#define EXPLORATION_ALGORITHM_HH

#include <iostream>
#include <complex>
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>

//#include "opencv2/objdetect/objdetect.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include <opencv2/opencv.hpp>
//#include "opencv2/highgui/highgui.hpp"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>


struct Pose
	{
		float x;
		float y;
		float orientation; //in degree

		Pose()
			{
				x = 0;
				y = 0;
				orientation = 0;
			}

		Pose(float x_, float y_, float orientation_)
			{
				x = x_;
				y = y_;
				orientation = orientation_;
			}
	};


/*
enum costmap
	{
		costmap_1 = 1 ,
		costmap_2 ,
		costmap_3 ,
		costmap_4 ,
		costmap_5 ,
		costmap_6 ,
		costmap_7 ,
		costmap_8
	};
*/

class Exploration
	{
		public:
			typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
			Exploration();

			//float temp_x;
			//float temp_valid_x;
			//float temp_y;

			double map_resolution_;		// in [m/cell]
			double robotRadius;			// in [m]
			//double personRadius;		// in [m]

			std::vector < int > Room_min_x;
			std::vector < int > Room_max_x;
			std::vector < int > Room_min_y;
			std::vector < int > Room_max_y;

			cv::Point2d map_origin_;	// in [m]
			cv::Mat map_;
			cv::Mat expanded_map_;
			cv::Mat Segmented_Map;
			cv::Mat Room_Inspection_Map_Show;

			ros::NodeHandle m_n;
			ros::Publisher m_pub;
			ros::Subscriber m_sub;
			tf::TransformListener listener;
			tf::StampedTransform transform;

			std::vector < cv::Point > Center_of_Room;
			std::vector < int > Center_of_Room_x;
			std::vector < int > Center_of_Room_y;

			template <class T>
			T convertFromMeterToPixelCoordinates(const Pose& pose)
				{
					T val;
					val.x = (pose.x - map_origin_.x)/map_resolution_;
					val.y = (pose.y - map_origin_.y)/map_resolution_;
					return val;
				}


			void init(ros::NodeHandle nh);
			bool validApproachPosition( Pose robotLocation, Pose potentialApproachPose);
			void updateMapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg);

			cv::Mat Image_Segmentation( cv::Mat Original_Map );
			void Find_Next_Room( cv::Mat FNR );
			void Room_Inspection( cv::Mat RI , int room_Number );
			bool Obstacle_free_Point( cv::Mat OFP , int x , int y );

			//bool Room_Inspection_Robot_Ground_Clearence( cv::Mat RIRGC , int x , int y , int PixelValue );
			//cv::Point use_costmap ( costmap cost , cv::Mat RI_Cost , int x ,int y );

			move_base_msgs::MoveBaseGoal Move( double X , double Y , double Z );
			move_base_msgs::MoveBaseGoal Move_in_pixel( int X , int Y );

			//void Execution_positive_X(float temp_arg_y);
			//void Execution_negative_X(float temp_arg_y);

	};


inline Exploration::Exploration()
	{
			map_resolution_ = 0 ;		// in [m/cell]
			robotRadius = 0.4 ;			// in [m]
	}


#endif
