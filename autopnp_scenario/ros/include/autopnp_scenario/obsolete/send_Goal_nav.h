#ifndef SEND_GOAL_NAV_HH
#define SEND_GOAL_NAV_HH

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <nav_msgs/OccupancyGrid.h>

//Specify the move_base action which is a ROS action that exposes a high level Interface to the navigation stack
#include <move_base_msgs/MoveBaseAction.h>

//A Simple client implementation of the ActionInterface which supports only one goal at a time.
#include <actionlib/client/simple_action_client.h>


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



class VisiblePoints
	{
		public:
			typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
			VisiblePoints();

			double map_resolution_;		// in [m/cell]
			double robotRadius;		// in [m]
			//double personRadius;	// in [m]

			cv::Point2d map_origin_;	// in [m]
			cv::Mat map_;
			cv::Mat expanded_map_;

			ros::NodeHandle m_n;
			ros::Publisher m_pub;
			ros::Subscriber m_sub;
			void init(ros::NodeHandle nh);
			bool validApproachPosition( Pose robotLocation, Pose potentialApproachPose);
			void updateMapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg);

			move_base_msgs::MoveBaseGoal Move( double X , double Y , double Z );
			move_base_msgs::MoveBaseGoal Move_X( double X );
			move_base_msgs::MoveBaseGoal Move_Y( double Y );

			void Execution(float temp_y);

			template <class T>
			T convertFromMeterToPixelCoordinates(const Pose& pose)
				{
					T val;
					val.x = (pose.x - map_origin_.x)/map_resolution_;
					val.y = (pose.y - map_origin_.y)/map_resolution_;
					return val;
				}

	};


inline VisiblePoints::VisiblePoints()
	{
			map_resolution_ = 0 ;		// in [m/cell]
			robotRadius = 0.4 ;			// in [m]
	}



#endif
