#include <ros/ros.h>
#include <autopnp_scenario/send_Goal_nav.h>

//Specify the move_base action which is a ROS action that exposes a high level Interface to the navigation stack
//#include <move_base_msgs/MoveBaseAction.h>

//A Simple client implementation of the ActionInterface which supports only one goal at a time.
//#include <actionlib/client/simple_action_client.h>


//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


//move_base_msgs::MoveBaseGoal Move( double X, double Y, double Z );


void VisiblePoints::init(ros::NodeHandle nh)
	{
		m_n = nh;
		m_sub = m_n.subscribe <nav_msgs::OccupancyGrid> ("/map", 1000, &VisiblePoints::updateMapCallback, this);
	}



void VisiblePoints::updateMapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg)
	{
		// copy properties
		map_resolution_ = map_msg->info.resolution;
		map_origin_ = cv::Point2d(map_msg->info.origin.position.x, map_msg->info.origin.position.y);

		// create empty copy of map
		map_ = 255*cv::Mat::ones(map_msg->info.height, map_msg->info.width, CV_8UC1);

		// copy real static map into cv::Mat element-wise
		for (unsigned int v=0, i=0; v<map_msg->info.height; v++)
			{
				for (unsigned int u=0; u<map_msg->info.width; u++, i++)
					{
						if (map_msg->data[i] != 0)											//why the matrix element should be zero??
						map_.at<unsigned char>(v,u) = 0;
					}
			}

		// create the inflated map
		int iterations = (int)(robotRadius/map_resolution_);
		//std::cout << "iterations=" << iterations << std::endl;
		cv::erode(map_, expanded_map_, cv::Mat(), cv::Point(-1,-1), iterations);

		// display maps
		//		cv::imshow("blown up map", expanded_map_);
		//		cv::imshow("map", map_);
		//		cv::waitKey(10);

		std::cout << "\nMap Recieved.\n";

		//MoveBaseClient ac("move_base", true);
		//while(!ac.waitForServer(ros::Duration(5.0)))
			//{
				//ROS_INFO("\nWaiting for the move_base action server to come up");
			//}


		//Pose robotLocation(0,0,0);		//  todo: read in robot location

		//Pose temp;//todo
		//int temp_x = 0;
		//int temp_y = 0;
		//Pose targetPose( temp_x , temp_y , temp_orientation );
		//Pose personLocation(8,9,0);



		Execution(0);



	}



bool VisiblePoints::validApproachPosition(Pose robotLocation, Pose potentialApproachPose)			//just some obstacle, robots coordinates, the next goal pose for the robot
	{
		// convert coordinates to pixels
		cv::Point potentialApproachPosePixel = convertFromMeterToPixelCoordinates<cv::Point>(potentialApproachPose);


		//cv::Point personLocationPixel = convertFromMeterToPixelCoordinates<cv::Point>(personLocation);

		// copy expanded map
		cv::Mat expanded_map_copy = expanded_map_.clone();

		// draw person into map as obstacle
		//int personRadiusPixel = (int)((personRadius+robotRadius)/map_resolution_);

		//std::cout << "cx:" << center.x << "  cy:" << center.y
		std::cout<< "mx:" << map_origin_.x << "  my:" << map_origin_.y << "  resolution:" << map_resolution_ << std::endl;
		//! draws the circle outline or a solid circle in the image
		//CV_EXPORTS_W void circle(Mat& img, Point center, int radius,const Scalar& color, int thickness=1, int lineType=8, int shift=0);

		//cv::circle(expanded_map_copy, personLocationPixel, personRadiusPixel, cv::Scalar(0,0,0,0), -1);

		// display new inflated map
		//cv::imshow("inflated map", expanded_map_with_person);
		//cv::waitKey(10);

		// find the individual connected areas
		std::vector< std::vector<cv::Point> > contours;		// first index=contour index;  second index=point index within contour


		//std::vector<cv::Vec4i> hierarchy;
		//cv::Mat expanded_map_with_person_copy = expanded_map_with_person.clone();
		cv::findContours(expanded_map_copy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

		// display found contours
		cv::drawContours(expanded_map_copy, contours, -1, cv::Scalar(128,128,128,128), 2);
		cv::circle(expanded_map_copy, convertFromMeterToPixelCoordinates<cv::Point>(robotLocation), 5, cv::Scalar(200,200,200,200), -1);
		cv::circle(expanded_map_copy, potentialApproachPosePixel, 5, cv::Scalar(200,200,200,200), -1);
		cv::line(expanded_map_copy, convertFromMeterToPixelCoordinates<cv::Point>(Pose(1.f,0.2f,0.f)), convertFromMeterToPixelCoordinates<cv::Point>(Pose(-1.f,0.2f,0.f)), cv::Scalar(0,0,0,0), 2);
		//cv::line( map_, convertFromMeterToPixelCoordinates<cv::Point>(Pose(1.f,0.2f,0.f)), convertFromMeterToPixelCoordinates<cv::Point>(PoApproachTestse(-1.f,0.2f,0.f)), cv::Scalar(0,0,0,0), 2);
		cv::imshow("contour areas", expanded_map_copy);
		cv::waitKey(50);

		// check whether potentialApproachPose and robotLocation are in the same area (=same contour)
		int contourIndexRobot = -1;
		int contourIndexPotentialApproachPose = -1;

		for (unsigned int i=0; i<contours.size(); i++)
			{
				if (0 <= cv::pointPolygonTest(contours[i], convertFromMeterToPixelCoordinates<cv::Point2f>(potentialApproachPose), false))
					contourIndexPotentialApproachPose = i;
				if (0 <= cv::pointPolygonTest(contours[i], convertFromMeterToPixelCoordinates<cv::Point2f>(robotLocation), false))
					contourIndexRobot = i;
			}

		std::cout << "contourIndexPotentialApproachPose=" << contourIndexPotentialApproachPose << "\t\tcontourIndexRobot=" << contourIndexRobot << std::endl;
		if (contourIndexRobot != contourIndexPotentialApproachPose || (contourIndexRobot==-1 && contourIndexPotentialApproachPose==-1))
			return false;
/*
		// check whether there is an obstacle in direct line of sight between personLocation and potentialApproachPose
		double dx = personLocationPixel.x - potentialApproachPosePixel.x;
		double dy = personLocationPixel.y - potentialApproachPosePixel.y;	{
											ROS_INFO("The approach position is valid.");
										}
		double interpolationSteps = 0.;

		if (dx>=dy) // normalize
			{
				interpolationSteps = dx;
				dy /= dx;
				dx = 1.;
			}

		else
			{
				interpolationSteps = dy;
				dx /= dy;
				dy = 1.;
			}

		for (int i=0; i<interpolationSteps; i++)
			{
				if (map_.at<unsigned char>(potentialApproachPosePixel.y + i*dy, potentialApproachPosePixel.x + i*dx) == 0)  // if there is an obstacle in line of sight (map(y,x)==0)
					return false;
			}
*/
		return true;

	}






int main(int argc, char** argv)
	{

		  ros::init(argc, argv, "send_goal_nav");
		  ros::NodeHandle n;
		  /**
		   * \brief Simple constructor
		   *
		   * Constructs a SingleGoalActionClient and sets up the necessary ros topics for the ActionInterface
		   * \param name The action name. Defines the namespace in which the action communicates
		   * \param spin_thread If true, spins up a thread to service this action's subscriptions. If false,
		   *                    then the user has to call ros::spin() themselves. ,Defaults to True
		   */
		  //tell the action client that we want to spin a thread by default

		  //MoveBaseClient ac("move_base", true);

		  /**
		   * \brief Waits for the ActionServer to connect to this client
		   *
		   * Often, it can take a second for the action server & client to negotiate
		   * a connection, thus, riskitypedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;ng the first few goals to be dropped. This call lets
		   * the user wait until the network connection to the server is negotiated
		   * \param timeout Max time to block before returning. A zero timeout is interpreted as an infinite timeout.
		   * \return True if the server connected in the allocated time. False on timeout

		  //wait for the action server to come up

		  while(!ac.waitForServer(ros::Duration(5.0)))
			{
		    		ROS_INFO("Waiting for the move_base action server to come up");
		  	}
*/
		  ROS_INFO("Sending goal");

		  VisiblePoints My;

		  My.init(n);

		  ros::spin();
/*
		   * \brief Sends a goal to the ActionServer, and also registers callbacks

		   * If a previous goal is already active when this is called. We simply forget
		     about that goal and start tracking the new goal. No cancel requests are made.

		   * Here we create a goal to send to move_base using the move_base_msgs::MoveBaseGoal message type which is
		     included automatically with the MoveBaseAction.h header. We'll just tell the base to move 1 meter forward in the
		     "base_link" coordinate frame. The call to ac.sendGoal will actually push the goal out over the wire to the move_base
		     node for processing.


		  while(ros::ok())
			  {updateMapCallback
				  ac.sendGoal(Move( 0.0, 1.0, 0.0));	{
											ROS_INFO("The approach position is valid.");
										}


				   * \brief Blocks until this goal finishes
				   * \param timeout Max time to block before returning. A zero timeout is interpreted as an infinite timeout.
				   * \return True if the goal finished. False if the goal didn't finish within the allocated timeout


				  ac.waitForResult();

				  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					  {
						  ROS_INFO("The base moved 1 meter in positive x-direction");
					  }

				  else if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
					  {
						  ROS_INFO("The base failed to move 1 meter in positive x-direction");

						  ac.sendGoal( Move ( -1.0, 0.0, 0.0 ));
						  ac.waitForResult();
						  ROS_INFO("The base rotated 90 degree in clockwise direction along z-axis");

						  //if( ac.getState() == actionlib::SimpleGoalState::ACTIVE)
						  ac.sendGoal( Move ( 0.0, 0.0, -1.0 ));
						  ac.waitForResult();
						  ROS_INFO("The base moved 1 meter in negative y-direction");

						  while(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
							  {
								  ac.sendGoal( Move ( 0.0, -1.0, 0.0 ));
								  ROS_INFO("The base moved 1 meter in negative x-direction");
							  }

					  }

				  else if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
					  {
						  ROS_INFO("The base fail to move 1 meter in positive x-direction");
						  ROS_INFO("The base failed to rotate 90 degree in clockwise direction along z-axis");

						  ac.sendGoal( Move( 1.0, 0.0, 0.0 ));
						  ac.waitForResult();
						  ROS_INFO("The base rotated 90 degree in anti-clockwise direction along z-axis");

						  ac.sendGoal( Move ( 0.0, 0.0, 1.0 ));
						  ac.waitForResult();
						  ROS_INFO("The base moved 1 meter in positive y-direction");

						  while(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
						  {
							  ac.sendGoal( Move ( 0.0, 1.0, 0.0 ));
							  ROS_INFO("The base moved 1 meter in positive x-direction");
						  }

					  }
			  }

		  */

		  return 0;
	}



move_base_msgs::MoveBaseGoal VisiblePoints::Move( double X , double Y , double Z )
	{

		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.orientation.z = Z;
		goal.target_pose.pose.position.x = X;
		goal.target_pose.pose.position.y = Y;

		goal.target_pose.pose.orientation.w = 1.0;

		return goal;
	}


move_base_msgs::MoveBaseGoal VisiblePoints::Move_X( double X )
	{

		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		//goal.target_pose.pose.orientation.z = Z;
		goal.target_pose.pose.position.x = X;
		//goal.target_pose.pose.position.y = Y;

		goal.target_pose.pose.orientation.w = 1.0;

		return goal;
	}



move_base_msgs::MoveBaseGoal VisiblePoints::Move_Y( double Y )
	{

		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		//goal.target_pose.pose.orientation.z = Z;
		//goal.target_pose.pose.position.x = X;
		goal.target_pose.pose.position.y = Y;

		goal.target_pose.pose.orientation.w = 1.0;

		return goal;
	}


void VisiblePoints::Execution(float temp_y)
{
			//float counter_y = 0 ;
				//		for ( float temp_y = 0 ; -5 <= temp_y ; temp_y-- )
					//			{
						//		{
			MoveBaseClient ac("move_base", true);
			float temp_x_new = 0 ;
			float counter_x = 0 ;
			while(!ac.waitForServer(ros::Duration(5.0)))
				{
						ROS_INFO("\nWaiting for the move_base action server to come up");
				}
						Pose robotLocation(0,0,0);
						for ( float temp_x = 1 ; temp_x < 7 ; temp_x++ )
								{
									Pose targetpose;
									targetpose.x = temp_x ;
									targetpose.y = temp_y ;
									bool validApproach = validApproachPosition( robotLocation, targetpose );

									if (validApproach == true)
										{
											ROS_INFO("The approach position is valid.");
											std::cout<<"targetpose.x= "<<targetpose.x;
											std::cout<<"\ntargetpose.y= "<<targetpose.y << "\n\n";

											temp_x_new = temp_x - counter_x;
											std::cout<<"temp_x_new: "<<temp_x_new<<"\n\n";
											//float temp_y_new = 0 ;
											//temp_y_new = temp_y + counter_y ;
											double temp_X = (double)temp_x_new ;
											//double temp_Y = (double)temp_y_new ;
											ac.sendGoal(Move_X ( temp_X ));
											ac.waitForResult();
											counter_x++;

											//ac.sendGoal(Move_Y ( temp_Y ));
											//ac.waitForResult();
										}
									else
										{
											ROS_INFO("The approach position is invalid.");
											std::cout<<"targetpose.x= "<<targetpose.x;
											std::cout<<"\ntargetpose.y= "<<targetpose.y<<"\n\n";
										}



								//}
						//}
						//counter_y++;
				}

						ac.sendGoal(Move ( 0.0 , 0.0 , -1.0 ));
						ac.waitForResult();
						for (int iI = 0; iI < 4 ; iI++ )
							{
								ac.sendGoal(Move ( 1.0 , 0.0 , 0.0 ));
								ac.waitForResult();
							}
						ac.sendGoal(Move ( 0.5 , 0.0 , 0.0 ));
						ac.waitForResult();
						ac.sendGoal(Move ( 0.0 , 0.0 , -1.0 ));
						ac.waitForResult();
						for (int iI = 0; iI < 5 ; iI++ )
							{
								ac.sendGoal(Move ( 1.0 , 0.0 , 0.0 ));
								ac.waitForResult();
							}
						ac.sendGoal(Move ( 0.0 , 0.0 , -1.0 ));
						ac.waitForResult();
						for (int iI = 0; iI < 3 ; iI++ )
							{
								ac.sendGoal(Move ( 1.0 , 0.0 , 0.0 ));
								ac.waitForResult();
							}
						ac.sendGoal(Move ( 0.0 , 0.0 , -1.0 ));
						ac.waitForResult();
						for (int iI = 0; iI < 3 ; iI++ )
							{
								ac.sendGoal(Move ( 1.0 , 0.0 , 0.0 ));
								ac.waitForResult();
							}
						ac.sendGoal(Move ( 0.0 , 0.0 , -1.0 ));
						ac.waitForResult();
						for (int iI = 0; iI < 2 ; iI++ )
							{
								ac.sendGoal(Move ( 1.0 , 0.0 , 0.0 ));
								ac.waitForResult();
							}
						ac.sendGoal(Move ( 0.0 , 0.0 , -1.0 ));
						ac.waitForResult();
						for (int iI = 0; iI < 3 ; iI++ )
							{
								ac.sendGoal(Move ( 1.0 , 0.0 , 0.0 ));
								ac.waitForResult();
							}
						ac.sendGoal(Move ( 0.0 , 0.0 , -1.0 ));
						ac.waitForResult();
						for (int iI = 0; iI < 3 ; iI++ )
							{
								ac.sendGoal(Move ( 1.0 , 0.0 , 0.0 ));
								ac.waitForResult();
							}
						ac.sendGoal(Move ( 0.0 , 0.0 , -1.0 ));
						ac.waitForResult();



}



