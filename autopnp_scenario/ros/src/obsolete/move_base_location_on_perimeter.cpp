#include <autopnp_scenario/obsolete/move_base_location_on_perimeter.h>



//void Clean_Pose::Set_Map_Resoloution_and_Origin( double temp_Map_Res , cv::Point2d temp_map_origin )
//	{
//		map_resolution_ = temp_Map_Res;
//		map_origin_ = temp_map_origin;
//	}



void Clean_Pose::MapDataCallback( const nav_msgs::OccupancyGrid::ConstPtr& map_msg_data)
	{
//		{
//			boost::mutex::scoped_lock lock(mutex_inflation_topic_);

			map_resolution_ = map_msg_data->info.resolution;
			map_origin_ = cv::Point2d(map_msg_data->info.origin.position.x, map_msg_data->info.origin.position.y);
			std::cout<<"\nmap resolution: "<<map_msg_data->info.resolution<<std::endl;

			// create empty copy of map
			original_map_ = 255*cv::Mat::ones(map_msg_data->info.height, map_msg_data->info.width, CV_8UC1);

			// copy real static map into cv::Mat element-wise
			for (unsigned int v=0, i=0; v<map_msg_data->info.height; v++)
				{
					for (unsigned int u=0; u<map_msg_data->info.width; u++, i++)
						{
							if (map_msg_data->data[i] != 0)
							original_map_.at<unsigned char>(v,u) = 0;
						}
				}
//		}
//
//		condition_inflation_topic_.notify_one();
//	}
//
//
//
//void Clean_Pose::original_inflation()
//	{
//		cv::imshow("Original Map", original_map_ );
//		cv::waitKey();

		double inflation_thickness_;
		ros::param::get("/move_base/local_costmap/robot_radius", inflation_thickness_);

		std::cout<<"\nInflation thickness: "<<inflation_thickness_<<std::endl;

		cv::Mat working_map = original_map_.clone();
//		std::cout<<working_map;

		Black_Pixel_x.clear();
		Black_Pixel_y.clear();

		for(int y = 1; y < working_map.cols; y++)
			{
				for(int x = 1; x < working_map.rows; x++)
					{
						if (working_map.at<unsigned char>(x,y) == 0 &&
						   (working_map.at<unsigned char>(x+1,y) == 255||
						    working_map.at<unsigned char>(x,y+1) == 255||
						    working_map.at<unsigned char>(x+1,y+1) == 255||
						    working_map.at<unsigned char>(x-1,y) == 255||
						    working_map.at<unsigned char>(x,y-1) == 255||
						    working_map.at<unsigned char>(x-1,y-1) == 255))
							{
								Black_Pixel_x.push_back(x);
								Black_Pixel_y.push_back(y);
							}
					}
			}


		inflated_original_map_ = original_map_.clone();

		for( unsigned int idx = 0 ; idx < Black_Pixel_x.size(); idx++)
			{
				for(int i = 0; i < (int)(inflation_thickness_/map_resolution_ ); i++)
					{
						for(int j = 0; j < (int)(inflation_thickness_/map_resolution_ ); j++)
							{
								inflated_original_map_.at<unsigned char>( (Black_Pixel_x[idx] + j ) , (Black_Pixel_y[idx] + i) ) = 0;
								inflated_original_map_.at<unsigned char>( (Black_Pixel_x[idx] - j ) , (Black_Pixel_y[idx] + i) ) = 0;
								inflated_original_map_.at<unsigned char>( (Black_Pixel_x[idx] - j ) , (Black_Pixel_y[idx] - i) ) = 0;
								inflated_original_map_.at<unsigned char>( (Black_Pixel_x[idx] + j ) , (Black_Pixel_y[idx] - i) ) = 0;
							}
					}
			}

//		cv::imshow("Inflated Original Map", inflated_original_map_ );
//		cv::waitKey();

		map_data_recieved_ = true;
		ROS_INFO("Map received.");
		map_msg_sub_.shutdown();
	}



void Clean_Pose::MapInit( ros::NodeHandle nh_map )
	{
		map_data_recieved_ = false;
		map_msg_sub_ = nh_map.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &Clean_Pose::MapDataCallback, this);
		ROS_INFO("Waiting to receive map...");
		while (map_data_recieved_ == false)
			{
					ros::spinOnce();
			}
	}



void Clean_Pose::InflationDataCallback( const nav_msgs::GridCells::ConstPtr& obstacles_data,
										const nav_msgs::GridCells::ConstPtr& inflated_obstacles_data)
	{
//	std::cout<<"\ncallback function started."<<std::endl;
		{
			boost::mutex::scoped_lock lock(mutex_inflation_topic_);

			inflated_map_ = inflated_original_map_.clone();
			for(unsigned int i=0; i< obstacles_data->cells.size() ; i++)
				{
					inflated_map_.at<uchar>((obstacles_data->cells[i].y-map_origin_.y)/map_resolution_, (obstacles_data->cells[i].x-map_origin_.x)/map_resolution_) = 0;
				}

			for(unsigned int i=0; i< inflated_obstacles_data->cells.size() ; i++)
				{
					inflated_map_.at<uchar>((inflated_obstacles_data->cells[i].y-map_origin_.y)/map_resolution_, (inflated_obstacles_data->cells[i].x-map_origin_.x)/map_resolution_) = 0;
				}
		}

//		condition_inflation_topic_.notify_one();

//		cv::imshow("Inflated Map", inflated_map_ );
//		cv::waitKey();

//		if(once)
//			{
//				cleaning_pose(493,346);
//				once = false;
//			}

	}



//void Clean_Pose::InflationDataCallback(const nav_msgs::GridCells::ConstPtr& inflated_obstacles_data)
//	{
//		for(unsigned int i=0; i< inflated_obstacles_data->cells.size() ; i++)
//			{
//				inflated_map_.at<uchar>((inflated_obstacles_data->cells[i].y-map_origin_.y)/map_resolution_, (inflated_obstacles_data->cells[i].x-map_origin_.x)/map_resolution_) = 0;
//			}
//	}
//
//
//void Clean_Pose::ObstacleDataCallback(const nav_msgs::GridCells::ConstPtr& obstacles_data)
//	{
//		for(unsigned int i=0; i< obstacles_data->cells.size() ; i++)
//			{
//				inflated_map_.at<uchar>((obstacles_data->cells[i].y-map_origin_.y)/map_resolution_, (obstacles_data->cells[i].x-map_origin_.x)/map_resolution_) = 0;
//			}
//	}



cv::Mat Clean_Pose::get_inflated_map()
	{
		return inflated_map_;
	}



std::vector<bool> Clean_Pose::get_available_points_( std::vector<geometry_msgs::Pose2D>& checking_points )
	{
		std::vector<bool> obstacle_free_points(checking_points.size(), true);

		obstacle_free_points.clear();

		for(unsigned int i = 0 ; i < checking_points.size() ; i++ )
			{
				if (inflated_map_.at<uchar>( checking_points[i].y , checking_points[i].x ) == 0 )
					obstacle_free_points[i] = false;
			}

		return obstacle_free_points;
	}



//bool Clean_Pose::get_available_points( int goal_point_x , int goal_point_y  )
//	{
//		ros::NodeHandle node2;
//
//		ros::Rate r(1);
//
//		MapInit(node2);
//
//		r.sleep();
//
//		std::cout<<"\n Map data callback started";
//
//		ros::Rate r1(1);
//
//		InflationInit(node2);
//
//		ros::spin();
//
//		r1.sleep();
////		if (original_map_ == 0 )
//
////		original_inflation();
//
////		InflationInit(node2);
//
////		inflated_map_ = inflated_original_map_.clone();
////
////
////		ros::Rate r1(1);
////
////		InflationInit(node2);
////
////		ros::spinOnce();
////
////		r1.sleep();
////
////		ros::Rate r2(1);
////
////		ObstacleInit(node2);
////
////		ros::spinOnce();
////
////		r2.sleep();
//
//		bool obstacle_free_point = true ;
//
//		cv::imshow("Get available points",inflated_original_map_);
//		cv::waitKey();
//
////		cv::imshow("Inflated Map",inflated_map_);
////		cv::waitKey();
//
////		std::cout<<inflated_map_;
////		std::cout<<"\npixel value: "<<(int)inflated_map_.at<unsigned char>( 346 , 493 )<<std::endl;
////
////		std::cout<<"\npixel value: "<<(int)inflated_map_.at<unsigned char>( goal_point_x , goal_point_y )<<std::endl;
////
////		if (inflated_map_.at<unsigned char>( goal_point_x , goal_point_y ) == 0 )
////			{
////				obstacle_free_point = false;
////			}
//
//		return obstacle_free_point;
//	}



void Clean_Pose::InflationInit(ros::NodeHandle nh)
	{
		inflation_node_ = nh;

		obstacles_sub_.subscribe(inflation_node_, "/move_base/local_costmap/obstacles", 1);
		inflated_obstacles_sub_.subscribe(inflation_node_, "/move_base/local_costmap/inflated_obstacles", 1);

		inflated_obstacles_sub_sync_ = boost::shared_ptr<message_filters::Synchronizer<InflatedObstaclesSyncPolicy> >(new message_filters::Synchronizer<InflatedObstaclesSyncPolicy>(InflatedObstaclesSyncPolicy(30)));
		inflated_obstacles_sub_sync_->connectInput(obstacles_sub_, inflated_obstacles_sub_);
		inflated_obstacles_sub_sync_->registerCallback(boost::bind(&Clean_Pose::InflationDataCallback, this, _1, _2));
//		ros::spinOnce();
	}



//void Clean_Pose::InflationInit(ros::NodeHandle nh)
//	{
//		inflated_obstacles_sub_ = nh.subscribe<nav_msgs::GridCells>("/move_base/local_costmap/inflated_obstacles", 1, &Clean_Pose::InflationDataCallback, this);
//	}
//
//
//
//void Clean_Pose::ObstacleInit(ros::NodeHandle nh)
//	{
//		obstacles_sub_ = nh.subscribe<nav_msgs::GridCells>("/move_base/local_costmap/obstacles", 1, &Clean_Pose::ObstacleDataCallback, this);
//	}



//void Clean_Pose::cleaning_pose( int center_of_circle_x = 493 , int center_of_circle_y = 346 )
//	{
//		bool found_pose = false;
//
//		cv::Point Clean_Point;
//
//		MoveBaseClient ac("move_base", true);
//
//		for (int angle = 0; angle <= 360 && found_pose == false; angle+=10)
//			{
//				Clean_Point.x = center_of_circle_x + ((( robotRadius * 1.5 ) * cos (( angle * PI / 180.0 ))))/map_resolution_;
//				Clean_Point.y = center_of_circle_y + ((( robotRadius * 1.5 ) * sin (( angle * PI / 180.0 ))))/map_resolution_;
//
//				std::cout<<"\n Pixel value at clean point"<<(int)inflated_map_.at<unsigned char>(Clean_Point);
//
//				if( inflated_map_.at<unsigned char>(Clean_Point)!= 0 )
//					{
//						found_pose = true;
//					}
//			}
//
//		if (found_pose == true)
//			{
//				std::cout <<"\nCenter of room x value: "<<center_of_circle_x;
//				std::cout <<"\nCenter of room y value: "<< center_of_circle_y;
//				std::cout <<"\nClean Point of room x value: "<< Clean_Point.x;
//				std::cout <<"\nClean Point of room y value: "<< Clean_Point.y<<std::endl;
//
//				ac.waitForServer();
//				ac.sendGoal( stay_forward( Clean_Point.x , Clean_Point.y ));
//				ac.waitForResult();
//
//				ac.waitForServer();
//				ac.sendGoal( Move_in_pixel( Clean_Point.x , Clean_Point.y ));
//				ac.waitForResult();
//
//				ac.waitForServer();
//				ac.sendGoal( stay_backward( Clean_Point.x , Clean_Point.y , center_of_circle_x , center_of_circle_y ));
//				ac.waitForResult();
//			}
//
//		else
//			std::cout << "\nNo valid pose found."<< std::endl;
//	}



move_base_msgs::MoveBaseGoal Clean_Pose::Move_in_pixel( int X , int Y  )
	{
		ros::Rate rate(1);
		listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);
		rate.sleep();

		move_base_msgs::MoveBaseGoal Goal;
		geometry_msgs::PoseStamped goal;

		double p = ( X * map_resolution_ ) + map_origin_.x ;
		double q = ( Y * map_resolution_ ) + map_origin_.y ;

		double m = transform.getOrigin().x();
		double n = transform.getOrigin().y();

		double angel = atan2((q-n),(p-m));

		goal.header.frame_id = "map";
		goal.header.stamp = ros::Time::now();

		goal.pose.position.x = p ;
		goal.pose.position.y = q;
		tf::Quaternion quat = tf::createQuaternionFromYaw(angel);
		tf::quaternionTFToMsg(quat, goal.pose.orientation);

		Goal.target_pose = goal;

		return Goal;
	}



move_base_msgs::MoveBaseGoal Clean_Pose::stay_forward( int X , int Y )
	{
		ros::Rate rate(1);
		listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);
		rate.sleep();

		move_base_msgs::MoveBaseGoal Goal;
		geometry_msgs::PoseStamped goal;

		double p = ( X * map_resolution_ ) + map_origin_.x ;
		double q = ( Y * map_resolution_ ) + map_origin_.y ;

		double m = transform.getOrigin().x();
		double n = transform.getOrigin().y();

		double angel = atan2((q-n),(p-m));

		goal.header.frame_id = "map";
		goal.header.stamp = ros::Time::now();

		goal.pose.position.x = m ;
		goal.pose.position.y = n;
		tf::Quaternion quat = tf::createQuaternionFromYaw(angel);
		tf::quaternionTFToMsg(quat, goal.pose.orientation);

		Goal.target_pose = goal;

		return Goal;
	}



move_base_msgs::MoveBaseGoal Clean_Pose::stay_backward( int X , int Y , int CircleCenterX , int CircleCenterY )
	{
		move_base_msgs::MoveBaseGoal Goal;
		geometry_msgs::PoseStamped goal;

		double p = ( X * map_resolution_ ) + map_origin_.x ;
		double q = ( Y * map_resolution_ ) + map_origin_.y ;

		double b = ( CircleCenterX * map_resolution_ ) + map_origin_.x ;
		double v = ( CircleCenterY * map_resolution_ ) + map_origin_.y ;

		double angel = atan2((q - v),(p - b));

		goal.header.frame_id = "map";
		goal.header.stamp = ros::Time::now();

		goal.pose.position.x = p ;
		goal.pose.position.y = q;
		tf::Quaternion quat = tf::createQuaternionFromYaw(angel);
		tf::quaternionTFToMsg(quat, goal.pose.orientation);

		Goal.target_pose = goal;

		return Goal;
	}



int main(int argc, char** argv)
	{
		ros::init(argc, argv, "Check_Points");

		ros::NodeHandle node2;

		ROS_INFO("Sending goal");

		Clean_Pose My;

		My.MapInit(node2);

		My.InflationInit(node2);

		ros::spin();

		return 0;
	}


