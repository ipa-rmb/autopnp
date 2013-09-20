#include <autopnp_scenario/exploration_algorithm_action_server_library.h>



void Exploration::Set_Map_Resoloution_and_Origin( double temp_Map_Res , cv::Point2d temp_map_origin )
	{
		map_resolution_ = temp_Map_Res ;
		map_origin_ = temp_map_origin;
	}



move_base_msgs::MoveBaseGoal Exploration::Move( double X , double Y , double Z )
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



move_base_msgs::MoveBaseGoal Exploration::Move_in_pixel( int X , int Y  )
	{
		listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);

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



move_base_msgs::MoveBaseGoal Exploration::stay_forward( int X , int Y )
	{
		listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);

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



move_base_msgs::MoveBaseGoal Exploration::stay_backward( int X , int Y , int CircleCenterX , int CircleCenterY )
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




cv::Mat Exploration::Image_Segmentation( cv::Mat Original_Map , double map_resolution )
	{
		std::vector< std::vector <cv::Point> > contours_temp;
		std::vector< std::vector <cv::Point> > saved_contours;

		cv::Mat temp = Original_Map.clone();
		cv::Mat Expanded_Map;
		cv::Mat Contour_map;
		std::vector<cv::Vec4i> hierarchy;

		//When mode is needed
		cv::Mat New_map = Original_Map.clone();


		for ( int i = 0 ; i < 1.5  / map_resolution ; i++ )
			{

				double area = 0.0;
				cv::erode(temp, Expanded_Map, cv::Mat(), cv::Point(-1,-1), 1);
				//cv::imshow("Expanded Map", Expanded_Map );
				temp = Expanded_Map;
				Contour_map = Expanded_Map.clone();

				//*********************** Find contours and Draw Contours*******************

				cv::findContours( Contour_map , contours_temp , hierarchy , CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
				cv::drawContours( Contour_map , contours_temp, -1, cv::Scalar( 128 , 128 , 128 , 128 ), 2);
				//cv::imshow("contour areas from Original Image", Contour_map);

				//*********************** Find contours and Draw Contours*******************


				//***********************Print Contour size and pixel positions*****************

				std::cout<<"\n-----------------------------------------\nContour size: "<<contours_temp.size()<<"\n\n"<<"Contour Array: \n";

				for(unsigned int i=0; i<contours_temp.size(); i++)
					{
						for( int idx = 0 ; idx >= 0; idx = hierarchy[idx][0] )
							{
								std::cout << contours_temp[i][idx]  << "  ";
							}
						std::cout << std::endl;
					}

				//***********************Print Contour size and pixel positions*****************


				//***********************Print Area Save Contour in another Matrix Array*********

				std::cout<<"\n"<<"Contour Areas: \n";

				for( int idx = 0 ; idx >= 0; idx = hierarchy[idx][0] )
					{
						area = map_resolution * map_resolution * cv::contourArea(contours_temp[idx]);

						std::cout << "area[" << idx << "]: "<< area << std::endl;

						if( 3.0 < area && area < 40.0 && contours_temp.size() != 0 )
							{
								saved_contours.push_back(contours_temp[idx]);
								std::cout<<"Contour Area Number matched with the condition is: ["<<idx<<"]\n";

								//************Remove the Saved Contour or make the region Black************
								cv::drawContours(temp , contours_temp , idx , cv::Scalar(0) , CV_FILLED , 8 , hierarchy , 2 );
								//************Remove the Saved Contour or make the region Black************
							}
					 }

				//***********************Print Area Save Contour in another Matrix Array*********


				//***********************Print Saved Contour size and Saved Contour Pixel position*********************

				std::cout<<"\n"<<"Saved Contour Areas: \nSaved Contour size: "<<saved_contours.size()<<"\n";

				for(unsigned int i=0; i<saved_contours.size(); i++)
					{
						for(int idx = 0 ; idx >= 0; idx = hierarchy[idx][0] )
							{
								std::cout << saved_contours[i][idx]  << "  ";
							}
						std::cout << std::endl;
					}

				//***********************Print Saved Contour size and Saved Contour Pixel position*********************

				std::cout<<"\n-----------------------------------------\n";

				//cv::waitKey(10);
			}


		//**********************Draw the saved Contours in the clone version of original image***************

		//When mode is needed
		int count = 10;
		//cv::Scalar color_to_fill( rand()% 30 + 128 ) ;  //,rand()%10 +245, rand()%30 + 120, rand()%30 + 200 );
		for( unsigned int idx = 0; idx < saved_contours.size(); idx++ )
			{
				cv::drawContours( New_map , saved_contours , idx , cv::Scalar( count ) , -1 );
				//cv::imshow("New Map with saved contour", New_map);
				count = count + 30 ;
				//count = count + 3 ;
				//cv::imwrite("Wg_mao.png",New_map);
				//cv::waitKey(10);
			}

		//**********************Draw the saved Contours in the clone version of original image***************


		//*********** To draw the Obstacle in the modified map*********************

		cv::Mat New_obstacle_map =New_map.clone();
		std::vector <cv::Point> Black_Pixel;
		for(int y = 0; y < Original_Map.cols; y++)
			{
				for(int x = 0; x < Original_Map.rows; x++)
					{
						if (Original_Map.at<unsigned char>(x,y) == 0 )
							{
								Black_Pixel.push_back(cv::Point( y , x));
							}
					}
			}

		for( unsigned int idx = 0; idx < Black_Pixel.size(); idx++ )
			{
				New_obstacle_map.at<unsigned char>(Black_Pixel[idx]) = 0;
			}
		//cv::imshow("New Map with saved contour and the obstacles", New_obstacle_map);
		//cv::waitKey(10);

		//*********** To draw the Obstacle in the modified map*********************


		//************Replica-Padding to the Image region*************

		cv::Mat Complete_Map = New_obstacle_map.clone() ;
		int y , x = 0;

		cv::Mat New_Filled_Map = New_obstacle_map.clone();
		std::vector <cv::Point> NeighbourHood_Pixel;

		for(int i =0;i<1000;i++){
		for( y = 0 ; y < Original_Map.cols ; y++)
			{
				for( x = 0 ; x < Original_Map.rows ; x++ )
					{
						if ( Complete_Map.at < unsigned char > (x,y) != 0 && Complete_Map.at < unsigned char > (x,y) != 255 )
							{
								//Check every Pixel where its neighborhood is already replaced or not
								if( Complete_Map.at < unsigned char > ( x-1 , y-1 ) == 255 &&
										New_Filled_Map.at < unsigned char > ( x-1 , y-1 ) != Complete_Map.at < unsigned char > ( x , y ))
									{
										NeighbourHood_Pixel.push_back( cv::Point ( y-1 , x-1 ) );
									}

								if ( Complete_Map.at < unsigned char > ( x-1 , y ) == 255 &&
										New_Filled_Map.at < unsigned char > ( x-1 , y ) != Complete_Map.at < unsigned char > ( x , y ))
									{
										NeighbourHood_Pixel.push_back( cv::Point ( y , x-1 ) );
									}

								if ( Complete_Map.at < unsigned char > (x-1 , y+1 ) == 255 &&
										New_Filled_Map.at < unsigned char > ( x-1 , y+1 ) != Complete_Map.at < unsigned char > ( x , y ) )
									{
										NeighbourHood_Pixel.push_back( cv::Point ( y+1 , x-1 ) );
									}

								if ( Complete_Map.at < unsigned char > (x , y-1 ) == 255 &&
										New_Filled_Map.at < unsigned char > ( x , y-1 ) != Complete_Map.at < unsigned char > ( x , y ))
									{
										NeighbourHood_Pixel.push_back( cv::Point ( y-1 , x ) );
									}

								if ( Complete_Map.at < unsigned char > (x , y+1 ) == 255 &&
										New_Filled_Map.at < unsigned char > ( x , y+1 ) != Complete_Map.at < unsigned char > ( x , y ))
									{
										NeighbourHood_Pixel.push_back( cv::Point ( y+1 , x ) );
									}

								if ( Complete_Map.at < unsigned char > ( x+1 , y-1 ) == 255 &&
										New_Filled_Map.at < unsigned char > ( x+1 , y-1 ) != Complete_Map.at < unsigned char > ( x , y ))
									{
										NeighbourHood_Pixel.push_back( cv::Point ( y-1 , x+1 ) );
									}

								if ( Complete_Map.at < unsigned char > ( x+1 , y ) == 255 &&
										New_Filled_Map.at < unsigned char > ( x+1 , y ) != Complete_Map.at < unsigned char > ( x , y ))
									{
										NeighbourHood_Pixel.push_back( cv::Point ( y , x+1 ) );
									}

								if ( Complete_Map.at < unsigned char > ( x+1 , y+1 ) == 255 &&
										New_Filled_Map.at < unsigned char > ( x+1 , y+1 ) != Complete_Map.at < unsigned char > ( x , y ))
									{
										NeighbourHood_Pixel.push_back( cv::Point ( y+1 , x+1 ) );
									}
							}

						for( unsigned int idx = 0; idx < NeighbourHood_Pixel.size() ; idx++ )
							{
								if(NeighbourHood_Pixel.size()!=0)
									New_Filled_Map.at<unsigned char>(NeighbourHood_Pixel[idx]) = Complete_Map.at < unsigned char > (x,y) ;
							}
						NeighbourHood_Pixel.clear();
					}

			}
		//Go for Next Check where the Replaced Pixel are now set as original
		Complete_Map=New_Filled_Map.clone();
		}

		//cv::imshow("New Map with increased contour", New_Filled_Map);
		//cv::waitKey(10);

		//************Replica-Padding to the Image region*************


		//************Bounding Box**********************************

		cv::Mat Bounding_Box_BB;
		Bounding_Box_BB = New_Filled_Map.clone();
		cv::Point pt1, pt2 , centroid ;
		std::vector < double > distance_bt_Centers;

		std::vector < int > min_Y ( 255 , 100000000 );
		std::vector < int > max_Y ( 255 , 0 );
		std::vector < int > min_X ( 255 , 100000000 );
		std::vector < int > max_X ( 255 , 0 );

		for( y = 0 ; y < Original_Map.cols ; y++)
			{
				for( x = 0 ; x < Original_Map.rows ; x++ )
					{
						if ( Bounding_Box_BB.at< unsigned char > (x,y) != 0 &&
							 Bounding_Box_BB.at< unsigned char > (x,y) != 255 )
							{
								min_Y[ New_Filled_Map.at < unsigned char > (x,y) ] = std::min( y , min_Y [ New_Filled_Map.at < unsigned char > (x,y) ] );
								max_Y[ New_Filled_Map.at < unsigned char > (x,y) ] = std::max( y , max_Y [ New_Filled_Map.at < unsigned char > (x,y) ] );
								min_X[ New_Filled_Map.at < unsigned char > (x,y) ] = std::min( x , min_X [ New_Filled_Map.at < unsigned char > (x,y) ] );
								max_X[ New_Filled_Map.at < unsigned char > (x,y) ] = std::max( x , max_X [ New_Filled_Map.at < unsigned char > (x,y) ] );

							}
					}
			}

		for(unsigned int a=0 ; a < min_Y.size(); a++)
			{
				if( min_Y[a] != 100000000 &&
					min_X[a] != 100000000 &&
					max_Y[a] != 0 &&
					max_X[a] != 0)
				{
					pt1.x = min_Y[a];
					pt1.y = min_X[a];
					pt2.x = max_Y[a];
					pt2.y = max_X[a];

					centroid.x = min_Y[a] + (max_Y[a]-min_Y[a]) /2;
					centroid.y = min_X[a] + (max_X[a]-min_X[a]) /2;

					std::cout<<"\nCenter of the bounding Box: [ "<<centroid.x<<" , "<<centroid.y<<" ]\n";

					Room_min_x.push_back(min_X[a]);
					Room_min_y.push_back(min_Y[a]);
					Room_max_x.push_back(max_X[a]);
					Room_max_y.push_back(max_Y[a]);

					Center_of_Room.push_back(centroid);
					Center_of_Room_x.push_back(centroid.x);
					Center_of_Room_y.push_back(centroid.y);

					cv::rectangle(Bounding_Box_BB , pt1, pt2, cv::Scalar(255), 1);
					cv::circle( Bounding_Box_BB , centroid , 3 , cv::Scalar(255), -1);
				}
			}

		//std::cout<<"\n++++++++++++++++++++++++++++++++++++++++++\n";

		for(unsigned int a=0 ; a < Center_of_Room.size(); a++)
			{
				double Center_distance = std::sqrt(((Center_of_Room_x[1]-Center_of_Room_x[a])*(Center_of_Room_x[1]-Center_of_Room_x[a])) +
												   ((Center_of_Room_y[1]-Center_of_Room_y[a])*(Center_of_Room_y[1]-Center_of_Room_y[a])));

				distance_bt_Centers.push_back(Center_distance);
				cv::line(Bounding_Box_BB, Center_of_Room[4], Center_of_Room[a], cv::Scalar(255), 2);
			}

		//std::cout<<"\n+++++++++++++++++++++++++++++++++++++++++++\n";

		//cv::imshow("bounding box", Bounding_Box_BB);
		//cv::waitKey(100);

		//************Bounding Box**********************************


		//************Memory_Clear(Vector-Memory)**********************************

		contours_temp.clear();
		saved_contours.clear();
		hierarchy.clear();
		Black_Pixel.clear();
		NeighbourHood_Pixel.clear();

		//************Memory_Clear(Vector-Memory)**********************************

		return New_Filled_Map.clone();
	}



std::vector<int>& Exploration::get_Center_of_Room_x()
	{
		return Center_of_Room_x;
	}



std::vector<int>& Exploration::get_Center_of_Room_y()
	{
		return Center_of_Room_y;
	}



void Exploration::Find_Next_Room(cv::Mat FNR ,
								 std::vector<cv::Point> center_of_room ,
								 std::vector<int> center_of_room_x ,
								 std::vector<int> center_of_room_y)
	{
		cv::Mat Find_Next_Room_FNR = FNR.clone();
		cv::Mat FNR_Path_direction_temp = FNR.clone();
		cv::Mat FNR_Path_direction ;

		double robot_distance_From_Center = 10000;
		Center_Position_x = 0 ,
		Center_Position_y = 0 ;

		cv::Point robot_location_in_pixel;

		std::cout<<"\n§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§\n";

		//Get the current position of the robot
		listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);

		Pose robot_location_in_meter(transform.getOrigin().x(),transform.getOrigin().y(),transform.getRotation().z());

		robot_location_in_pixel = convertFromMeterToPixelCoordinates < cv::Point > (robot_location_in_meter);

		std::cout<<"\nRobot location in pixel value: "<<robot_location_in_pixel.x;

		int Pixel_Value = (int)Find_Next_Room_FNR.at< unsigned char >(robot_location_in_pixel);
		/*
		 * comment: If Pixel_value gives the same value then that means
		 * it is traveling on the same room still
		 */
		std::cout<<"\nPixel Value in robot Location point: "<< Pixel_Value << "\n";

		for( unsigned int n=0 ; n<center_of_room.size() ; n++ )
			{
				if ( Find_Next_Room_FNR.at < unsigned char > (center_of_room[n]) != 255 &&
					 Find_Next_Room_FNR.at < unsigned char > (center_of_room[n]) != 0 )
					{
						robot_distance_From_Center = std::min ( robot_distance_From_Center ,
																std::sqrt(((center_of_room_x[n]-robot_location_in_pixel.x)*(center_of_room_x[n]-robot_location_in_pixel.x)) +
																		  ((center_of_room_y[n]-robot_location_in_pixel.y)*(center_of_room_y[n]-robot_location_in_pixel.y)))
															   );
					}
			}

		std::cout<<"Robot Distance from Nearest Room Location: "<< robot_distance_From_Center;

		for( unsigned int n=0 ; n<center_of_room.size() ; n++ )
			{
				if (std::sqrt(((center_of_room_x[n]-robot_location_in_pixel.x)*(center_of_room_x[n]-robot_location_in_pixel.x)) +
							  ((center_of_room_y[n]-robot_location_in_pixel.y)*(center_of_room_y[n]-robot_location_in_pixel.y)))
								== robot_distance_From_Center )
					{
						Center_Position_x = center_of_room_x [n];
						Center_Position_y = center_of_room_y [n];
						room_number.push_back(n);
						std::cout<<"\nNext room to Visit: "<<room_number.back()<<"\n";
					}
			}

		std::cout<<"\n§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§\n";
	}



int Exploration::get_center_position_x()
	{
		return Center_Position_x;
	}



int Exploration::get_center_position_y()
	{
		return Center_Position_y;
	}



std::string Exploration::go_to_destination( cv::Mat FTL , int CenterPositionX , int CenterPositionY )
	{
		cv::Mat go_to_des = FTL.clone();

		std::string Feedback_ = "True";

		MoveBaseClient ac("move_base", true);
		ac.waitForServer();
		ac.sendGoal( stay_forward( CenterPositionX , CenterPositionY ));
		ac.waitForResult();

		ac.waitForServer();
		ac.sendGoal( Move_in_pixel( CenterPositionX , CenterPositionY ));

		bool finished_before_timeout = ac.waitForResult();

		if (finished_before_timeout)
			{
				actionlib::SimpleClientGoalState state = ac.getState();
				ROS_INFO("Move Base Action for go to destination finished: %s",state.toString().c_str());
			}

		 else
			 {
				ROS_INFO("Move Base Action for go to destination did not finish before the time out.");
			 }

		cv::Point robot_location_in_pixel;

		listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);

		Pose robot_location_in_meter(transform.getOrigin().x(),transform.getOrigin().y(),transform.getRotation().z());

		robot_location_in_pixel = convertFromMeterToPixelCoordinates < cv::Point > (robot_location_in_meter);

		if (go_to_des.at<unsigned char> ( CenterPositionY , CenterPositionX ) == go_to_des.at<unsigned char> ( robot_location_in_pixel ))
			{
				Feedback_ = "True";
				//cleaning_pose(FTL,CenterPositionX,CenterPositionY);
				return Feedback_;
			}
		else
			{
				Feedback_ = "False";
				return Feedback_;
			}


	}



cv::Mat Exploration::random_location(cv::Mat RL,
									 std::vector<int> room_Number,
									 std::vector<int> room_min_x,
									 std::vector<int> room_max_x,
									 std::vector<int> room_min_y,
									 std::vector<int> room_max_y,
									 int unsuccessful_times)
	{
		cv::Mat Random_Location = RL.clone();
		cv::Mat Random_Location_mutable = RL ;

		int random_location_x = rand()% room_max_x[room_Number.back()] + room_min_x[room_Number.back()] ;
		int random_location_y = rand()% room_max_y[room_Number.back()] + room_min_y[room_Number.back()] ;

		bool loop = true ;
		while( loop )
			{
				random_location_point.y = random_location_x ;
				random_location_point.x = random_location_y ;

				if (Random_Location.at<unsigned char> ( random_location_point ) != 0 &&
						 Obstacle_free_Point( Random_Location , random_location_x , random_location_y ))
					{
						loop = false;
					}

				else
					{
						random_location_x = rand()% room_max_x[room_Number.back()] + room_min_x[room_Number.back()] ;
						random_location_y = rand()% room_max_y[room_Number.back()] + room_min_y[room_Number.back()] ;
						loop = true;
					}
			}

		cv::Point center_of_room;

		center_of_room.y = (room_max_x[room_Number.back()] + room_min_x[room_Number.back()])/2;
		center_of_room.x = (room_max_y[room_Number.back()] + room_min_y[room_Number.back()])/2;

		std::cout<<"\nCenter of Room x: "<<center_of_room.x<<"\n";
		std::cout<<"\nCenter of Room y: "<<center_of_room.y<<"\n";

		if(unsuccessful_times == 6)
			{
				cv::circle( Random_Location_mutable , center_of_room , 3 , cv::Scalar(255), -1);
				//cv::imshow( "Random Location", Random_Location_mutable );
				//cv::waitKey(100);
			}

		return Random_Location_mutable;
	}



cv::Point Exploration::get_random_location()
	{
		return random_location_point;
	}



std::vector<int>& Exploration::get_room_number()
	{
		return room_number;
	}


std::vector<int>& Exploration::get_room_min_x()
	{
		return Room_min_x;
	}



std::vector<int>& Exploration::get_room_max_x()
	{
		return Room_max_x;
	}



std::vector<int>& Exploration::get_room_min_y()
	{
		return Room_min_y;
	}



std::vector<int>& Exploration::get_room_max_y()
	{
		return Room_max_y;
	}



void Exploration::InflationDataCallback(const nav_msgs::GridCells::ConstPtr& obstacles_data, const nav_msgs::GridCells::ConstPtr& inflated_obstacles_data)
	{
		{
			boost::mutex::scoped_lock lock(mutex_inflation_topic_);

			Inflation_data_X.clear();
			Inflation_data_Y.clear();


			for(unsigned int i=0; i< obstacles_data->cells.size() ; i++)
				{
					Inflation_data_X.push_back(obstacles_data->cells[i].x);
					Inflation_data_Y.push_back(obstacles_data->cells[i].y);
				}


			for(unsigned int i=0; i< inflated_obstacles_data->cells.size() ; i++)
				{
					Inflation_data_X.push_back(inflated_obstacles_data->cells[i].x);
					Inflation_data_Y.push_back(inflated_obstacles_data->cells[i].y);
				}
		}

		condition_inflation_topic_.notify_one();
	}



void Exploration::InflationInit(ros::NodeHandle nh)
	{
		inflation_node_ = nh;
		obstacles_sub_.subscribe(inflation_node_, "/move_base/local_costmap/obstacles", 1);
		inflated_obstacles_sub_.subscribe(inflation_node_, "/move_base/local_costmap/inflated_obstacles", 1);

		inflated_obstacles_sub_sync_ = boost::shared_ptr<message_filters::Synchronizer<InflatedObstaclesSyncPolicy> >(new message_filters::Synchronizer<InflatedObstaclesSyncPolicy>(InflatedObstaclesSyncPolicy(3)));
		inflated_obstacles_sub_sync_->connectInput(obstacles_sub_, inflated_obstacles_sub_);
		inflated_obstacles_sub_sync_->registerCallback(boost::bind(&Exploration::InflationDataCallback, this, _1, _2));
	}



void Exploration::cleaning_pose(cv::Mat CP,
								int center_of_room_x,
								int center_of_room_y)
	{
		cv::Mat Cleaning_Pose = CP.clone();

		cv::Point Clean_Point;

		ros::NodeHandle node2;

		bool found_pose = false;
		MoveBaseClient ac("move_base", true);
		std::cout<<"\nCleaning Pose Function is started4"<<std::endl;

		InflationInit(node2);

		for (int angle = 0; angle < 360 && found_pose==false; angle+=10)
			{
				Clean_Point.x = center_of_room_x + ((( robotRadius * 1.5 ) * cos (( angle * PI / 180.0 ))))/map_resolution_;
				Clean_Point.y = center_of_room_y + ((( robotRadius * 1.5 ) * sin (( angle * PI / 180.0 ))))/map_resolution_;

				double x_m = (Clean_Point.x * map_resolution_)+map_origin_.x;
				double y_m = (Clean_Point.y * map_resolution_)+map_origin_.y;

				if (Cleaning_Pose.at<unsigned char> ( Clean_Point.y ,Clean_Point.x ) != 0 &&
					Obstacle_free_Point( Cleaning_Pose , Clean_Point.y , Clean_Point.x ))
					{
						boost::mutex::scoped_lock lock(mutex_inflation_topic_);
						boost::system_time const timeout=boost::get_system_time()+ boost::posix_time::milliseconds(5000);
						if (condition_inflation_topic_.timed_wait(lock, timeout))
							std::cout << "Got the inflation data." << std::endl;
						else
						{
							std::cout << "Inflation data not available." << std::endl;
						}

						for (unsigned int i = 0 ; i < Inflation_data_X.size() ; i++ )
							{
								if( std::abs( ( x_m - Inflation_data_X[i] ) ) > 0.1 &&
									std::abs( ( y_m - Inflation_data_Y[i] ) ) > 0.1 )
									{
										found_pose = true;
									}
							}
					}
			}

		if (found_pose == true)
			{
				std::cout <<"\nCenter of room x value: "<<center_of_room_x;
				std::cout <<"\nCenter of room y value: "<< center_of_room_y ;
				std::cout <<"\nClean Point of room x value: "<< Clean_Point.x;
				std::cout <<"\nClean Point of room y value: "<< Clean_Point.y<<std::endl;

				ac.waitForServer();
				ac.sendGoal( stay_forward( Clean_Point.x , Clean_Point.y ));
				ac.waitForResult();

				ac.waitForServer();
				ac.sendGoal( Move_in_pixel( Clean_Point.x , Clean_Point.y ));
				ac.waitForResult();

				ac.waitForServer();
				ac.sendGoal( stay_backward( Clean_Point.x , Clean_Point.y , center_of_room_x , center_of_room_y ));
				ac.waitForResult();
			}
		else
			std::cout << "\nNo valid pose found."<< std::endl;
	}



cv::Mat Exploration::Room_Inspection( cv::Mat RI,
									  std::vector<cv::Point> center_of_room,
									  std::vector<int> room_Number,
									  std::vector<int> center_of_room_x ,
									  std::vector<int> center_of_room_y,
									  std::vector<int> room_min_x,
									  std::vector<int> room_max_x,
									  std::vector<int> room_min_y,
									  std::vector<int> room_max_y)
	{
		cv::Mat Room_Inspection_RI = RI;
		cv::Point Pixel_Point_next ;
		int Pixel_Value_of_the_room = Room_Inspection_RI.at<unsigned char> ( center_of_room[room_Number.back()] );
		int loop_count = 0 ;
		int Step_Size = 20 ;
		MoveBaseClient ac("move_base", true);

		ros::NodeHandle node_;

		ros::Rate r(1);
		InflationInit(node_);
		ros::spinOnce();
		r.sleep();

		for ( int m = room_min_x[room_Number.back()] ; m < room_max_x[room_Number.back()] ; m=m+Step_Size )
				{
					if(loop_count % 2 == 0)
						{
							for ( int n = room_min_y[room_Number.back()] ; n < room_max_y[room_Number.back()] ; n=n+Step_Size )
								{
									if ( Room_Inspection_RI.at<unsigned char> ( m , n ) == Pixel_Value_of_the_room &&
										 Obstacle_free_Point( Room_Inspection_RI , m , n )
										)
										{
											cob_map_accessibility_analysis::CheckPointAccessibility::Request req_points;
											cob_map_accessibility_analysis::CheckPointAccessibility::Response res_points;
											std::string points_service_name = "/map_accessibility_analysis/map_points_accessibility_check";

											geometry_msgs::Pose2D point;
											point.x = (n*map_resolution_)+map_origin_.x;
											point.y = (m*map_resolution_)+map_origin_.y;
											req_points.points_to_check.push_back(point);

											// this calls the service server to process our request message and put the result into the response message
											// this call is blocking, i.e. this program will not proceed until the service server sends the response
											bool success = ros::service::call(points_service_name, req_points, res_points);

											if (success == true)
											{
												printf("Points request successful, results:\n");
												for (unsigned int i=0; i<res_points.accessibility_flags.size(); ++i)
													printf(" - (xy)=(%f, %f), accessible=%d\n", req_points.points_to_check[i].x, req_points.points_to_check[i].y, res_points.accessibility_flags[i]);
											}
											else
												std::cout << "The service call for points was not successful.\n" << std::endl;
//											bool obstacle_in_way = false ;
//											for (unsigned int i = 0 ; i < Inflation_data_X.size() ; i++ )
//												{
//													if( std::abs( ( ( (n*map_resolution_)+map_origin_.x) - Inflation_data_X[i] ) ) < 0.1 &&
//														std::abs( ( ( (m*map_resolution_)+map_origin_.y) - Inflation_data_Y[i] ) ) < 0.1 )
//														{
//															obstacle_in_way = true ;
//														}
//												}
//
///*
//											//boost::mutex::scoped_lock lock(mutex_inflation_topic_);
//											//boost::system_time const timeout=boost::get_system_time() + boost::posix_time::milliseconds(5000);
//
//											cv::Mat RM_obstacles_check = RI.clone();
//
//											for( unsigned int idx = 0; idx < Inflation_data_X.size(); idx++ )
//												{
//													RM_obstacles_check.at<unsigned char>( (Inflation_data_Y[idx]- map_origin_.y)/map_resolution_ , (Inflation_data_X[idx]- map_origin_.x)/map_resolution_ ) = 0;
//												}
//
//											cv::imshow("Inflated Map", RM_obstacles_check );
//											cv::waitKey();
//
//											bool obstacle_in_way = false ;
//
//											if( RM_obstacles_check.at<unsigned char>(m,n)== 0 )
//												{
//													obstacle_in_way = true;
//												}
//*/
											if(res_points.accessibility_flags[0])
												{
													Pixel_Point_next.x = n;
													Pixel_Point_next.y = m;

													ac.waitForServer();
													ac.sendGoal( stay_forward( n , m ));
													ac.waitForResult();

													ac.waitForServer();
													ac.sendGoal( Move_in_pixel ( n , m ));

													bool finished_before_timeout = ac.waitForResult();

													if (finished_before_timeout)
														{
															actionlib::SimpleClientGoalState state = ac.getState();
															ROS_INFO("Move Base Action for RI finished: %s",state.toString().c_str());
															//cv::circle( Room_Inspection_RI , Pixel_Point_next , 3 , cv::Scalar(255), -1 );
															//cv::imshow( "Room Inspection", Room_Inspection_RI );
															//cv::waitKey(100);
														}
													else
														{
															ROS_INFO("Move Base Action for RI did not finish before the time out.");
														}
												}
										}
								}

						}

					else
						{
							for ( int n = room_max_y[room_Number.back()] ; n > room_min_y[room_Number.back()] ; n=n-Step_Size )
								{
									if ( Room_Inspection_RI.at<unsigned char> ( m , n ) == Pixel_Value_of_the_room &&
										 Obstacle_free_Point( Room_Inspection_RI , m , n )
										)
										{
//											bool obstacle_in_way = false ;
//
//											for (unsigned int i = 0 ; i < Inflation_data_X.size() ; i++ )
//												{
//													if( std::abs( ( ( (n*map_resolution_)+map_origin_.x) - Inflation_data_X[i] ) ) < 0.1 &&
//														std::abs( ( ( (m*map_resolution_)+map_origin_.y) - Inflation_data_Y[i] ) ) < 0.1 )
//														{
//															obstacle_in_way = true ;
//														}
//												}
///*
//											//boost::mutex::scoped_lock lock(mutex_inflation_topic_);
//											//boost::system_time const timeout=boost::get_system_time() + boost::posix_time::milliseconds(5000);
//
//											cv::Mat RM_obstacles_check = RI.clone();
//
//											for( unsigned int idx = 0; idx < Inflation_data_X.size(); idx++ )
//												{
//													RM_obstacles_check.at<unsigned char>( (Inflation_data_Y[idx]- map_origin_.y)/map_resolution_ , (Inflation_data_X[idx]- map_origin_.x)/map_resolution_ ) = 0;
//												}
//
//											cv::imshow("Inflated Map", RM_obstacles_check );
//											cv::waitKey();
//
//											bool obstacle_in_way = false ;
//
//											if( RM_obstacles_check.at<unsigned char>(m,n)== 0 )
//												{
//													obstacle_in_way = true;
//												}
//*/

											cob_map_accessibility_analysis::CheckPointAccessibility::Request req_points;
											cob_map_accessibility_analysis::CheckPointAccessibility::Response res_points;
											std::string points_service_name = "/map_accessibility_analysis/map_points_accessibility_check";

											geometry_msgs::Pose2D point;
											point.x = (n*map_resolution_)+map_origin_.x;
											point.y = (m*map_resolution_)+map_origin_.y;
											req_points.points_to_check.push_back(point);

											// this calls the service server to process our request message and put the result into the response message
											// this call is blocking, i.e. this program will not proceed until the service server sends the response
											bool success = ros::service::call(points_service_name, req_points, res_points);

											if (success == true)
											{
												printf("Points request successful, results:\n");
												for (unsigned int i=0; i<res_points.accessibility_flags.size(); ++i)
													printf(" - (xy)=(%f, %f), accessible=%d\n", req_points.points_to_check[i].x, req_points.points_to_check[i].y, res_points.accessibility_flags[i]);
											}
											else
												std::cout << "The service call for points was not successful.\n" << std::endl;

											if(res_points.accessibility_flags[0])
												{
													Pixel_Point_next.x = n;
													Pixel_Point_next.y = m;

													ac.waitForServer();
													ac.sendGoal( stay_forward( n , m ));
													ac.waitForResult();

													ac.waitForServer();
													ac.sendGoal( Move_in_pixel ( n , m ));
													bool finished_before_timeout = ac.waitForResult();

													if (finished_before_timeout)
														{
															actionlib::SimpleClientGoalState state = ac.getState();
															ROS_INFO("Move Base Action for RI finished: %s",state.toString().c_str());
															//cv::circle( Room_Inspection_RI , Pixel_Point_next , 3 , cv::Scalar(255), -1 );
															//cv::imshow( "Room Inspection", Room_Inspection_RI );
															//cv::waitKey(100);
														}

													else
														{
															ROS_INFO("Move Base Action for RI did not finish before the time out.");
														}
												}
										}
								}
						}
					loop_count++;
				}

		cv::circle( Room_Inspection_RI , center_of_room[room_Number.back()] , 3 , cv::Scalar(255), -1);
		//cv::imshow("Find Next Room Map", Room_Inspection_RI );
		//cv::waitKey(100);

		ac.waitForServer();
		ac.sendGoal( Move_in_pixel( center_of_room_x[room_Number.back()] , center_of_room_y[room_Number.back()] ));
		bool timeout = ac.waitForResult();

		if (timeout)
			{
				actionlib::SimpleClientGoalState state = ac.getState();
				ROS_INFO("Move Base Action for RI finished: %s",state.toString().c_str());
			}

		 else
			ROS_INFO("Move Base Action for RI did not finish before the time out.");

		if (room_Number.size() == center_of_room.size())
			{
				ac.waitForServer();
				ac.sendGoal( Move( 0 , 0 , 0 ));
				bool timeout = ac.waitForResult();

				if (timeout)
					{
						actionlib::SimpleClientGoalState state = ac.getState();
						ROS_INFO("Move Base Action for initial pose finished: %s",state.toString().c_str());
					}

				 else
					ROS_INFO("Move Base Action for initial pose did not finish before the time out.");
			}

		return Room_Inspection_RI ;
	}



bool Exploration::Obstacle_free_Point( cv::Mat OFP , int x , int y )
	{
		int compare , compare_temp = 1;
		int Clearence_factor = 15 ;
		for ( int p = 0 ; p < Clearence_factor ; p++ )
			{
				for ( int q = 0 ; q < Clearence_factor ; q++ )
					{
						if( OFP.at<unsigned char> ( x + q , y + p ) != 0 &&
							OFP.at<unsigned char> ( x - q , y + p ) != 0 &&
							OFP.at<unsigned char> ( x + q , y - p ) != 0 &&
							OFP.at<unsigned char> ( x - q , y - p ) != 0
						  ){
							compare = 1 ;
						   }

						else compare = 0;
						compare_temp = std::min( compare_temp , compare );
					}
			}

		if ( compare_temp == 1 )
			return true;
		else return false;
	}

