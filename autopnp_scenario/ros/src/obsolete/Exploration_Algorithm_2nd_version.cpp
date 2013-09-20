#include <autopnp_scenario/Exploration_Algorithm.h>


void Exploration::init(ros::NodeHandle nh)
	{
		m_n = nh;
		m_sub = m_n.subscribe <nav_msgs::OccupancyGrid> ("/map", 1000, &Exploration::updateMapCallback, this);
	}



void Exploration::updateMapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg)
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
						if (map_msg->data[i] != 0)
						map_.at<unsigned char>(v,u) = 0;
					}
			}

		//cv::imshow("Original Map", map_);

		Segmented_Map = Image_Segmentation(map_);
		Room_Inspection_Map_Show = Segmented_Map.clone();

		//cv::circle(Segmented_Map, convertFromMeterToPixelCoordinates<cv::Point>(robot_origin_in_meter), 5, cv::Scalar(255), -1);

		//cv::imshow("Segmented Map", Segmented_Map);
		//cv::waitKey(100);

		//******************************Find Next Room**************************************
		Find_Next_Room( Segmented_Map.clone() );
		//******************************Find Next Room**************************************
	}



int main(int argc, char** argv)
	{
		  ros::init(argc, argv, "Send_Goal");
		  ros::NodeHandle n;

		  ROS_INFO("Sending goal");

		  Exploration My;

		  My.init(n);

		  ros::spin();

		  return 0;
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



move_base_msgs::MoveBaseGoal Exploration::Move_in_pixel( int X , int Y )
	{
		move_base_msgs::MoveBaseGoal goal;

		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = ( X * map_resolution_ ) + map_origin_.x ;
		goal.target_pose.pose.position.y = ( Y * map_resolution_ ) + map_origin_.y ;

		goal.target_pose.pose.orientation.w = 1.0;

		return goal;
	}



cv::Mat Exploration::Image_Segmentation( cv::Mat Original_Map )
	{
		std::vector< std::vector <cv::Point> > contours_temp;
		std::vector< std::vector <cv::Point> > saved_contours;

		cv::Mat temp = Original_Map.clone();
		cv::Mat Expanded_Map;
		cv::Mat Contour_map;
		std::vector<cv::Vec4i> hierarchy;

		//When mode is needed
		cv::Mat New_map = Original_Map.clone();


		for ( int i = 0 ; i < 1.5  / map_resolution_ ; i++ )
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
						area = map_resolution_ * map_resolution_ * cv::contourArea(contours_temp[idx]);

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

		//std::cout<<"\nBlack Pixel array size: "<<Black_Pixel.size()<<"\n";

		//std::cout<<"\n-----------------------------------------\n";

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
		//std::vector < cv::Point > Center_of_Room;
		//std::vector < int > Center_of_Room_x;
		//std::vector < int > Center_of_Room_y;
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
				//std::cout<<"\nCenter of the bounding Box["<<a<<"]: [ "<<Center_of_Room_x[a]<<" , "<<Center_of_Room_y[a]<<" ]\n";

				double Center_distance = std::sqrt(((Center_of_Room_x[1]-Center_of_Room_x[a])*(Center_of_Room_x[1]-Center_of_Room_x[a])) +
												   ((Center_of_Room_y[1]-Center_of_Room_y[a])*(Center_of_Room_y[1]-Center_of_Room_y[a])));

				distance_bt_Centers.push_back(Center_distance);
				//std::cout<<"\nCentral Distance from First Room [4] to room ["<<a<<"]: "<<distance_bt_Centers[a]<<"\n";
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
		Center_of_Room_x.clear();
		Center_of_Room_y.clear();

		//************Memory_Clear(Vector-Memory)**********************************

		return New_Filled_Map.clone();
	}



void Exploration::Find_Next_Room( cv::Mat FNR )
	{
		cv::Mat Find_Next_Room_FNR = FNR.clone();
		cv::Mat FNR_Path_direction = FNR.clone();
		double robot_distance_From_Center = 10000;
		int loop_count = Center_of_Room.size();
		int Center_Position_x = 0 , Center_Position_y = 0 , room_number = 0 ;
		cv::Point robot_location_in_pixel;
		MoveBaseClient ac("move_base", true);

		std::cout<<"\n§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§\n";
		do
			{
				//Get the current position of the robot
				listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);

				Pose robot_location_in_meter(transform.getOrigin().x(),transform.getOrigin().y(),transform.getRotation().z());

				robot_location_in_pixel = convertFromMeterToPixelCoordinates < cv::Point > (robot_location_in_meter);

				int Pixel_Value = (int)Find_Next_Room_FNR.at< unsigned char >(robot_location_in_pixel);
				/*
				 * comment: If Pixel_value gives the same value then that means
				 * it is traveling on the same room still
				 */
				std::cout<<"\nRobot Location in pixel: "<<robot_location_in_pixel;
				std::cout<<"\nPixel Value in robot Location point: "<< Pixel_Value << "\n";

				for( unsigned int n=0 ; n<Center_of_Room.size() ; n++ )
					{
						if ( Find_Next_Room_FNR.at < unsigned char > (Center_of_Room[n]) != 255 &&
							 Find_Next_Room_FNR.at < unsigned char > (Center_of_Room[n]) != 0 )
							{
								robot_distance_From_Center = std::min ( robot_distance_From_Center ,
																		std::sqrt(((Center_of_Room_x[n]-robot_location_in_pixel.x)*(Center_of_Room_x[n]-robot_location_in_pixel.x)) +
																				  ((Center_of_Room_y[n]-robot_location_in_pixel.y)*(Center_of_Room_y[n]-robot_location_in_pixel.y)))
																	   );
							}
					}

				std::cout<<"Robot Distance from Nearest Room Location: "<< robot_distance_From_Center;

				for( unsigned int n=0 ; n<Center_of_Room.size() ; n++ )
					{
						if (std::sqrt(((Center_of_Room_x[n]-robot_location_in_pixel.x)*(Center_of_Room_x[n]-robot_location_in_pixel.x)) +
									  ((Center_of_Room_y[n]-robot_location_in_pixel.y)*(Center_of_Room_y[n]-robot_location_in_pixel.y)))
										== robot_distance_From_Center )
							{
								Center_Position_x = Center_of_Room_x [n];
								Center_Position_y = Center_of_Room_y [n];
								room_number = n ;
								std::cout<<"\nNext room to Visit: "<<room_number<<"\n";
							}
					}

				//MoveBaseClient ac("move_base", true);
				while(!ac.waitForServer(ros::Duration(5.0)))
					{
						ROS_INFO("\nWaiting for the move_base action server to come up");
					}
				ac.sendGoal( Move_in_pixel( Center_Position_x , Center_Position_y ));
				ac.waitForResult();

				cv::circle( FNR_Path_direction, Center_of_Room[room_number] , 5 , cv::Scalar(255), -1);
				cv::line( FNR_Path_direction , robot_location_in_pixel , Center_of_Room[room_number] , cv::Scalar(255), 2);

				cv::imshow("Find Next Room Map", FNR_Path_direction);
				cv::waitKey(100);

				//@@@@@@@@@@@@@@@@@@@@@ Room inspection @@@@@@@@@@@@@@@@@@@@@@@@@@

				Room_Inspection( FNR.clone() , room_number );
				//Room_Inspection();

				//@@@@@@@@@@@@@@@@@@@@@ Room inspection @@@@@@@@@@@@@@@@@@@@@@@@@@

				//cv::circle( Find_Next_Room_FNR , Center_of_Room[room_number] , 3 , cv::Scalar(255), -1);
				Find_Next_Room_FNR.at<unsigned char> ( Center_Position_y , Center_Position_x ) = 255 ;

				//cv::circle( FNR_Path_direction, Center_of_Room[room_number] , 5 , cv::Scalar(255), -1);
				//cv::line( FNR_Path_direction , robot_location_in_pixel , Center_of_Room[room_number] , cv::Scalar(255), 2);

				robot_distance_From_Center = 10000;
				loop_count--;

				//cv::imshow("Find Next Room Map", FNR_Path_direction);
				//cv::waitKey(100);

				std::cout<<"\n§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§\n";
			}
		while( loop_count != 0 );

		ac.waitForServer(ros::Duration(5.0));
		ac.sendGoal( Move( 0 , 0 , 0 ));
		ac.waitForResult();
	}



void Exploration::Room_Inspection( cv::Mat RI , int room_Number )
	{
		cv::Mat Room_Inspection_RI = RI.clone();
		cv::Point Pixel_Point_next ; //Pixel_Point_Previous ;
		int Pixel_Value_of_the_room = Room_Inspection_RI.at<unsigned char> ( Center_of_Room[room_Number] ) ;
		int loop_count = 0 ;
		int Step_Size = 20 ;
		MoveBaseClient ac("move_base", true);

		for ( int m = Room_min_x[room_Number] ; m < Room_max_x[room_Number] ; m=m+Step_Size )
				{
					if(loop_count % 2 == 0)
						{
							for ( int n = Room_min_y[room_Number] ; n < Room_max_y[room_Number] ; n=n+Step_Size )
								{
									if ( Room_Inspection_RI.at<unsigned char> ( m , n ) == Pixel_Value_of_the_room &&
										 Obstacle_free_Point( Room_Inspection_RI , m , n )
										)
										{
											Pixel_Point_next.x = n;
											Pixel_Point_next.y = m;

											/*
											Pixel_Point_Previous.x = n + 20 ;
											Pixel_Point_Previous.y = m ;
											if( n > (Room_max_y[room_Number] - 19) )
												{
													Pixel_Point_Previous.x = Room_min_y[room_Number];
													Pixel_Point_Previous.y = m + 20;
												}
											cv::line( Room_Inspection_Map_Show , Pixel_Point_next , Pixel_Point_Previous , cv::Scalar(255) , 2 );
											*/

											ac.waitForServer(ros::Duration(5.0));
											ac.sendGoal( Move_in_pixel ( n , m ));
											ac.waitForResult();
											cv::circle( Room_Inspection_Map_Show , Pixel_Point_next , 3 , cv::Scalar(255), -1 );
											cv::imshow( "Room Inspection", Room_Inspection_Map_Show );
											cv::waitKey(100);
										}
								}

						}

					else
						{
							for ( int n = Room_max_y[room_Number] ; n > Room_min_y[room_Number] ; n=n-Step_Size )
								{
									if ( Room_Inspection_RI.at<unsigned char> ( m , n ) == Pixel_Value_of_the_room &&
										 Obstacle_free_Point( Room_Inspection_RI , m , n )
										)
										{
											Pixel_Point_next.x = n;
											Pixel_Point_next.y = m;

											/*
											Pixel_Point_Previous.x = n + 20 ;
											Pixel_Point_Previous.y = m ;
											if( n > (Room_max_y[room_Number] - 19) )
												{
													Pixel_Point_Previous.x = Room_min_y[room_Number];
													Pixel_Point_Previous.y = m + 20;
												}
											cv::line( Room_Inspection_Map_Show , Pixel_Point_next , Pixel_Point_Previous , cv::Scalar(255) , 2 );
											*/

											ac.waitForServer(ros::Duration(5.0));
											ac.sendGoal( Move_in_pixel ( n , m ));
											ac.waitForResult();
											cv::circle( Room_Inspection_Map_Show , Pixel_Point_next , 3 , cv::Scalar(255), -1 );
											cv::imshow( "Room Inspection", Room_Inspection_Map_Show );
											cv::waitKey(100);
										}
								}
						}
					loop_count++;
				}
		ac.waitForServer(ros::Duration(5.0));
		ac.sendGoal( Move_in_pixel( Center_of_Room_x[room_Number] , Center_of_Room_y[room_Number] ));
		ac.waitForResult();
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
							OFP.at<unsigned char> ( x - q , y - p ) != 0 //&&
							//OFP.at<unsigned char> ( x , y + p ) != 0 &&
							//OFP.at<unsigned char> ( x + p , y ) != 0 &&
							//OFP.at<unsigned char> ( x - p , y ) != 0 &&
							//OFP.at<unsigned char> ( x , y - p ) != 0
						  ){
							compare = 1 ;
						   }

						else compare = 0;
						compare_temp = std::min( compare_temp , compare );
					}
			}
		//std::cout<<"\ncompare_temp : "<<compare_temp<<"\n";

		if ( compare_temp == 1 )
			return true;
		else return false;
	}



/*


bool Exploration::Obstacle_free_Point( cv::Mat OFP , int x , int y )
	{
		if( OFP.at<unsigned char> ( x + 0 , y + 0 ) != 0 &&
			OFP.at<unsigned char> ( x - 0 , y + 0 ) != 0 &&
			OFP.at<unsigned char> ( x + 0 , y - 0 ) != 0 &&
			OFP.at<unsigned char> ( x - 0 , y - 0 ) != 0 &&
			OFP.at<unsigned char> ( x , y + 0 ) != 0 &&
			OFP.at<unsigned char> ( x + 0 , y ) != 0 &&
			OFP.at<unsigned char> ( x - 0 , y ) != 0 &&
			OFP.at<unsigned char> ( x , y - 0 ) != 0 &&
			OFP.at<unsigned char> ( x + 1 , y + 1 ) != 0 &&
			OFP.at<unsigned char> ( x - 1 , y + 1 ) != 0 &&
			OFP.at<unsigned char> ( x + 1 , y - 1 ) != 0 &&
			OFP.at<unsigned char> ( x - 1 , y - 1 ) != 0 &&
			OFP.at<unsigned char> ( x , y + 1 ) != 0 &&
			OFP.at<unsigned char> ( x + 1 , y ) != 0 &&
			OFP.at<unsigned char> ( x - 1 , y ) != 0 &&
			OFP.at<unsigned char> ( x , y - 1 ) != 0 &&
			OFP.at<unsigned char> ( x + 2 , y + 2 ) != 0 &&
			OFP.at<unsigned char> ( x - 2 , y + 2 ) != 0 &&
			OFP.at<unsigned char> ( x + 2 , y - 2 ) != 0 &&
			OFP.at<unsigned char> ( x - 2 , y - 2 ) != 0 &&
			OFP.at<unsigned char> ( x , y + 2 ) != 0 &&
			OFP.at<unsigned char> ( x + 2 , y ) != 0 &&
			OFP.at<unsigned char> ( x - 2 , y ) != 0 &&
			OFP.at<unsigned char> ( x , y - 2 ) != 0 &&
			OFP.at<unsigned char> ( x + 3 , y + 3 ) != 0 &&
			OFP.at<unsigned char> ( x - 3 , y + 3 ) != 0 &&
			OFP.at<unsigned char> ( x + 3 , y - 3 ) != 0 &&
			OFP.at<unsigned char> ( x - 3 , y - 3 ) != 0 &&
			OFP.at<unsigned char> ( x , y + 3 ) != 0 &&
			OFP.at<unsigned char> ( x + 3 , y ) != 0 &&
			OFP.at<unsigned char> ( x - 3 , y ) != 0 &&
			OFP.at<unsigned char> ( x , y - 3 ) != 0 &&
			OFP.at<unsigned char> ( x + 4 , y + 4 ) != 0 &&
			OFP.at<unsigned char> ( x - 4 , y + 4 ) != 0 &&
			OFP.at<unsigned char> ( x + 4 , y - 4 ) != 0 &&
			OFP.at<unsigned char> ( x - 4 , y - 4 ) != 0 &&
			OFP.at<unsigned char> ( x , y + 4 ) != 0 &&
			OFP.at<unsigned char> ( x + 4 , y ) != 0 &&
			OFP.at<unsigned char> ( x - 4 , y ) != 0 &&
			OFP.at<unsigned char> ( x , y - 4 ) != 0 &&
			OFP.at<unsigned char> ( x + 5 , y + 5 ) != 0 &&
			OFP.at<unsigned char> ( x - 5 , y + 5 ) != 0 &&
			OFP.at<unsigned char> ( x + 5 , y - 5 ) != 0 &&
			OFP.at<unsigned char> ( x - 5 , y - 5 ) != 0 &&
			OFP.at<unsigned char> ( x , y + 5 ) != 0 &&
			OFP.at<unsigned char> ( x + 5 , y ) != 0 &&
			OFP.at<unsigned char> ( x - 5 , y ) != 0 &&
			OFP.at<unsigned char> ( x , y - 5 ) != 0 &&
			OFP.at<unsigned char> ( x + 6 , y + 6 ) != 0 &&
			OFP.at<unsigned char> ( x - 6 , y + 6 ) != 0 &&
			OFP.at<unsigned char> ( x + 6 , y - 6 ) != 0 &&
			OFP.at<unsigned char> ( x - 6 , y - 6 ) != 0 &&
			OFP.at<unsigned char> ( x , y + 6 ) != 0 &&
			OFP.at<unsigned char> ( x + 6 , y ) != 0 &&
			OFP.at<unsigned char> ( x - 6 , y ) != 0 &&
			OFP.at<unsigned char> ( x , y - 6 ) != 0 &&
			OFP.at<unsigned char> ( x + 7 , y + 7 ) != 0 &&
			OFP.at<unsigned char> ( x - 7 , y + 7 ) != 0 &&
			OFP.at<unsigned char> ( x + 7 , y - 7 ) != 0 &&
			OFP.at<unsigned char> ( x - 7 , y - 7 ) != 0 &&
			OFP.at<unsigned char> ( x , y + 7 ) != 0 &&
			OFP.at<unsigned char> ( x + 7 , y ) != 0 &&
			OFP.at<unsigned char> ( x - 7 , y ) != 0 &&
			OFP.at<unsigned char> ( x , y - 7 ) != 0
		  )
		return true;

		else return false;
	}



void Exploration::Room_Inspection( cv::Mat RI , int room_Number)
	{
		cv::Mat Room_Inspection_RI = RI.clone();
		int Center_Position_x = Center_of_Room_x[room_Number] , Center_Position_y = Center_of_Room_y[room_Number];
		int Pixel_Value_of_the_room = Room_Inspection_RI.at<unsigned char> ( Center_of_Room[room_Number] ) ;
		int constant = ( 1 / map_resolution_);
		MoveBaseClient ac("move_base", true);
		cv::Point Update_Position;
		//cv::Point Line_begin_Point;
		//std::cout<<"\nCenter_Position_x: "<<Center_Position_x<<"\n";

		if ( Room_Inspection_RI.at<unsigned char> ( Center_of_Room[room_Number] ) != 255 )
			{
				//Room_Inspection_RI.at<unsigned char> ( Center_of_Room[room_Number] ) = 255 ;
				cv::circle( Room_Inspection_RI , Center_of_Room[room_Number] , 3 , cv::Scalar(255), -1 );
				cv::circle( Room_Inspection_Map_Show , Center_of_Room[room_Number] , 5 , cv::Scalar(255), -1 );
				//cv::imshow( "Room Inspection", Room_Inspection_Map_Show );
				//cv::waitKey(100);
			}

		//std::cout<<"\nCenter Pixel Value: "<<(int)Room_Inspection_RI.at<unsigned char> ( Center_Position_y , Center_Position_x )<<"\n";

		if ( Room_Inspection_RI.at<unsigned char> ( Center_of_Room[room_Number] ) == 255 )
			{
				while ( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x - constant) ) 			 == Pixel_Value_of_the_room ||
						Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + constant) , (Center_Position_x - constant) ) == Pixel_Value_of_the_room ||
						Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + constant) , (Center_Position_x) ) 			 == Pixel_Value_of_the_room ||
						Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + constant) , (Center_Position_x + constant) ) == Pixel_Value_of_the_room ||
						Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x + constant) ) 			 == Pixel_Value_of_the_room ||
						Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - constant) , (Center_Position_x + constant) ) == Pixel_Value_of_the_room ||
						Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - constant) , (Center_Position_x) ) 			 == Pixel_Value_of_the_room ||
						Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - constant) , (Center_Position_x - constant) ) == Pixel_Value_of_the_room
					  ){
							if( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x + constant) ) == Pixel_Value_of_the_room &&
							    Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x + 2 * constant) ) != 0
							    //Room_Inspection_Robot_Ground_Clearence( Room_Inspection_RI , (Center_Position_x + constant) , (Center_Position_y) , Pixel_Value_of_the_room )
							  )
								{
									Update_Position = use_costmap( costmap_1 , Room_Inspection_RI , Center_Position_x , Center_Position_y );
									Center_Position_y = Update_Position.y ;
									Center_Position_x = Update_Position.x ;
								}

							else if( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + constant) , (Center_Position_x) ) == Pixel_Value_of_the_room &&
									 Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + 2 * constant) , (Center_Position_x) ) != 0
									 //Room_Inspection_Robot_Ground_Clearence( Room_Inspection_RI , (Center_Position_x) , (Center_Position_y + 2 * constant) , Pixel_Value_of_the_room )
								   )
									{
										Update_Position = use_costmap( costmap_2 , Room_Inspection_RI , Center_Position_x , Center_Position_y );
										Center_Position_y = Update_Position.y ;
										Center_Position_x = Update_Position.x ;
									}

							else if( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x - constant) ) == Pixel_Value_of_the_room &&
									 Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x - 2 * constant) ) != 0
									 //Room_Inspection_Robot_Ground_Clearence( Room_Inspection_RI , (Center_Position_x) , (Center_Position_y + 2 * constant) , Pixel_Value_of_the_room )
								   )
									{
										Update_Position = use_costmap( costmap_3 , Room_Inspection_RI , Center_Position_x , Center_Position_y );
										Center_Position_y = Update_Position.y ;
										Center_Position_x = Update_Position.x ;
									}

							else if( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - constant) , (Center_Position_x) ) == Pixel_Value_of_the_room &&
									 Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - 2 * constant) , (Center_Position_x) ) != 0
									 //Room_Inspection_Robot_Ground_Clearence( Room_Inspection_RI , (Center_Position_x) , (Center_Position_y + 2 * constant) , Pixel_Value_of_the_room )
								   )
									{
										Update_Position = use_costmap( costmap_4 , Room_Inspection_RI , Center_Position_x , Center_Position_y  );
										Center_Position_y = Update_Position.y ;
										Center_Position_x = Update_Position.x ;
									}

							else if( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + constant) , (Center_Position_x + constant) ) == Pixel_Value_of_the_room &&
									 Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + 2 * constant) , (Center_Position_x + 2 * constant) ) != 0
									 //Room_Inspection_Robot_Ground_Clearence( Room_Inspection_RI , (Center_Position_x) , (Center_Position_y + 2 * constant) , Pixel_Value_of_the_room )
								   )
									{
										Update_Position = use_costmap( costmap_5 , Room_Inspection_RI , Center_Position_x , Center_Position_y );
										Center_Position_y = Update_Position.y ;
										Center_Position_x = Update_Position.x ;
									}

							else if( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + constant) , (Center_Position_x - constant) ) == Pixel_Value_of_the_room &&
									 Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + 2 * constant) , (Center_Position_x - 2 * constant) ) != 0
									 //Room_Inspection_Robot_Ground_Clearence( Room_Inspection_RI , (Center_Position_x) , (Center_Position_y + 2 * constant) , Pixel_Value_of_the_room )
								   )
									{
										Update_Position = use_costmap( costmap_6 , Room_Inspection_RI , Center_Position_x , Center_Position_y );
										Center_Position_y = Update_Position.y ;
										Center_Position_x = Update_Position.x ;
									}

							else if( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - constant) , (Center_Position_x - constant) ) == Pixel_Value_of_the_room &&
									 Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - 2 * constant) , (Center_Position_x - 2* constant) ) != 0
									 //Room_Inspection_Robot_Ground_Clearence( Room_Inspection_RI , (Center_Position_x) , (Center_Position_y + 2 * constant) , Pixel_Value_of_the_room )
								   )
									{
										Update_Position = use_costmap( costmap_7 , Room_Inspection_RI , Center_Position_x , Center_Position_y );
										Center_Position_y = Update_Position.y ;
										Center_Position_x = Update_Position.x ;
									}

							else if( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - constant) , (Center_Position_x + constant) ) == Pixel_Value_of_the_room &&
									 Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - 2 * constant) , (Center_Position_x + 2 * constant) ) != 0
									 //Room_Inspection_Robot_Ground_Clearence( Room_Inspection_RI , (Center_Position_x) , (Center_Position_y + 2 * constant) , Pixel_Value_of_the_room )
								   )
									{
										Update_Position = use_costmap( costmap_8 , Room_Inspection_RI , Center_Position_x , Center_Position_y );
										Center_Position_y = Update_Position.y ;
										Center_Position_x = Update_Position.x ;
									}
							cv::imshow( "Room Inspection", Room_Inspection_Map_Show );
							cv::waitKey(100);
					   }
				ac.waitForServer(ros::Duration(5.0));
				ac.sendGoal( Move_in_pixel( Center_Position_x , Center_Position_y ));
				ac.waitForResult();
			}
	}



bool Exploration::Room_Inspection_Robot_Ground_Clearence( cv::Mat RIRGC , int x , int y , int PixelValue )
	{
		float Clearence_Factor = 5 / robotRadius ;
		if ( RIRGC.at<unsigned char> ( ( y ) , ( x + Clearence_Factor ) ) == PixelValue &&
			 RIRGC.at<unsigned char> ( ( y + Clearence_Factor ) , ( x ) ) == PixelValue &&
			 RIRGC.at<unsigned char> ( ( y ) , ( x - Clearence_Factor ) ) == PixelValue &&
			 RIRGC.at<unsigned char> ( ( y - Clearence_Factor ) , ( x ) ) == PixelValue &&
			 RIRGC.at<unsigned char> ( ( y + Clearence_Factor ) , ( x + Clearence_Factor ) ) == PixelValue &&
			 RIRGC.at<unsigned char> ( ( y + Clearence_Factor ) , ( x - Clearence_Factor ) ) == PixelValue &&
			 RIRGC.at<unsigned char> ( ( y - Clearence_Factor ) , ( x - Clearence_Factor ) ) == PixelValue &&
			 RIRGC.at<unsigned char> ( ( y - Clearence_Factor ) , ( x + Clearence_Factor ) ) == PixelValue
		   )
			{
				return true;
			}
		else return false;
	}



cv::Point Exploration::use_costmap( costmap cost , cv::Mat RI_Cost , int x ,int y )
	{
		cv::Point Line_begin_Point;
		cv::Point Pose_update;

		int constant = ( 1 / map_resolution_);

		MoveBaseClient ac("move_base", true);

		switch ( cost )
			{
				case costmap_1 :
					{
						ac.waitForServer(ros::Duration(5.0));
						ac.sendGoal( Move_in_pixel ( x + constant , y ));
						ac.waitForResult();
						Line_begin_Point.x = x ;
						Line_begin_Point.y = y ;
						Pose_update.x = x + constant;
						Pose_update.y = y ;
						//Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x - constant) ) = 255 ;
						cv::circle( RI_Cost , Pose_update , 3 , cv::Scalar(255), -1 );
						cv::circle( Room_Inspection_Map_Show , Pose_update , 5 , cv::Scalar(255), -1 );
						cv::line( Room_Inspection_Map_Show , Pose_update , Line_begin_Point , cv::Scalar(255) , 2 );
					}
				break;

				case costmap_2 :
					{
						ac.waitForServer(ros::Duration(5.0));
						ac.sendGoal( Move_in_pixel ( x , y + constant ));
						ac.waitForResult();
						Line_begin_Point.x = x ;
						Line_begin_Point.y = y ;
						Pose_update.x = x ;
						Pose_update.y = y + constant ;
						//Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x - constant) ) = 255 ;
						cv::circle( RI_Cost , Pose_update , 3 , cv::Scalar(255), -1 );
						cv::circle( Room_Inspection_Map_Show , Pose_update , 5 , cv::Scalar(255), -1 );
						cv::line( Room_Inspection_Map_Show , Pose_update , Line_begin_Point , cv::Scalar(255) , 2 );
					}
				break;

				case costmap_3 :
					{
						ac.waitForServer(ros::Duration(5.0));
						ac.sendGoal( Move_in_pixel ( x - constant , y ));
						ac.waitForResult();
						Line_begin_Point.x = x ;
						Line_begin_Point.y = y ;
						Pose_update.x = x - constant;
						Pose_update.y = y ;
						//Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x - constant) ) = 255 ;
						cv::circle( RI_Cost , Pose_update , 3 , cv::Scalar(255), -1 );
						cv::circle( Room_Inspection_Map_Show , Pose_update , 5 , cv::Scalar(255), -1 );
						cv::line( Room_Inspection_Map_Show , Pose_update , Line_begin_Point , cv::Scalar(255) , 2 );
					}
				break;

				case costmap_4 :
					{
						ac.waitForServer(ros::Duration(5.0));
						ac.sendGoal( Move_in_pixel ( x , y - constant ));
						ac.waitForResult();
						Line_begin_Point.x = x ;
						Line_begin_Point.y = y ;
						Pose_update.x = x ;
						Pose_update.y = y - constant ;
						//Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x - constant) ) = 255 ;
						cv::circle( RI_Cost , Pose_update , 3 , cv::Scalar(255), -1 );
						cv::circle( Room_Inspection_Map_Show , Pose_update , 5 , cv::Scalar(255), -1 );
						cv::line( Room_Inspection_Map_Show , Pose_update , Line_begin_Point , cv::Scalar(255) , 2 );
					}
				break;

				case costmap_5 :
					{
						ac.waitForServer(ros::Duration(5.0));
						ac.sendGoal( Move_in_pixel ( x + constant , y + constant ));
						ac.waitForResult();
						Line_begin_Point.x = x ;
						Line_begin_Point.y = y ;
						Pose_update.x = x + constant ;
						Pose_update.y = y + constant ;
						//Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x - constant) ) = 255 ;
						cv::circle( RI_Cost , Pose_update , 3 , cv::Scalar(255), -1 );
						cv::circle( Room_Inspection_Map_Show , Pose_update , 5 , cv::Scalar(255), -1 );
						cv::line( Room_Inspection_Map_Show , Pose_update , Line_begin_Point , cv::Scalar(255) , 2 );
					}
				break;

				case costmap_6 :
					{
						ac.waitForServer(ros::Duration(5.0));
						ac.sendGoal( Move_in_pixel ( x - constant , y + constant ));
						ac.waitForResult();
						Line_begin_Point.x = x ;
						Line_begin_Point.y = y ;
						Pose_update.x = x - constant ;
						Pose_update.y = y + constant ;
						//Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x - constant) ) = 255 ;
						cv::circle( RI_Cost , Pose_update , 3 , cv::Scalar(255), -1 );
						cv::circle( Room_Inspection_Map_Show , Pose_update , 5 , cv::Scalar(255), -1 );
						cv::line( Room_Inspection_Map_Show , Pose_update , Line_begin_Point , cv::Scalar(255) , 2 );
					}
				break;

				case costmap_7 :
					{
						ac.waitForServer(ros::Duration(5.0));
						ac.sendGoal( Move_in_pixel ( x - constant , y - constant ));
						ac.waitForResult();
						Line_begin_Point.x = x ;
						Line_begin_Point.y = y ;
						Pose_update.x = x - constant ;
						Pose_update.y = y - constant ;
						//Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x - constant) ) = 255 ;
						cv::circle( RI_Cost , Pose_update , 3 , cv::Scalar(255), -1 );
						cv::circle( Room_Inspection_Map_Show , Pose_update , 5 , cv::Scalar(255), -1 );
						cv::line( Room_Inspection_Map_Show , Pose_update , Line_begin_Point , cv::Scalar(255) , 2 );
					}
				break;

				case costmap_8 :
					{
						ac.waitForServer(ros::Duration(5.0));
						ac.sendGoal( Move_in_pixel ( x + constant , y - constant ));
						ac.waitForResult();
						Line_begin_Point.x = x ;
						Line_begin_Point.y = y ;
						Pose_update.x = x + constant ;
						Pose_update.y = y - constant ;
						//Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x - constant) ) = 255 ;
						cv::circle( RI_Cost , Pose_update , 3 , cv::Scalar(255), -1 );
						cv::circle( Room_Inspection_Map_Show , Pose_update , 5 , cv::Scalar(255), -1 );
						cv::line( Room_Inspection_Map_Show , Pose_update , Line_begin_Point , cv::Scalar(255) , 2 );
					}
				break;

				default:
					{}
			}
		return Pose_update;
	}



 void Exploration::Room_Inspection( cv::Mat RI , int room_Number)
	{
		cv::Mat Room_Inspection_RI = RI.clone();
		int Center_Position_x = Center_of_Room_x[room_Number] , Center_Position_y = Center_of_Room_y[room_Number];
		int Pixel_Value_of_the_room = Room_Inspection_RI.at<unsigned char> ( Center_of_Room[room_Number] ) ;
		int constant = ( 1 / map_resolution_);
		MoveBaseClient ac("move_base", true);
		cv::Point draw_circle_in_Path;
		cv::Point Line_begin_Point;
		std::cout<<"\nCenter_Position_x: "<<Center_Position_x<<"\n";

		if ( Room_Inspection_RI.at<unsigned char> ( Center_of_Room[room_Number] ) != 255 )
			{
				Room_Inspection_RI.at<unsigned char> ( Center_of_Room[room_Number] ) = 255 ;
				cv::circle( Room_Inspection_Map_Show , Center_of_Room[room_Number] , 5 , cv::Scalar(255), -1 );
				//cv::imshow( "Room Inspection", Room_Inspection_Map_Show );
				//cv::waitKey(100);
			}

		//std::cout<<"\nCenter Pixel Value: "<<(int)Room_Inspection_RI.at<unsigned char> ( Center_Position_y , Center_Position_x )<<"\n";

		if ( Room_Inspection_RI.at<unsigned char> ( Center_of_Room[room_Number] ) == 255 )
			{
				while ( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x - constant) ) 			 == Pixel_Value_of_the_room ||
						//Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + constant) , (Center_Position_x - constant) ) == Pixel_Value_of_the_room ||
						Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + constant) , (Center_Position_x) ) 			 == Pixel_Value_of_the_room ||
						//Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + constant) , (Center_Position_x + constant) ) == Pixel_Value_of_the_room ||
						Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x + constant) ) 			 == Pixel_Value_of_the_room ||
						//Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - constant) , (Center_Position_x + constant) ) == Pixel_Value_of_the_room ||
						Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - constant) , (Center_Position_x) ) 			 == Pixel_Value_of_the_room
						//Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - constant) , (Center_Position_x - constant) ) == Pixel_Value_of_the_room
					  ){
							while( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x - constant) ) == Pixel_Value_of_the_room &&
								Room_Inspection_Robot_Ground_Clearence( Room_Inspection_RI , (Center_Position_x - constant) , (Center_Position_y) , Pixel_Value_of_the_room )
							  )
								{
									ac.waitForServer(ros::Duration(5.0));
									ac.sendGoal( Move_in_pixel ( Center_Position_x - constant , Center_Position_y ));
									if(ac.waitForResult())
										{
											Line_begin_Point.x = Center_Position_x ;
											Line_begin_Point.y = Center_Position_y ;
											Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x - constant) ) = 255 ;
											Center_Position_y = Center_Position_y ;
											Center_Position_x = Center_Position_x - constant;
											draw_circle_in_Path.x = Center_Position_x - constant;
											draw_circle_in_Path.y = Center_Position_y ;
											cv::circle( Room_Inspection_Map_Show , draw_circle_in_Path , 5 , cv::Scalar(255), -1 );
											cv::line( Room_Inspection_Map_Show , Line_begin_Point , draw_circle_in_Path , cv::Scalar(255) , 2 );
										}
								}

							else if( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + constant) , (Center_Position_x - constant) ) == Pixel_Value_of_the_room &&
									 Room_Inspection_Robot_Ground_Clearence( Room_Inspection_RI , (Center_Position_x - constant) , (Center_Position_y + constant) , Pixel_Value_of_the_room )
							       )
								{
									ac.waitForServer(ros::Duration(5.0));
									ac.sendGoal( Move_in_pixel ( Center_Position_x - constant , Center_Position_y + constant ));
									ac.waitForResult();
									Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + constant) , (Center_Position_x - constant) ) = 255 ;
									Center_Position_y = Center_Position_y + constant;
									Center_Position_x = Center_Position_x - constant;
								}

							while( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + constant) , (Center_Position_x) ) == Pixel_Value_of_the_room &&
									 Room_Inspection_Robot_Ground_Clearence( Room_Inspection_RI , (Center_Position_x) , (Center_Position_y + constant) , Pixel_Value_of_the_room )
								   )
								{
									ac.waitForServer(ros::Duration(5.0));
									ac.sendGoal( Move_in_pixel ( Center_Position_x , Center_Position_y + constant ));
									if(ac.waitForResult())
										{
											Line_begin_Point.x = Center_Position_x ;
											Line_begin_Point.y = Center_Position_y ;
											Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + constant) , (Center_Position_x) ) = 255 ;
											Center_Position_y =  Center_Position_y + constant;
											Center_Position_x = Center_Position_x;
											draw_circle_in_Path.x = Center_Position_x;
											draw_circle_in_Path.y =  Center_Position_y + constant;
											cv::circle( Room_Inspection_Map_Show , draw_circle_in_Path , 5 , cv::Scalar(255), -1 );
											cv::line( Room_Inspection_Map_Show , Line_begin_Point , draw_circle_in_Path , cv::Scalar(255) , 2 );
										}
								}

							else if( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + constant) , (Center_Position_x + constant) ) == Pixel_Value_of_the_room &&
									 Room_Inspection_Robot_Ground_Clearence( Room_Inspection_RI , (Center_Position_x + constant) , (Center_Position_y + constant) , Pixel_Value_of_the_room )
								   )
								{
									ac.waitForServer(ros::Duration(5.0));
									ac.sendGoal( Move_in_pixel ( Center_Position_x + constant , Center_Position_y + constant ));
									ac.waitForResult();
									Room_Inspection_RI.at<unsigned char> ( (Center_Position_y + constant) , (Center_Position_x + constant) ) = 255 ;
									Center_Position_y = Center_Position_y + constant;
									Center_Position_x = Center_Position_x + constant;
								}

							while( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x + constant) ) == Pixel_Value_of_the_room &&
									 Room_Inspection_Robot_Ground_Clearence( Room_Inspection_RI , (Center_Position_x + constant) , (Center_Position_y) , Pixel_Value_of_the_room )
								   )
								{
									ac.waitForServer(ros::Duration(5.0));
									ac.sendGoal( Move_in_pixel ( Center_Position_x + constant , Center_Position_y));
									if(ac.waitForResult())
										{
											Line_begin_Point.x = Center_Position_x ;
											Line_begin_Point.y = Center_Position_y ;
											Room_Inspection_RI.at<unsigned char> ( (Center_Position_y) , (Center_Position_x + constant) ) = 255 ;
											Center_Position_y =  Center_Position_y;
											Center_Position_x = Center_Position_x + constant;
											draw_circle_in_Path.x = Center_Position_x + constant;
											draw_circle_in_Path.y =  Center_Position_y;
											cv::circle( Room_Inspection_Map_Show , draw_circle_in_Path , 5 , cv::Scalar(255), -1 );
											cv::line( Room_Inspection_Map_Show , Line_begin_Point , draw_circle_in_Path , cv::Scalar(255) , 2 );
										}
								}

							else if( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - constant) , (Center_Position_x + constant) ) == Pixel_Value_of_the_room &&
									 Room_Inspection_Robot_Ground_Clearence( Room_Inspection_RI , (Center_Position_x + constant) , (Center_Position_y - constant) , Pixel_Value_of_the_room )
								   )
								{
									ac.waitForServer(ros::Duration(5.0));
									ac.sendGoal( Move_in_pixel ( Center_Position_x + constant , Center_Position_y - constant ));
									ac.waitForResult();
									Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - constant) , (Center_Position_x + constant) ) = 255 ;
									Center_Position_y = Center_Position_y - constant;
									Center_Position_x = Center_Position_x + constant;
								}

							while( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - constant) , (Center_Position_x) ) == Pixel_Value_of_the_room &&
									 Room_Inspection_Robot_Ground_Clearence( Room_Inspection_RI , (Center_Position_x) , (Center_Position_y - constant) , Pixel_Value_of_the_room )
								   )
								{
									ac.waitForServer(ros::Duration(5.0));
									ac.sendGoal( Move_in_pixel ( Center_Position_x , Center_Position_y - constant ));
									if(ac.waitForResult())
										{
											Line_begin_Point.x = Center_Position_x ;
											Line_begin_Point.y = Center_Position_y ;
											Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - constant) , (Center_Position_x) ) = 255 ;
											Center_Position_y =  Center_Position_y - constant;
											Center_Position_x = Center_Position_x;
											draw_circle_in_Path.x = Center_Position_x;
											draw_circle_in_Path.y =  Center_Position_y - constant;
											cv::circle( Room_Inspection_Map_Show , draw_circle_in_Path , 5 , cv::Scalar(255), -1 );
											cv::line( Room_Inspection_Map_Show , Line_begin_Point , draw_circle_in_Path , cv::Scalar(255) , 2 );
										}
								}

							else if( Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - constant) , (Center_Position_x - constant) ) == Pixel_Value_of_the_room &&
									 Room_Inspection_Robot_Ground_Clearence( Room_Inspection_RI , (Center_Position_x - constant) , (Center_Position_y - constant) , Pixel_Value_of_the_room )
								   )
								{
									ac.waitForServer(ros::Duration(5.0));
									ac.sendGoal( Move_in_pixel ( Center_Position_x - constant , Center_Position_y - constant ));
									ac.waitForResult();
									Room_Inspection_RI.at<unsigned char> ( (Center_Position_y - constant) , (Center_Position_x - constant) ) = 255 ;
									Center_Position_y = Center_Position_y - constant;
									Center_Position_x = Center_Position_x - constant;
								}

							//constant = constant + ( 1 / map_resolution_);

							cv::imshow( "Room Inspection", Room_Inspection_Map_Show );
							cv::waitKey(100);
						}
			}

		//MoveBaseClient ac("move_base", true);
		ac.waitForServer(ros::Duration(5.0));
		ac.sendGoal( Move_in_pixel( Center_Position_x , Center_Position_y ));
		ac.waitForResult();

	}

 */
















