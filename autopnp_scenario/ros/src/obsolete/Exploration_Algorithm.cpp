#include <ros/ros.h>
#include <autopnp_scenario/Exploration_Algorithm.h>
#include <iostream>
//#include <algorithm>
//#include <iterator>
#include <complex>

#include "opencv2/objdetect/objdetect.hpp"
//#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>


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

		cv::imshow("Original Map", map_);

		std::vector< std::vector <cv::Point> > contours_temp;
		std::vector< std::vector <cv::Point> > saved_contours;

		cv::Mat temp = map_.clone();
		cv::Mat Expanded_Map;
		cv::Mat Contour_map;
		std::vector<cv::Vec4i> hierarchy;

		//When mode is needed
		cv::Mat New_map = map_.clone();


		for ( int i = 0 ; i < 1.5  / map_resolution_ ; i++ )
			{

				double area = 0.0;
				cv::erode(temp, Expanded_Map, cv::Mat(), cv::Point(-1,-1), 1);
				cv::imshow("Expanded Map", Expanded_Map );
				temp = Expanded_Map;
				Contour_map = Expanded_Map.clone();

				//*********************** Find contours and Draw Contours*******************

				cv::findContours( Contour_map , contours_temp , hierarchy , CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
				cv::drawContours( Contour_map , contours_temp, -1, cv::Scalar( 128 , 128 , 128 , 128 ), 2);
				//cv::line( Contour_map , convertFromMeterToPixelCoordinates<cv::Point>(Pose(1.f,0.2f,0.f)), convertFromMeterToPixelCoordinates<cv::Point>(Pose(-1.f,0.2f,0.f)), cv::Scalar(0,0,0,0), 2);
				cv::imshow("contour areas from Original Image", Contour_map);

				//*********************** Find contours and Draw Contours*******************


				//***********************Print Contour size and pixel positions*****************

				std::cout<<"\n-----------------------------------------\nContour size: "<<contours_temp.size()<<"\n\n"<<"Contour Array: \n";

				for(unsigned int i=0; i<contours_temp.size(); i++)    //This loops on the rows.
					{
						for( int idx = 0 ; idx >= 0; idx = hierarchy[idx][0] ) //This loops on the columns
							{
								std::cout << contours_temp[i][idx]  << "  ";
							}
						std::cout << std::endl;
					}

				//***********************Print Contour size and pixel positions*****************


				//***********************Print Area Save Contour in another Matrix Array*********

				std::cout<<"\n"<<"Contour Areas: \n";

				for( int idx = 0 ; idx >= 0; idx = hierarchy[idx][0] )
				//for (unsigned int idx=0; idx<contours_temp.size(); idx++)
					{
						area = map_resolution_ * map_resolution_ * cv::contourArea(contours_temp[idx]);

						std::cout << "area[" << idx << "]: "<< area << std::endl;

						if( 3.0 < area && area < 40.0 && contours_temp.size() != 0 )
							{
								//cv::drawContours(temp , contours_temp , idx , cv::Scalar(0) , CV_FILLED , 8 , hierarchy , 2 );
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
						//for (unsigned int idx=0; idx<saved_contours.end(); idx++)
							{
								std::cout << saved_contours[i][idx]  << "  ";
							}
						std::cout << std::endl;
					}

				//***********************Print Saved Contour size and Saved Contour Pixel position*********************

				std::cout<<"\n-----------------------------------------\n";

				cv::waitKey();
			}


		//**********************Draw the saved Contours in the clone version of original image***************

		//When mode is needed
		int count = 10;
		//cv::Scalar color_to_fill( rand()% 30 + 128 ) ;  //,rand()%10 +245, rand()%30 + 120, rand()%30 + 200 );
		for( unsigned int idx = 0; idx < saved_contours.size(); idx++ )
			{
				cv::drawContours( New_map , saved_contours , idx , cv::Scalar( count ) , -1 );
				cv::imshow("New Map with saved contour", New_map);
				count = count + 30 ;
				//count = count + 3 ;
				//cv::imwrite("Wg_mao.png",New_map);
				cv::waitKey();
			}

		//**********************Draw the saved Contours in the clone version of original image***************


		//*********** To draw the Obstacle in the modified map*********************

		cv::Mat New_obstacle_map =New_map.clone();
		std::vector <cv::Point> Black_Pixel;
		for(int y = 0; y < map_.cols; y++)
			{
				for(int x = 0; x < map_.rows; x++)
					{
						if (map_.at<unsigned char>(x,y) == 0 )
							{
								Black_Pixel.push_back(cv::Point( y , x));
							}
					}
			}

		std::cout<<"\nBlack Pixel array size: "<<Black_Pixel.size()<<"\n";

		for( unsigned int idx = 0; idx < Black_Pixel.size(); idx++ )
			{
				New_obstacle_map.at<unsigned char>(Black_Pixel[idx]) = 0;
			}
		cv::imshow("New Map with saved contour and the obstacles", New_obstacle_map);
		cv::waitKey();

		//*********** To draw the Obstacle in the modified map*********************


		//************Replica-Padding to the Image region*************

		cv::Mat Complete_Map = New_obstacle_map.clone() ;
		int y , x = 0;

		//int neighbour_X = { -1 , 0 , 1 , -1 , 1 , -1 , 0 , 1 };
		//int neighbour_Y = { -1 , -1 , -1 , 0 , 0 , 1 , 1 , 1 };

/*
		//Top Fill-Up
		for( y = map_.cols; y >0 ; y--)
			{
				for( x = map_.rows; x > 0 ; x--)
					{
						if ( Complete_Map.at < unsigned char > (x,y) != 0 && Complete_Map.at < unsigned char > (x,y) != 255 )
							{
								while( Complete_Map.at < unsigned char > (x-1,y) == 255 )
									{
										Complete_Map.at < unsigned char > ( cv::Point (y , x-1 )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
									}
							}
					}
			}

		//Bottom Fill-Up
		for( y = 0; y < map_.cols; y++)
			{
				for( x = 0; x < map_.rows; x++)
					{
						if ( Complete_Map.at < unsigned char > (x,y) != 0 && Complete_Map.at < unsigned char > (x,y) != 255 )
							{

								while( Complete_Map.at < unsigned char > (x+1,y) == 255 )
									{
										Complete_Map.at < unsigned char > ( cv::Point (y , x+1 )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
									}
							}
					}
			}

		//Left Fill-Up
		for( y = map_.cols; y >0 ; y--)
			{
				for( x = map_.rows; x > 0 ; x--)
					{
						if ( Complete_Map.at < unsigned char > (x,y) != 0 && Complete_Map.at < unsigned char > (x,y) != 255 )
							{
								while( Complete_Map.at < unsigned char > (x,y-1) == 255 )
									{
										Complete_Map.at < unsigned char > ( cv::Point (y-1 , x )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
									}
							}
					}
			}

		//Right Fill-Up
		for( y = 0; y < map_.cols; y++)
			{
				for( x = 0; x < map_.rows; x++)
					{
						if ( Complete_Map.at < unsigned char > (x,y) != 0 && Complete_Map.at < unsigned char > (x,y) != 255 )
							{
								while( Complete_Map.at < unsigned char > (x,y+1) == 255 )
									{
										Complete_Map.at < unsigned char > ( cv::Point (y+1 , x )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
									}
							}
					}
			}

		//Left-Top Fill-Up
		for( y = map_.cols; y >0 ; y--)
			{
				for( x = map_.rows; x > 0 ; x--)
					{
						if ( Complete_Map.at < unsigned char > (x,y) != 0 && Complete_Map.at < unsigned char > (x,y) != 255 )
							{
								while( Complete_Map.at < unsigned char > (x-1,y-1) == 255 )
									{
										Complete_Map.at < unsigned char > ( cv::Point (y-1 , x-1 )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
									}
							}
					}
			}

		//Right-Bottom Fill-Up
		for( y = 0; y < map_.cols; y++)
			{
				for( x = 0; x < map_.rows; x++)
					{
						if ( Complete_Map.at < unsigned char > (x,y) != 0 && Complete_Map.at < unsigned char > (x,y) != 255 )
							{
								while( Complete_Map.at < unsigned char > (x+1,y+1) == 255 )
									{
										Complete_Map.at < unsigned char > ( cv::Point (y+1 , x+1 )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
									}
							}
					}
			}


		for( y = map_.cols; y >0 ; y--)
			{
				for( x = map_.rows; x > 0 ; x--)
					{
						if ( Complete_Map.at < unsigned char > (x,y) != 0 && Complete_Map.at < unsigned char > (x,y) != 255 )
							{
								if( Complete_Map.at < unsigned char > (x-1,y) == 255 ||
									Complete_Map.at < unsigned char > (x+1,y) == 255 ||
									Complete_Map.at < unsigned char > (x-1,y) == 255 ||
									Complete_Map.at < unsigned char > (x,y+1) == 255 ||
									Complete_Map.at < unsigned char > (x-1,y-1) == 255 ||
									Complete_Map.at < unsigned char > (x+1,y+1) == 255 )
									{
										Complete_Map.at < unsigned char > ( cv::Point (y , x-1 )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
										Complete_Map.at < unsigned char > ( cv::Point (y , x+1 )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
										Complete_Map.at < unsigned char > ( cv::Point (y-1 , x )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
										Complete_Map.at < unsigned char > ( cv::Point (y+1 , x )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
										Complete_Map.at < unsigned char > ( cv::Point (y-1 , x-1 )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
										Complete_Map.at < unsigned char > ( cv::Point (y+1 , x+1 )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
									}
							}
					}
			}
*/
		cv::Mat New_Filled_Map = New_obstacle_map.clone();
		std::vector <cv::Point> NeighbourHood_Pixel;
		//std::vector < unsigned char > Pixel_Value;
		for(int i =0;i<1000;i++){
		for( y = 0 ; y < map_.cols ; y++)
			{
				for( x = 0 ; x < map_.rows ; x++ )
					{
						if ( Complete_Map.at < unsigned char > (x,y) != 0 && Complete_Map.at < unsigned char > (x,y) != 255 )
							{
								//Check every Pixel where its neighborhood is already replaced or not
								if( Complete_Map.at < unsigned char > ( x-1 , y-1 ) == 255 &&
										New_Filled_Map.at < unsigned char > ( x-1 , y-1 ) != Complete_Map.at < unsigned char > ( x , y )){
									//Complete_Map.at < unsigned char > ( cv::Point ( y-1 , x-1 )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
									NeighbourHood_Pixel.push_back( cv::Point ( y-1 , x-1 ) );
									//Pixel_Value.push_back( Complete_Map.at < unsigned char > (x,y) );
									}

								if ( Complete_Map.at < unsigned char > ( x-1 , y ) == 255 &&
										New_Filled_Map.at < unsigned char > ( x-1 , y ) != Complete_Map.at < unsigned char > ( x , y )){
									//Complete_Map.at < unsigned char > ( cv::Point (y , x-1 )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
									NeighbourHood_Pixel.push_back( cv::Point ( y , x-1 ) );
									//Pixel_Value.push_back(Complete_Map.at < unsigned char > (x,y));
									}

								if ( Complete_Map.at < unsigned char > (x-1 , y+1 ) == 255 &&
										New_Filled_Map.at < unsigned char > ( x-1 , y+1 ) != Complete_Map.at < unsigned char > ( x , y ) ){
									//Complete_Map.at < unsigned char > ( cv::Point ( y+1 , x-1 )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
									NeighbourHood_Pixel.push_back( cv::Point ( y+1 , x-1 ) );
									//Pixel_Value.push_back(Complete_Map.at < unsigned char > (x,y) );
									}

								if ( Complete_Map.at < unsigned char > (x , y-1 ) == 255 &&
										New_Filled_Map.at < unsigned char > ( x , y-1 ) != Complete_Map.at < unsigned char > ( x , y )){
									//Complete_Map.at < unsigned char > ( cv::Point ( y-1 , x )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
									NeighbourHood_Pixel.push_back( cv::Point ( y-1 , x ) );
									//Pixel_Value.push_back( Complete_Map.at < unsigned char > (x,y) );
									}

								if ( Complete_Map.at < unsigned char > (x , y+1 ) == 255 &&
										New_Filled_Map.at < unsigned char > ( x , y+1 ) != Complete_Map.at < unsigned char > ( x , y )){
									//Complete_Map.at < unsigned char > ( cv::Point ( y+1 , x )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
									NeighbourHood_Pixel.push_back( cv::Point ( y+1 , x ) );
									//Pixel_Value.push_back(Complete_Map.at < unsigned char > (x,y) );
									}

								if ( Complete_Map.at < unsigned char > ( x+1 , y-1 ) == 255 &&
										New_Filled_Map.at < unsigned char > ( x+1 , y-1 ) != Complete_Map.at < unsigned char > ( x , y )){
									//Complete_Map.at < unsigned char > ( cv::Point ( y-1 , x+1 )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
									NeighbourHood_Pixel.push_back( cv::Point ( y-1 , x+1 ) );
									//Pixel_Value.push_back(Complete_Map.at < unsigned char > (x,y) );
									}

								if ( Complete_Map.at < unsigned char > ( x+1 , y ) == 255 &&
										New_Filled_Map.at < unsigned char > ( x+1 , y ) != Complete_Map.at < unsigned char > ( x , y )){
									//Complete_Map.at < unsigned char > ( cv::Point ( y , x+1 )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
									NeighbourHood_Pixel.push_back( cv::Point ( y , x+1 ) );
									//Pixel_Value.push_back( Complete_Map.at < unsigned char > (x,y) );
									}

								if ( Complete_Map.at < unsigned char > ( x+1 , y+1 &&
										New_Filled_Map.at < unsigned char > ( x+1 , y+1 ) != Complete_Map.at < unsigned char > ( x , y )) == 255 ){
									//Complete_Map.at < unsigned char > ( cv::Point ( y+1 , x+1 )) = Complete_Map.at < unsigned char > (cv::Point ( y , x ) ) ;
									NeighbourHood_Pixel.push_back( cv::Point ( y+1 , x+1 ) );
									//Pixel_Value.push_back(Complete_Map.at < unsigned char > (x,y) );
									}
							}
						for( unsigned int idx = 0; idx < NeighbourHood_Pixel.size() ; idx++ )
							{
								if(NeighbourHood_Pixel.size()!=0)
									New_Filled_Map.at<unsigned char>(NeighbourHood_Pixel[idx]) = Complete_Map.at < unsigned char > (x,y) ;
							}
						//if(NeighbourHood_Pixel.size()!=0)
						//std::cout<<"\nNeighbourHood_Pixel"<<NeighbourHood_Pixel.size();
						NeighbourHood_Pixel.clear();
						//Pixel_Value.clear();
					}

			}
		//Go for Next Check where the Replaced Pixel are now set as original
		Complete_Map=New_Filled_Map.clone();
		}

		cv::imshow("New Map with increased contour", New_Filled_Map);
		//cv::imwrite("Final_Output.png",New_Filled_Map);
		cv::waitKey();

		//************Replica-Padding to the Image region*************


		//************Bounding Box**********************************

		cv::Mat Bounding_Box_BB;
		Bounding_Box_BB = New_Filled_Map.clone();
		cv::Point pt1, pt2 , centroid ;
		std::vector < cv::Point > Center_of_Room;
		std::vector < int > Center_of_Room_x;
		std::vector < int > Center_of_Room_y;
		std::vector < double > distance_bt_Centers;

		std::vector < int > min_Y ( 255 , 100000000 );
		std::vector < int > max_Y ( 255 , 0 );
		std::vector < int > min_X ( 255 , 100000000 );
		std::vector < int > max_X ( 255 , 0 );

		for( y = 0 ; y < map_.cols ; y++)
			{
				for( x = 0 ; x < map_.rows ; x++ )
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
					Center_of_Room.push_back(centroid);
					Center_of_Room_x.push_back(centroid.x);
					Center_of_Room_y.push_back(centroid.y);

					cv::rectangle(Bounding_Box_BB , pt1, pt2, cv::Scalar(255), 1);
					cv::circle( Bounding_Box_BB , centroid , 3 , cv::Scalar(255), -1);
					//cv::line(Bounding_Box_BB, Center_of_Room[a], Center_of_Room[a+1], cv::Scalar(0,0,0,0), 2);
					//cv::imshow("bounding box", Bounding_Box_BB);
				}
			}

		for(unsigned int a=0 ; a < Center_of_Room.size(); a++)
			{
				std::cout<<"\nCenter of the bounding Box["<<a<<"]: [ "<<Center_of_Room_x[a]<<" , "<<Center_of_Room_y[a]<<" ]\n";

				double Center_distance = std::sqrt(((Center_of_Room_x[1]-Center_of_Room_x[a])*(Center_of_Room_x[1]-Center_of_Room_x[a])) +
												   ((Center_of_Room_y[1]-Center_of_Room_y[a])*(Center_of_Room_y[1]-Center_of_Room_y[a])));

				distance_bt_Centers.push_back(Center_distance);
				std::cout<<"\nCentral Distance from First Room [1] to room ["<<a<<"]: "<<distance_bt_Centers[a]<<"\n";
				cv::line(Bounding_Box_BB, Center_of_Room[1], Center_of_Room[a], cv::Scalar(255), 2);
			}

		cv::imshow("bounding box", Bounding_Box_BB);
		cv::waitKey();

		//************Bounding Box**********************************

/*
		//**********************Draw the saved Contours in the clone version of original image***************

		//When pixel intensity is needed.
		cv::Mat New_map = map_.clone();
		cv::Mat temp_1;

		//Creation of Black Mask
		cv::Mat mask = cv::Mat::zeros( map_.rows, map_.cols , CV_8UC1 );
		cv::Mat temp_mask;
		cv::Scalar avgPixelIntensity;

		for( unsigned int idx = 0; idx < saved_contours.size(); idx++ )
			{
				//cv::Scalar color( rand()%30 + 200,rand()%10 +245, rand()%30 + 120, rand()%30 + 200 );
				cv::Scalar color_mask (255);
				cv::Scalar color(128);

				temp_mask = mask.clone();
				temp_1 = map_.clone();

				//Draw mask of Contour size
				cv::drawContours( temp_mask , saved_contours , idx , color_mask , -1 );

				//Calculate the average pixel intensity in the mask area
				avgPixelIntensity = cv::mean( temp_1 , mask = temp_mask );
				std::cout << "Pixel intensity over ROI [ "<< idx << "] = "<< avgPixelIntensity.val[0] << std::endl;
				cv::imshow("New_mask_map" , temp_mask );

				//Based on Pixel Intensity set the condition
				if(avgPixelIntensity.val[0]>253)

					{
						cv::drawContours( New_map , saved_contours , idx , color , 3 );
						cv::imshow("New Map", New_map);
					}

				temp_mask.setTo( 0 , mask );
				temp_1.setTo( 0 , mask );
				cv::waitKey();
			}

			//**********************Draw the saved Contours in the clone version of original image***************
*/

		// create the inflated map
		int iterations = (int)(robotRadius/map_resolution_);
		std::cout << "iterations=" << iterations << std::endl;
		cv::erode(map_, expanded_map_, cv::Mat(), cv::Point(-1,-1), iterations);

		std::cout << "\nMap Recieved.\n";

		Execution_positive_X(0);
		//Execution_negative_X(-1);
		//Execution_positive_X(-2);
		//Execution_negative_X(-3);
		//Execution_positive_X(-4);
		//Execution_negative_X(-7);
		//Execution_positive_X(-8);


		MoveBaseClient ac("move_base", true);
		while(!ac.waitForServer(ros::Duration(5.0)))
			{
				ROS_INFO("\nWaiting for the move_base action server to come up");
			}
		//ac.sendGoal(Move ( 0.0 , 0.0 , 0.0));
		//ac.waitForResult();

	}



bool Exploration::validApproachPosition(Pose robotLocation, Pose potentialApproachPose)
	{
		// convert coordinates to pixels
		cv::Point potentialApproachPosePixel = convertFromMeterToPixelCoordinates<cv::Point>(potentialApproachPose);

		// copy expanded map
		cv::Mat expanded_map_copy = expanded_map_.clone();

		std::cout<< "mx:" << map_origin_.x << "  my:" << map_origin_.y << "  resolution:" << map_resolution_ << std::endl;

		// find the individual connected areas
		std::vector< std::vector<cv::Point> > contours;		// first index=contour index;  second index=point index within contour

		cv::findContours(expanded_map_copy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

		// display found contours
		cv::drawContours(expanded_map_copy, contours, -1, cv::Scalar(128,128,128,128), 2);
		cv::circle(expanded_map_copy, convertFromMeterToPixelCoordinates<cv::Point>(robotLocation), 5, cv::Scalar(200,200,200,200), -1);
		cv::circle(expanded_map_copy, potentialApproachPosePixel, 5, cv::Scalar(200,200,200,200), -1);
		cv::line(expanded_map_copy, convertFromMeterToPixelCoordinates<cv::Point>(Pose(1.f,0.2f,0.f)), convertFromMeterToPixelCoordinates<cv::Point>(Pose(-1.f,0.2f,0.f)), cv::Scalar(0,0,0,0), 2);
		cv::imshow("contour areas--Checking the available Points", expanded_map_copy);
		cv::waitKey(50);

		std::cout<<"\nContour size: "<<contours.size()<<"\n";


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

		return true;

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


/*
move_base_msgs::MoveBaseGoal Exploration::Move( double X , double Y , double Z )
	{

		move_base_msgs::MoveBaseGoal goal;
		geometry_msgs::PoseStamped goal_pose;

		goal_pose.header.frame_id = "map";
		goal_pose.header.stamp = ros::Time::now();

		goal_pose.pose.orientation.z = Z;
		goal_pose.pose.position.x = X;
		goal_pose.pose.position.y = Y;

		goal_pose.pose.orientation.w = 1.0;

		goal.target_pose = goal_pose;
		return goal;
	}
*/



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



void Exploration::Execution_positive_X( float temp_arg_y )
	{
		MoveBaseClient ac("move_base", true);
		while(!ac.waitForServer(ros::Duration(5.0)))
			{
				ROS_INFO("\nWaiting for the move_base action server to come up");
			}

		Pose robotLocation(0,0,0);
		Pose targetpose;
		bool validApproach ;
		bool validApproach_1;
		temp_y = temp_arg_y ;

		/*
				cv::Mat image = expanded_map_copy;
				cv::Mat color_image;
				cv::cvtColor(image,color_image,CV_GRAY2BGR);
				cv::Mat binary;
				//image=cvLoadImage("expanded_map_copy",CV_LOAD_IMAGE_COLOR);
				cv::imshow("map_image",image);
				cv::threshold(image,binary,100,255,CV_THRESH_BINARY);
				cv::imshow("Thresholded image",binary);

				 // Eliminate noise and smaller objects
				    cv::Mat fg;
				    cv::erode(binary,fg,cv::Mat(),cv::Point(-1,-1),2);

				    // Identify image pixels without objects
				    cv::Mat bg;
				    cv::dilate(binary,bg,cv::Mat(),cv::Point(-1,-1),3);
				    cv::threshold(bg,bg,1,128,cv::THRESH_BINARY_INV);

				// Create markers image
				    cv::Mat markers(binary.size(),CV_8U,cv::Scalar(0));
				    markers= fg+bg;

				markers.convertTo(markers, CV_32S);
				cv::watershed(color_image,markers);

				markers.convertTo(markers,CV_8U);
				cv::imshow("a",markers);


				cv::waitKey(50);
		*/
		for ( temp_x = 1 ; temp_x < 7 ; temp_x++ )
			{
				targetpose.x = temp_x ;
				targetpose.y = temp_y ;
				validApproach = validApproachPosition( robotLocation, targetpose );

				if (validApproach == true)
					{
						ROS_INFO("The approach position is valid.");
						std::cout<<"targetpose.x= "<<targetpose.x;
						std::cout<<"\ntargetpose.y= "<<targetpose.y << "\n\n";

						//ac.sendGoal(Move ( temp_x , temp_y, 0.0));
						//ac.waitForResult();
						temp_valid_x = temp_x;
					}
				else
					{
						ROS_INFO("The approach position is invalid.");
						std::cout<<"targetpose.x= "<<targetpose.x;
						std::cout<<"\ntargetpose.y= "<<targetpose.y<<"\n\n";
					}
			}

		targetpose.y = temp_y - 1 ;
		targetpose.x = temp_valid_x ;
		validApproach = validApproachPosition( robotLocation , targetpose );
		targetpose.y = temp_y + 1 ;
		targetpose.x = temp_valid_x ;
		validApproach_1 = validApproachPosition( robotLocation , targetpose );

		if (validApproach == true)
			{
				ROS_INFO("The approach position is valid.");
				std::cout<<"targetpose.x= "<<targetpose.x;
				std::cout<<"\ntargetpose.y= "<<targetpose.y << "\n\n";
				//ac.sendGoal(Move ( temp_valid_x , temp_y - 1 , 0.0 ));
				//ac.waitForResult();
			}

		else if (validApproach_1 == true)
			{
				ROS_INFO("The approach position is valid.");
				std::cout<<"targetpose.x= "<<targetpose.x;
				std::cout<<"\ntargetpose.y= "<<targetpose.y << "\n\n";
				//ac.sendGoal(Move ( temp_x , temp_y + 1 , 0.0 ));
				//ac.waitForResult();
			}
		else
			{
				ROS_INFO("The approach position is invalid.");
				std::cout<<"targetpose.x= "<<targetpose.x;
				std::cout<<"\ntargetpose.y= "<<targetpose.y<<"\n\n";
			}

	}



void Exploration::Execution_negative_X( float temp_arg_y )
	{
		MoveBaseClient ac("move_base", true);
		while(!ac.waitForServer(ros::Duration(5.0)))
			{
				ROS_INFO("\nWaiting for the move_base action server to come up");
			}

		Pose robotLocation(0,0,0);
		Pose targetpose;
		bool validApproach ;
		bool validApproach_1;
		temp_y = temp_arg_y ;

		for ( temp_x = 7 ; -3 < temp_x && temp_x < 8 ; temp_x-- )
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

						//ac.sendGoal(Move ( temp_x , temp_y, 0.0));
						//ac.waitForResult();
						temp_valid_x = temp_x;
					}
				else
					{
						ROS_INFO("The approach position is invalid.");
						std::cout<<"targetpose.x= "<<targetpose.x;
						std::cout<<"\ntargetpose.y= "<<targetpose.y<<"\n\n";
					}
			}

			targetpose.y = temp_y - 1 ;
			targetpose.x = temp_valid_x ;
			validApproach = validApproachPosition( robotLocation , targetpose );
			targetpose.y = temp_y + 1 ;
			targetpose.x = temp_valid_x ;
			validApproach_1 = validApproachPosition( robotLocation , targetpose );

			if (validApproach == true)
				{
					ROS_INFO("The approach position is valid.");
					std::cout<<"targetpose.x= "<<targetpose.x;
					std::cout<<"\ntargetpose.y= "<<targetpose.y << "\n\n";
					//ac.sendGoal(Move ( temp_valid_x , temp_y - 1 , 0.0 ));
					//ac.waitForResult();
				}

			else if (validApproach_1 == true)
				{
					ROS_INFO("The approach position is valid.");
					std::cout<<"targetpose.x= "<<targetpose.x;
					std::cout<<"\ntargetpose.y= "<<targetpose.y << "\n\n";
					//ac.sendGoal(Move ( temp_x , temp_y + 1 , 0.0 ));
					//ac.waitForResult();
				}
			else
				{
					ROS_INFO("The approach position is invalid.");
					std::cout<<"targetpose.x= "<<targetpose.x;
					std::cout<<"\ntargetpose.y= "<<targetpose.y<<"\n\n";
				}
//}
	}



