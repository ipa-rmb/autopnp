#include <autopnp_scenario/map_point_accessibility_check_server.h>


MapPointAccessibilityCheck::MapPointAccessibilityCheck(ros::NodeHandle nh)
: node_handle_(nh)
{
	// todo: read in parameters
	robot_radius_ = 0.4;
	// 	ros::param::get("/move_base/local_costmap/robot_radius", robot_radius_);
	map_topic_name_ = "/map";
	obstacles_topic_name_ = "/move_base/local_costmap/obstacles";
	inflated_obstacles_topic_name_ = "/move_base/local_costmap/inflated_obstacles";

	// receive ground floor map once
	mapInit(node_handle_);

	// then set up dynamic obstacle callbacks
	inflationInit(node_handle_);

	// advertise services
	map_points_accessibility_check_server_ = node_handle_.advertiseService("map_points_accessibility_check", &MapPointAccessibilityCheck::checkPose2DArrayCallback, this);
	map_perimeter_accessibility_check_server_ = node_handle_.advertiseService("map_perimeter_accessibility_check", &MapPointAccessibilityCheck::checkPerimeterCallback, this);

	ROS_INFO("MapPointAccessibilityCheck initialized.");
}

void MapPointAccessibilityCheck::mapInit(ros::NodeHandle& nh)
{
	map_data_recieved_ = false;
	map_msg_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic_name_, 1, &MapPointAccessibilityCheck::mapDataCallback, this);
	ROS_INFO("MapPointAccessibilityCheck: Waiting to receive map...");
	while (map_data_recieved_ == false)
		ros::spinOnce();
	ROS_INFO("MapPointAccessibilityCheck: Map received.");
}

void MapPointAccessibilityCheck::inflationInit(ros::NodeHandle& nh)
{
	obstacles_sub_.subscribe(nh, obstacles_topic_name_, 1);
	inflated_obstacles_sub_.subscribe(nh, inflated_obstacles_topic_name_, 1);

	inflated_obstacles_sub_sync_ = boost::shared_ptr<message_filters::Synchronizer<InflatedObstaclesSyncPolicy> >(new message_filters::Synchronizer<InflatedObstaclesSyncPolicy>(InflatedObstaclesSyncPolicy(5)));
	inflated_obstacles_sub_sync_->connectInput(obstacles_sub_, inflated_obstacles_sub_);
	inflated_obstacles_sub_sync_->registerCallback(boost::bind(&MapPointAccessibilityCheck::obstacleDataCallback, this, _1, _2));
}

void MapPointAccessibilityCheck::mapDataCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg_data)
{
	inverse_map_resolution_ = 1. / map_msg_data->info.resolution;
	map_origin_ = cv::Point2d(map_msg_data->info.origin.position.x, map_msg_data->info.origin.position.y);
	std::cout << "map resolution: " << map_msg_data->info.resolution << std::endl;

	// create original map
	original_map_ = 255 * cv::Mat::ones(map_msg_data->info.height, map_msg_data->info.width, CV_8UC1);
	for (unsigned int v = 0, i = 0; v < map_msg_data->info.height; v++)
	{
		for (unsigned int u = 0; u < map_msg_data->info.width; u++, i++)
		{
			if (map_msg_data->data[i] != 0)
				original_map_.at<unsigned char>(v, u) = 0;
		}
	}

	// compute inflated static map
	std::cout << "inflation thickness: " << robot_radius_ << std::endl;
	cv::erode(original_map_, inflated_original_map_, cv::Mat(), cv::Point(-1,-1), cvRound(robot_radius_*inverse_map_resolution_));

	// todo: you can comment the following two lines to not pop up the inflated original map
//	cv::imshow("Inflated Original Map", inflated_original_map_);
//	cv::waitKey();

	map_data_recieved_ = true;
	map_msg_sub_.shutdown();
}

void MapPointAccessibilityCheck::obstacleDataCallback(const nav_msgs::GridCells::ConstPtr& obstacles_data, const nav_msgs::GridCells::ConstPtr& inflated_obstacles_data)
{
	{
		boost::mutex::scoped_lock lock(mutex_inflated_map_);

		inflated_map_ = inflated_original_map_.clone();
		for (unsigned int i=0; i<obstacles_data->cells.size(); ++i)
			inflated_map_.at<uchar>((obstacles_data->cells[i].y - map_origin_.y) * inverse_map_resolution_, (obstacles_data->cells[i].x - map_origin_.x) * inverse_map_resolution_) = 0;

		for (unsigned int i = 0; i < inflated_obstacles_data->cells.size(); i++)
			inflated_map_.at<uchar>((inflated_obstacles_data->cells[i].y - map_origin_.y) * inverse_map_resolution_, (inflated_obstacles_data->cells[i].x - map_origin_.x) * inverse_map_resolution_) = 0;
	}

	//TODO: you can comment the following two lines to not pop up the inflated original map
//	cv::imshow("Inflated Map", inflated_map_);
//	cv::waitKey(10);
}

bool MapPointAccessibilityCheck::checkPose2DArrayCallback(autopnp_scenario::CheckPointAccessibility::Request &req, autopnp_scenario::CheckPointAccessibility::Response &res)
{
	ROS_INFO("Received request to check accessibility of %i points.",req.points_to_check.size());

	res.accessibility_flags.resize(req.points_to_check.size(), true);
	{
		boost::mutex::scoped_lock lock(mutex_inflated_map_);

//		cv::Mat display_map = inflated_map_.clone();
		for (unsigned int i=0; i<req.points_to_check.size(); ++i)
		{
			if (inflated_map_.at<uchar>((req.points_to_check[i].y-map_origin_.y)*inverse_map_resolution_, (req.points_to_check[i].x-map_origin_.x)*inverse_map_resolution_) == 0)
				res.accessibility_flags[i] = false;

//			if (res.accessibility_flags[i] == false)
//				cv::circle(display_map, cv::Point((req.points_to_check[i].x-map_origin_.x)*inverse_map_resolution_, (req.points_to_check[i].y-map_origin_.y)*inverse_map_resolution_), 2, cv::Scalar(64), 2);
//			else
//				cv::circle(display_map, cv::Point((req.points_to_check[i].x-map_origin_.x)*inverse_map_resolution_, (req.points_to_check[i].y-map_origin_.y)*inverse_map_resolution_), 2, cv::Scalar(192), 2);
		}

//		cv::imshow("points", display_map);
//		cv::waitKey(10);
	}

	return true;
}

bool MapPointAccessibilityCheck::checkPerimeterCallback(autopnp_scenario::CheckPerimeterAccessibility::Request &req, autopnp_scenario::CheckPerimeterAccessibility::Response &res)
{
	ROS_INFO("Received request to check accessibility of point (%f,%f).",req.center.x, req.center.y);

	{
		boost::mutex::scoped_lock lock(mutex_inflated_map_);

//		cv::Mat display_map = inflated_map_.clone();
		for (double angle=req.center.theta; angle<req.center.theta+2*CV_PI; angle+=req.rotational_sampling_step)
		{
			double x = req.center.x + req.radius * cos(angle);
			double y = req.center.y + req.radius * sin(angle);
			int u = (x-map_origin_.x)*inverse_map_resolution_;
			int v = (y-map_origin_.y)*inverse_map_resolution_;
			if (inflated_map_.at<uchar>(v, u) == 255)
			{
				// add accessible point to results
				geometry_msgs::Pose2D pose;
				pose.x = x;
				pose.y = y;
				pose.theta = angle + CV_PI;
				while (pose.theta > 2*CV_PI)
					pose.theta -= 2*CV_PI;
				while (pose.theta < 0.)
					pose.theta += 2*CV_PI;
				res.accessible_poses_on_perimeter.push_back(pose);

//				cv::circle(display_map, cv::Point(u, v), 2, cv::Scalar(192), 2);
			}
		}

//		cv::imshow("perimeter", display_map);
//		cv::waitKey(10);
	}

	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_point_accessibility_check_server");

	ros::NodeHandle nh;

	MapPointAccessibilityCheck map_point_accessibility_check(nh);

	ros::spin();

	return 0;
}

