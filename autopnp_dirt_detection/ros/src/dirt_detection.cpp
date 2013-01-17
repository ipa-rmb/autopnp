#include <autopnp_dirt_detection/dirt_detection.h>
#include <autopnp_dirt_detection/timer.h>

#include <set>
#include <time.h>

using namespace ipa_DirtDetection;
using namespace std;
using namespace cv;


struct lessPoint2i : public binary_function<cv::Point2i, cv::Point2i, bool>
{
	bool operator()(const cv::Point2i& a, const cv::Point2i& b) const
	{ return ((a.x<b.x) || ((a.x==b.x) && (a.y<b.y))); }
};


/////////////////////////////////////////////////
// Constructor
/////////////////////////////////////////////////


DirtDetection::DirtDetection(ros::NodeHandle node_handle)
: node_handle_(node_handle), transform_listener_(node_handle)
{
	it_ = 0;
	rosbagMessagesProcessed_ = 0;
	labelingStarted_ = false;
	lastIncomingMessage_ = ros::Time::now();
}

/////////////////////////////////////////////////
//  Destructor
/////////////////////////////////////////////////

DirtDetection::~DirtDetection()
{
	if (it_ != 0) delete it_;
}

/////////////////////////////////////////////////
// Create subscribers
/////////////////////////////////////////////////

void DirtDetection::init()
{
	// Parameters
	std::cout << "\n--------------------------\nDirt Detection Parameters:\n--------------------------\n";
	node_handle_.param("dirt_detection/spectralResidualGaussianBlurIterations", spectralResidualGaussianBlurIterations_, 2);
	std::cout << "spectralResidualGaussianBlurIterations = " << spectralResidualGaussianBlurIterations_ << std::endl;
	node_handle_.param("dirt_detection/dirtThreshold", dirtThreshold_, 0.5);
	std::cout << "dirtThreshold = " << dirtThreshold_ << std::endl;
	node_handle_.param("dirt_detection/spectralResidualNormalizationHighestMaxValue", spectralResidualNormalizationHighestMaxValue_, 1500.0);
	std::cout << "spectralResidualNormalizationHighestMaxValue = " << spectralResidualNormalizationHighestMaxValue_ << std::endl;
	node_handle_.param("dirt_detection/spectralResidualImageSizeRatio", spectralResidualImageSizeRatio_, 0.25);
	std::cout << "spectralResidualImageSizeRatio = " << spectralResidualImageSizeRatio_ << std::endl;
	node_handle_.param("dirt_detection/dirtCheckStdDevFactor", dirtCheckStdDevFactor_, 2.5);
	std::cout << "dirtCheckStdDevFactor = " << dirtCheckStdDevFactor_ << std::endl;
	node_handle_.param("dirt_detection/modeOfOperation", modeOfOperation_, 0);
	std::cout << "modeOfOperation = " << modeOfOperation_ << std::endl;
	node_handle_.param("dirt_detection/warpImage", warpImage_, true);
	std::cout << "warpImage = " << warpImage_ << std::endl;
	node_handle_.param("dirt_detection/birdEyeResolution", birdEyeResolution_, 300.0);
	std::cout << "birdEyeResolution = " << birdEyeResolution_ << std::endl;
	node_handle_.param("dirt_detection/maxDistanceToCamera", maxDistanceToCamera_, 300.0);
	std::cout << "maxDistanceToCamera = " << maxDistanceToCamera_ << std::endl;
	node_handle_.param("dirt_detection/removeLines", removeLines_, true);
	std::cout << "removeLines = " << removeLines_ << std::endl;
	node_handle_.param("dirt_detection/gridResolution", gridResolution_, 20.0);
	std::cout << "gridResolution = " << gridResolution_ << std::endl;
	double gox, goy;
	node_handle_.param("dirt_detection/gridOrigin_x", gox, 0.0);
	std::cout << "gridOrigin_x = " << gox << std::endl;
	node_handle_.param("dirt_detection/gridOrigin_y", goy, 0.0);
	std::cout << "gridOrigin_y = " << goy << std::endl;
	gridOrigin_ = cv::Point2d(gox, goy);
	node_handle_.param("dirt_detection/floorSearchIterations", floorSearchIterations_, 3);
	std::cout << "floorSearchIterations = " << floorSearchIterations_ << std::endl;
	node_handle_.param("dirt_detection/minPlanePoints", minPlanePoints_, 0);
	std::cout << "minPlanePoints = " << minPlanePoints_ << std::endl;
	node_handle_.param("dirt_detection/planeNormalMaxZ", planeNormalMaxZ_, -0.5);
	std::cout << "planeNormalMaxZ = " << planeNormalMaxZ_ << std::endl;
	node_handle_.param("dirt_detection/planeMaxHeight", planeMaxHeight_, 0.3);
	std::cout << "planeMaxHeight = " << planeMaxHeight_ << std::endl;
	node_handle_.param("dirt_detection/experimentFolder", experimentFolder_, std::string(""));
	std::cout << "experimentFolder = " << experimentFolder_ << std::endl;
	node_handle_.param("dirt_detection/labelingFilePath", labelingFilePath_, std::string(""));
	std::cout << "labelingFilePath = " << labelingFilePath_ << std::endl;

	node_handle_.param("dirt_detection/showOriginalImage", debug_["showOriginalImage"], true);
	std::cout << "showOriginalImage = " << debug_["showOriginalImage"] << std::endl;
	node_handle_.param("dirt_detection/showPlaneColorImage", debug_["showPlaneColorImage"], true);
	std::cout << "showPlaneColorImage = " << debug_["showPlaneColorImage"] << std::endl;
	node_handle_.param("dirt_detection/showWarpedOriginalImage", debug_["showWarpedOriginalImage"], false);
	std::cout << "showWarpedOriginalImage = " << debug_["showWarpedOriginalImage"] << std::endl;
	node_handle_.param("dirt_detection/showSaliencyBadScale", debug_["showSaliencyBadScale"], false);
	std::cout << "showSaliencyBadScale = " << debug_["showSaliencyBadScale"] << std::endl;
	node_handle_.param("dirt_detection/showColorWithArtificialDirt", debug_["showColorWithArtificialDirt"], false);
	std::cout << "showColorWithArtificialDirt = " << debug_["showColorWithArtificialDirt"] << std::endl;
	node_handle_.param("dirt_detection/showSaliencyWithArtificialDirt", debug_["showSaliencyWithArtificialDirt"], false);
	std::cout << "showSaliencyWithArtificialDirt = " << debug_["showSaliencyWithArtificialDirt"] << std::endl;
	node_handle_.param("dirt_detection/showSaliencyDetection", debug_["showSaliencyDetection"], true);
	std::cout << "showSaliencyDetection = " << debug_["showSaliencyDetection"] << std::endl;
	node_handle_.param("dirt_detection/showDetectedLines", debug_["showDetectedLines"], true);
	std::cout << "showDetectedLines = " << debug_["showDetectedLines"] << std::endl;
	node_handle_.param("dirt_detection/showDirtDetections", debug_["showDirtDetections"], true);
	std::cout << "showDirtDetections = " << debug_["showDirtDetections"] << std::endl;
	node_handle_.param("dirt_detection/showObservationsGrid", debug_["showObservationsGrid"], true);
	std::cout << "showObservationsGrid = " << debug_["showObservationsGrid"] << std::endl;
	node_handle_.param("dirt_detection/showDirtGrid", debug_["showDirtGrid"], true);
	std::cout << "showDirtGrid = " << debug_["showDirtGrid"] << std::endl;

	// prepare grid for dirt detection and observations
	gridPositiveVotes_ = cv::Mat::zeros(10*gridResolution_, 10*gridResolution_, CV_32SC1);
	gridNumberObservations_ = cv::Mat::zeros(gridPositiveVotes_.rows, gridPositiveVotes_.cols, CV_32SC1);

	it_ = new image_transport::ImageTransport(node_handle_);
//	color_camera_image_sub_ = it_->subscribe("image_color", 1, boost::bind(&DirtDetection::imageDisplayCallback, this, _1));
	dirt_detection_image_pub_ = it_->advertise("dirt_detections", 1);

	if (modeOfOperation_ == 0)	// detection
	{
		camera_depth_points_sub_ =  node_handle_.subscribe<sensor_msgs::PointCloud2>("colored_point_cloud", 1, &DirtDetection::planeDetectionCallback, this);
		detection_map_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("detection_map", 1);
	}
	else if (modeOfOperation_ == 1)		// labeling
	{
		camera_depth_points_sub_ =  node_handle_.subscribe<sensor_msgs::PointCloud2>("colored_point_cloud", 1, &DirtDetection::planeLabelingCallback, this);
	}
	else if (modeOfOperation_ == 2)		// database evaluation
	{
		camera_depth_points_sub_ =  node_handle_.subscribe<sensor_msgs::PointCloud2>("colored_point_cloud", 5, &DirtDetection::planeDetectionCallback, this);
		camera_depth_points_from_bag_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("colored_point_cloud_bagpub", 1);
		clock_pub_ = node_handle_.advertise<rosgraph_msgs::Clock>("/clock", 1);
		ground_truth_map_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("ground_truth_map", 1);
		detection_map_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("detection_map", 1);
		databaseTest();
	}

//	floor_plane_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("floor_plane", 1);
}

/////////////////////////////////////////////////
// Main functions
/////////////////////////////////////////////////

int main(int argc, char **argv)
{

	ros::init(argc, argv, "dirt_detection");

	ros::NodeHandle n;

	DirtDetection id(n);
	id.init();
/*
	printf("Read samples and split them into train-samples and test-samples.\n");
	std::vector<DirtDetection::CarpetFeatures> carp_feat_vec;
	std::vector<DirtDetection::CarpetClass> carp_class_vec;

	//data used to train the different learning algorithms
	std::vector<DirtDetection::CarpetFeatures> train_feat_vec;
	std::vector<DirtDetection::CarpetClass> train_class_vec;

	//used to test the different trained algorithms
	std::vector<DirtDetection::CarpetFeatures> test_feat_vec;
	std::vector<DirtDetection::CarpetClass> test_class_vec;


	//file path to carpet files
	std::string filepath = ros::package::getPath("autopnp_dirt_detection") + "/common/files/TeppichFiles/";

	std::string name;
	//vector of carpet file names
	std::vector<std::string> name_vec;

	name = "carpet-gray.tepp"; //threshold=0.18
	name_vec.push_back(name);

	name = "carpet2.tepp"; //threshold=0.25
	name_vec.push_back(name);

	name = "kitchen1a.tepp"; //threshold=0.2
	name_vec.push_back(name);

	name = "kitchen1b.tepp"; //threshold=0.2
	name_vec.push_back(name);

	name = "wood1.tepp"; //threshold=0.25
	name_vec.push_back(name);

	name = "wood2.tepp"; //threshold=0.25
	name_vec.push_back(name);

	name = "carpet6.tepp"; //threshold=0.25
	name_vec.push_back(name);

	name = "carpet4.tepp"; //threshold=0.3
	name_vec.push_back(name);

	name = "redblackpattern.tepp"; //threshold=0.35
	name_vec.push_back(name);


	for (unsigned int i = 0; i<name_vec.size(); i++)
	{
		carp_feat_vec.clear();
		carp_class_vec.clear();

		//read carpet data from current file
		id.ReadDataFromCarpetFile(carp_feat_vec, carp_class_vec, filepath, name_vec[i]);

		//split data into train and test samples
		id.SplitIntoTrainAndTestSamples(5, carp_feat_vec, carp_class_vec,
											train_feat_vec, train_class_vec,
											test_feat_vec, test_class_vec);
	}


	//determine maximum value of the mean value feature and the standard deviation feature
	//-> needed to draw the evaluation picture
	double maxMean;
	double maxStd;
	id.ScaleSamples(train_feat_vec, maxMean, maxStd);

	std::cout << "Data read and splitted.\n"  << std::endl;
	std::cout << "Total samples=" << train_feat_vec.size() + test_feat_vec.size()  << std::endl;
	std::cout << "Number of train-samples=" << train_feat_vec.size()  << std::endl;
	std::cout << "Number of test-samples=" << test_feat_vec.size()  << std::endl;

	// switchflag = 1; <-> SVM
	// switchflag = 2; <-> normal tree
	// switchflag = 3; <-> gradient boosted tree
	int switchflag = 3;

	switch (switchflag)
	{
		case 1: //<-> SVM
		{
			printf("Create SVM...\n");
			CvSVM carp_classi;
			id.CreateCarpetClassiefierSVM(train_feat_vec, train_class_vec, carp_classi);
			printf("SVM created.\n");

			printf("Check SVM...\n");
			id.SVMEvaluation(train_feat_vec, train_class_vec, test_feat_vec, test_class_vec, carp_classi, maxMean, maxStd);
			printf("Check SVM done.\n");

			break;
		}
		case 2: //<-> normal tree
		{
			printf("Create tree model...\n");
			CvRTrees rtree;
			id.CreateCarpetClassiefierRTree(train_feat_vec, train_class_vec, rtree);
			printf("Tree model created.\n");

			printf("Check Forest...\n");
			id.RTreeEvaluation(train_feat_vec, train_class_vec, test_feat_vec, test_class_vec, rtree, maxMean, maxStd);
			printf("Check tree model done.\n");

			break;
		}
		case 3: //<-> gradient boosted tree
		{
			printf("Create gradient boosted tree model...\n");
			CvGBTrees GBtree;
			id.CreateCarpetClassiefierGBTree(train_feat_vec, train_class_vec, GBtree);
			printf("Gradient boosted tree model created.\n");

			printf("Check gradient boosted tree model...\n");
			id.GBTreeEvaluation(train_feat_vec, train_class_vec, test_feat_vec, test_class_vec, GBtree, maxMean, maxStd);
			printf("Check gradient boosted tree model done.\n");

			break;
		}
	}
*/

	//start to look for messages (loop)
	ros::spin();

	return 0;
}


void DirtDetection::databaseTest()
{

	// read in file with information about the bag files to use
	// read in individual gridOrigin
//	std::string statsFilename = ros::package::getPath("autopnp_dirt_detection") + "/common/files/apartment/stats.txt";
//	std::string statsFilenameMatlab = ros::package::getPath("autopnp_dirt_detection") + "/common/files/apartment/stats_matlab.txt";
	std::string databaseFilename = experimentFolder_ + "dirt_database.txt";
	std::ifstream dbFile(databaseFilename.c_str());
	if (dbFile.is_open()==false)
	{
		ROS_ERROR("Database '%s' could not be opened.", databaseFilename.c_str());
		return;
	}

	std::string dbPath;
	dbFile >> dbPath;
	int numberBagFiles = 0;
	dbFile >> numberBagFiles;

//	std::map<std::string, std::map<int, Statistics> > statistics;
	for (int bagIndex=0; bagIndex<numberBagFiles; bagIndex++)
	{
		std::string filename;
		dbFile >> filename;
		std::string bagFilename = dbPath + filename + ".bag";
		std::string xmlFilename = dbPath + filename + ".xml";
		double dx=0, dy=0;
		dbFile >> dx;
		dbFile >> dy;
		gridOrigin_.x = dx;
		gridOrigin_.y = dy;
	//	std::string bagFilename = ros::package::getPath("autopnp_dirt_detection") + "/common/files/apartment/linoleum_apartment_paper_slam_2012-09-07-09-37-13.bag";
	//	std::string path = "/media/SAMSUNG/rmb/dirt_detection/apartment/kitchen-clean.bag";
	//	std::string xmlFilename = ros::package::getPath("autopnp_dirt_detection") + "/common/files/apartment/linoleum_apartment_paper_slam_2012-09-07-09-37-13.xml";
		std::cout << "Reading messages from bag file " << bagFilename << " ..." << std::endl;

		// load ground truth for the current bag file
		std::vector<labelImage> groundTruthData;
		labelImage::readTxt3d(groundTruthData, xmlFilename);
		for (int i=0; i<(int)groundTruthData.size(); i++)
			for (int j=0; j<(int)groundTruthData[i].allTexts.size(); j++)
				std::cout << "gt (" << i << ": " << j << "): \t" << groundTruthData[i].allTexts[j] << "\t center (" << groundTruthData[i].allRects3d[j].center.x << "," << groundTruthData[i].allRects3d[j].center.y << "," << groundTruthData[i].allRects3d[j].center.z << ")\t width (" << groundTruthData[i].allRects3d[j].p1.x << "," << groundTruthData[i].allRects3d[j].p1.y << "," << groundTruthData[i].allRects3d[j].p1.z << ")\t height (" << groundTruthData[i].allRects3d[j].p2.x << "," << groundTruthData[i].allRects3d[j].p2.y << "," << groundTruthData[i].allRects3d[j].p2.z << ")" << std::endl;

		// create ground truth occupancy grid
		cv::Mat groundTruthGrid = cv::Mat::zeros(gridPositiveVotes_.rows, gridPositiveVotes_.cols, CV_32SC1);;
		for (int i=0; i<(int)groundTruthData.size(); i++)
			for (int j=0; j<(int)groundTruthData[i].allTexts.size(); j++)
				putDetectionIntoGrid(groundTruthGrid, groundTruthData[i].allRects3d[j]);

		// write ground truth to file
		std::string groundTruthFile = experimentFolder_ + filename + "-gt.map";
		std::ofstream outGt(groundTruthFile.c_str());
		if (outGt.is_open() == false)
		{
			ROS_ERROR("File '%s' could not be opened.", groundTruthFile.c_str());
			return;
		}
		for (int v=0; v<groundTruthGrid.rows; v++)
		{
			for (int u=0; u<groundTruthGrid.cols; u++)
				outGt << groundTruthGrid.at<int>(v,u) << "\t";
			outGt << std::endl;
		}
		outGt.close();


		// ------- begin of for loop for changing parameter
		for (dirtThreshold_ = 0.1; dirtThreshold_<=0.5; dirtThreshold_+=0.05)
		{
			std::cout << "Processing dirtThreshold=" << dirtThreshold_ << std::endl;

			// reset results
			gridPositiveVotes_ = cv::Mat::zeros(10*gridResolution_, 10*gridResolution_, CV_32SC1);
			gridNumberObservations_ = cv::Mat::zeros(gridPositiveVotes_.rows, gridPositiveVotes_.cols, CV_32SC1);

			// play bag file, collect detections
			rosbag::Bag bag;
			bag.open(bagFilename, rosbag::bagmode::Read);

			std::vector<std::string> topics;
			topics.push_back(std::string("/tf"));
			topics.push_back(std::string("/cam3d/rgb/points"));

			rosbag::View view(bag, rosbag::TopicQuery(topics));

			Timer timer;
			timer.start();
			int rosbagMessagesSent = 0;
			rosbagMessagesProcessed_ = 0;
			BOOST_FOREACH(rosbag::MessageInstance const m, view)
			{
				rosgraph_msgs::Clock clock;
				clock.clock = ros::Time(timer.getElapsedTimeInSec());
				clock_pub_.publish(clock);

				tf::tfMessage::ConstPtr transform = m.instantiate<tf::tfMessage>();
				if (transform != NULL)
				{
					transform_broadcaster_.sendTransform(transform->transforms);
					//ROS_INFO_STREAM(transform_listener_.allFramesAsString());
				}

				sensor_msgs::PointCloud2::ConstPtr cloud = m.instantiate<sensor_msgs::PointCloud2>();
				if (cloud != NULL)
				{
					if (rosbagMessagesSent % 50 == 0)
						std::cout << "." << std::flush;
					//std::cout << "proc: " << rosbagMessagesProcessed_ << "/" << rosbagMessagesSent << std::endl;
					rosbagMessagesSent++;
					//if (rosbagMessagesSent % 20 == 0)
						camera_depth_points_from_bag_pub_.publish(cloud);

					//while (rosbagMessagesProcessed_ < rosbagMessagesSent)
					ros::spinOnce();
				}
			}

			std::cout << "\nfinished after " << timer.getElapsedTimeInSec() << "s." << std::endl;

			if (rosbagMessagesProcessed_ != rosbagMessagesSent)
				ROS_ERROR("DirtDetection::databaseTest: Only %d/%d messages processed from the provided bag file.", rosbagMessagesProcessed_, rosbagMessagesSent);
			else
				ROS_INFO("DirtDetection::databaseTest: %d messages processed from the provided bag file.", rosbagMessagesProcessed_);

			bag.close();

			// create ground truth occupancy grid
			nav_msgs::OccupancyGrid groundTruthMap;
			groundTruthMap.header.stamp = ros::Time::now();
			groundTruthMap.header.frame_id = "/map";
			groundTruthMap.info.resolution = 1.0/gridResolution_;
			groundTruthMap.info.width = groundTruthGrid.cols;
			groundTruthMap.info.height = groundTruthGrid.rows;
			groundTruthMap.info.origin.position.x = -groundTruthGrid.cols/2 / (-gridResolution_) + gridOrigin_.x;
			groundTruthMap.info.origin.position.y = -groundTruthGrid.rows/2 / gridResolution_ + gridOrigin_.y;
			groundTruthMap.info.origin.position.z = 0;
			btQuaternion rot(0,3.14159265359,0);
			groundTruthMap.info.origin.orientation.x = rot.getX();
			groundTruthMap.info.origin.orientation.y = rot.getY();
			groundTruthMap.info.origin.orientation.z = rot.getZ();
			groundTruthMap.info.origin.orientation.w = rot.getW();
			groundTruthMap.data.resize(groundTruthGrid.cols*groundTruthGrid.rows);
			for (int v=0, i=0; v<groundTruthGrid.rows; v++)
				for (int u=0; u<groundTruthGrid.cols; u++, i++)
					groundTruthMap.data[i] = (groundTruthGrid.at<int>(v,u)==0) ? (int8_t)0 : (int8_t)100;
			ground_truth_map_pub_.publish(groundTruthMap);

			// create occupancy grid map from detections
			nav_msgs::OccupancyGrid detectionMap;
			detectionMap.header.stamp = ros::Time::now();
			detectionMap.header.frame_id = "/map";
			detectionMap.info.resolution = 1.0/gridResolution_;
			detectionMap.info.width = gridPositiveVotes_.cols;
			detectionMap.info.height = gridPositiveVotes_.rows;
			detectionMap.info.origin.position.x = -gridPositiveVotes_.cols/2 / (-gridResolution_) + gridOrigin_.x;
			detectionMap.info.origin.position.y = -gridPositiveVotes_.rows/2 / gridResolution_ + gridOrigin_.y;
			detectionMap.info.origin.position.z = 0.02;
			detectionMap.info.origin.orientation.x = rot.getX();
			detectionMap.info.origin.orientation.y = rot.getY();
			detectionMap.info.origin.orientation.z = rot.getZ();
			detectionMap.info.origin.orientation.w = rot.getW();
			detectionMap.data.resize(gridPositiveVotes_.cols*gridPositiveVotes_.rows);
			for (int v=0, i=0; v<gridPositiveVotes_.rows; v++)
				for (int u=0; u<gridPositiveVotes_.cols; u++, i++)
				{
	//				if (gridPositiveVotes_.at<int>(v,u) > 0)
	//					std::cout << "p:" << gridPositiveVotes_.at<int>(v,u) << " o:" << gridNumberObservations_.at<int>(v,u) << " a:" << (100.*(double)gridPositiveVotes_.at<int>(v,u)/((double)gridNumberObservations_.at<int>(v,u))) << "   " << (int)(int8_t)(100.*(double)gridPositiveVotes_.at<int>(v,u)/((double)gridNumberObservations_.at<int>(v,u))) << std::endl;
					detectionMap.data[i] = (int8_t)(100.*(double)gridPositiveVotes_.at<int>(v,u)/((double)gridNumberObservations_.at<int>(v,u)));
				}
			detection_map_pub_.publish(detectionMap);

			// write result as image file (only black and white)
//			cv::Mat temp;
//			cv::normalize(groundTruthGrid, temp, 0., 255*256., cv::NORM_MINMAX);
//			std::string nameGt = ros::package::getPath("autopnp_dirt_detection") + "/common/files/results/" + experimentSubFolder_ + "/" + filename + "_gt.png";
//			cv::imwrite(nameGt, temp);
//			cv::normalize(gridPositiveVotes_, temp, 0., 255*256., cv::NORM_MINMAX);
//			std::string nameDet = ros::package::getPath("autopnp_dirt_detection") + "/common/files/results/" + experimentSubFolder_ + "/" + filename + "_det.png";
//			cv::imwrite(nameDet, temp);

			// save matlab readable outputs
			std::stringstream gridPositiveVotesFile;
			gridPositiveVotesFile << experimentFolder_ << filename << "-dt" << dirtThreshold_ << "-pv.map";
			std::ofstream outPv(gridPositiveVotesFile.str().c_str());
			if (outPv.is_open() == false)
			{
				ROS_ERROR("File '%s' could not be opened.", gridPositiveVotesFile.str().c_str());
				return;
			}
			for (int v=0; v<gridPositiveVotes_.rows; v++)
			{
				for (int u=0; u<gridPositiveVotes_.cols; u++)
					outPv << gridPositiveVotes_.at<int>(v,u) << "\t";
				outPv << std::endl;
			}
			outPv.close();

			std::stringstream gridNumberObservationsFile;
			gridNumberObservationsFile << experimentFolder_ << filename << "-dt" << dirtThreshold_ << "-no.map";
			std::ofstream outNo(gridNumberObservationsFile.str().c_str());
			if (outNo.is_open() == false)
			{
				ROS_ERROR("File '%s' could not be opened.", gridNumberObservationsFile.str().c_str());
				return;
			}
			for (int v=0; v<gridNumberObservations_.rows; v++)
			{
				for (int u=0; u<gridNumberObservations_.cols; u++)
					outNo << gridNumberObservations_.at<int>(v,u) << "\t";
				outNo << std::endl;
			}
			outNo.close();

		// generate statistics on detection results
//		for (int8_t dirtThreshold=10; dirtThreshold<=100; dirtThreshold+=10)
//		{
//			Statistics stat;
//			stat.setZero();
//			for (int v=0,i=0; v<groundTruthGrid.rows; v++)
//			{
//				for (int u=0; u<groundTruthGrid.cols; u++, i++)
//				{
//					// normal statistics
//					if (groundTruthMap.data[i]==(int8_t)0 && detectionMap.data[i]<dirtThreshold)
//						stat.tn++;
//					else if (groundTruthMap.data[i]==(int8_t)100 && detectionMap.data[i]>=dirtThreshold)
//						stat.tp++;
//					else if (groundTruthMap.data[i]==(int8_t)0 && detectionMap.data[i]>=dirtThreshold)
//						stat.fp++;
//					else
//						stat.fn++;
//
//					// neighborhood relaxed statistics
//					bool relaxedDirt = false;
//					for (int dv=-2; dv<=2; dv++)
//						for (int du=-2; du<=2; du++)
//							if (groundTruthMap.data[(v+dv)*groundTruthGrid.cols+u+du]==(int8_t)100)
//								relaxedDirt = true;
//					if (groundTruthMap.data[i]==(int8_t)0 && detectionMap.data[i]<dirtThreshold)
//						stat.tnr++;
//					else if (relaxedDirt==true && detectionMap.data[i]>=dirtThreshold)
//						stat.tpr++;
//					else if (relaxedDirt==false && detectionMap.data[i]>=dirtThreshold)
//						stat.fpr++;
//					else
//						stat.fnr++;
//				}
//			}
//
//			std::cout << filename << "\tdirtThreshold=" << (int)dirtThreshold << "\trecall=" << (double)stat.tp/(stat.tp+stat.fn) << "\tprecision=" << (double)stat.tp/(stat.tp+stat.fp) << "\ttp=" << stat.tp << "\tfp=" << stat.fp << "\tfn=" << stat.fn << "\ttn=" << stat.tn << std::endl;
//			std::cout << filename << "\tdirtThreshold=" << (int)dirtThreshold << "\trecallr=" << (double)stat.tpr/(stat.tpr+stat.fnr) << "\tprecisionr=" << (double)stat.tpr/(stat.tpr+stat.fpr) << "\ttpr=" << stat.tpr << "\tfpr=" << stat.fpr << "\tfnr=" << stat.fnr << "\ttnr=" << stat.tnr << std::endl;
//			statistics[filename][(int)dirtThreshold] = stat;

		}	// ------- end of for loop for changing parameter
	}

	dbFile.close();

//	// save statistics
//	std::ofstream statsFile(statsFilename.c_str());
//	std::ofstream statsFileMatlab(statsFilenameMatlab.c_str());
//	if (statsFile.is_open()==false)
//	{
//		ROS_ERROR("Statistics file '%s' could not be opened.", statsFilename.c_str());
//		return;
//	}
//	if (statsFileMatlab.is_open()==false)
//	{
//		ROS_ERROR("Statistics file '%s' could not be opened.", statsFilenameMatlab.c_str());
//		return;
//	}
//	for (std::map<std::string, std::map<int, Statistics> >::iterator itFile = statistics.begin(); itFile!=statistics.end(); itFile++)
//	{
//		for (std::map<int, Statistics>::iterator itThreshold = itFile->second.begin(); itThreshold!=itFile->second.end(); itThreshold++)
//		{
//			statsFile << itFile->first << "\t" << itThreshold->first << "\t" << itThreshold->second.tp << "\t" << itThreshold->second.fp << "\t" << itThreshold->second.fn << "\t" << itThreshold->second.tn
//					 << "\t" << itThreshold->second.tpr << "\t" << itThreshold->second.fpr << "\t" << itThreshold->second.fnr << "\t" << itThreshold->second.tnr << std::endl;
//			statsFileMatlab << itThreshold->first << "\t" << itThreshold->second.tp << "\t" << itThreshold->second.fp << "\t" << itThreshold->second.fn << "\t" << itThreshold->second.tn
//								 << "\t" << itThreshold->second.tpr << "\t" << itThreshold->second.fpr << "\t" << itThreshold->second.fnr << "\t" << itThreshold->second.tnr << std::endl;
//		}
//	}
//	std::cout << "Statistics saved at '" << statsFilename << "'." << std::endl;
//	statsFile.close();
//	statsFileMatlab.close();


	return;
}


/////////////////////////////////////////////////
// Callback functions
/////////////////////////////////////////////////

void DirtDetection::imageDisplayCallback(const sensor_msgs::ImageConstPtr& color_image_msg)
{
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	//convert message to cv::Mat
	convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

	// saliency detection
	cv::Mat result_image;
	SaliencyDetection_C3(color_image, result_image, 0, spectralResidualGaussianBlurIterations_);

	// post processing
	cv::Mat image_postproc;
	cv::Mat new_color_image = color_image;
	Image_Postprocessing_C1(result_image, image_postproc, new_color_image);

	if (debug_["showDirtDetections"] == true)
	{
		cv::imshow("dirt detections", new_color_image);
		cvMoveWindow("dirt detections", 0, 520);
	}
	cv::waitKey(10);
}

void DirtDetection::planeDetectionCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg)
{
	// get tf between camera and map
	tf::StampedTransform transformMapCamera;
	transformMapCamera.setIdentity();
	try
	{
		ros::Time time = point_cloud2_rgb_msg->header.stamp;
		std::string err;
		//std::cout << "Latest common time: " << transform_listener_.getLatestCommonTime("/map", point_cloud2_rgb_msg->header.frame_id, time, &err) << std::endl;
		transform_listener_.getLatestCommonTime("/map", point_cloud2_rgb_msg->header.frame_id, time, &err);
		transform_listener_.lookupTransform("/map", point_cloud2_rgb_msg->header.frame_id, time, transformMapCamera);
//		std::cout << "xyz: " << transformMapCamera.getOrigin().getX() << " " << transformMapCamera.getOrigin().getY() << " " << transformMapCamera.getOrigin().getZ() << "\n";
//		std::cout << "abcw: " << transformMapCamera.getRotation().getX() << " " << transformMapCamera.getRotation().getY() << " " << transformMapCamera.getRotation().getZ() << " " << transformMapCamera.getRotation().getW() << "\n";
//		std::cout << "frame_id: " << transformMapCamera.frame_id_ << "  child_frame_id: " << transformMapCamera.child_frame_id_ << std::endl;
	}
	catch (tf::TransformException ex)
	{
		ROS_WARN("%s",ex.what());
		return;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	convertPointCloudMessageToPointCloudPcl(point_cloud2_rgb_msg, input_cloud);

	// find ground plane
	cv::Mat plane_color_image = cv::Mat();
	cv::Mat plane_mask = cv::Mat();
	pcl::ModelCoefficients plane_model;
	bool found_plane = planeSegmentation(input_cloud, plane_color_image, plane_mask, plane_model, transformMapCamera, gridNumberObservations_);

	// check if a ground plane could be found
	if (found_plane == true)
	{
		//cv::cvtColor(plane_color_image, plane_color_image, CV_BGR2Lab);

//		cv::Mat laplace;
//		cv::Laplacian(plane_color_image, laplace, CV_32F, 5);
//		laplace = laplace.mul(laplace);
//		cv::normalize(laplace, laplace, 0, 1, NORM_MINMAX);
//		cv:imshow("laplace", laplace);

		// test with half-scale image
//		cv::Mat temp = plane_color_image;
//		cv::resize(temp, plane_color_image, cv::Size(), 0.5, 0.5);
//		temp = plane_mask;
//		cv::resize(temp, plane_mask, cv::Size(), 0.5, 0.5);

		// remove perspective from image
		cv::Mat H;			// homography between floor plane in image and bird's eye perspective
		cv::Mat R,t;		// transformation between world and floor plane coordinates, i.e. [xw,yw,zw] = R*[xp,yp,0]+t and [xp,yp,0] = R^T*[xw,yw,zw] - R^T*t
		cv::Point2f cameraImagePlaneOffset;		// offset in the camera image plane. Conversion from floor plane to  [xc, yc]
		cv::Mat plane_color_image_warped;
		cv::Mat plane_mask_warped;
		if (warpImage_ == true)
		{
			bool transformSuccessful = computeBirdsEyePerspective(input_cloud, plane_color_image, plane_mask, plane_model, H, R, t, cameraImagePlaneOffset, plane_color_image_warped, plane_mask_warped);
			if (transformSuccessful == false)
				return;
		}
		else
		{
			H = (cv::Mat_<double>(3,3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
			R = H;
			t = (cv::Mat_<double>(3,1) << 0.0, 0.0, 0.0);
			cameraImagePlaneOffset.x = 0.f;
			cameraImagePlaneOffset.y = 0.f;
			plane_color_image_warped = plane_color_image;
			plane_mask_warped = plane_mask;
		}

		// detect dirt on the floor
		cv::Mat C1_saliency_image;
		SaliencyDetection_C3(plane_color_image_warped, C1_saliency_image, &plane_mask_warped, spectralResidualGaussianBlurIterations_);

		// post processing, dirt/stain selection
		cv::Mat C1_BlackWhite_image;
		cv::Mat new_plane_color_image = plane_color_image_warped.clone();
		std::vector<cv::RotatedRect> dirtDetections;
		Image_Postprocessing_C1_rmb(C1_saliency_image, C1_BlackWhite_image, new_plane_color_image, dirtDetections, plane_mask_warped);

		// convert detections to map coordinates and mark dirt regions in map
		for (int i=0; i<(int)dirtDetections.size(); i++)
		{
			labelImage::RegionPointTriple pointsWorldMap;

			// center point
			cv::Mat pc;
			if (warpImage_ == true)
				pc = (cv::Mat_<double>(3,1) << (double)dirtDetections[i].center.x, (double)dirtDetections[i].center.y, 1.0);
			else
				pc = (cv::Mat_<double>(3,1) << (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].x, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].y, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].z);
			transformPointFromCameraWarpedToWorld(pc, R, t, cameraImagePlaneOffset, transformMapCamera, pointsWorldMap.center);
			//std::cout << "---------- world.x=" << pointsWorldMap.center.x << "   world.y=" << pointsWorldMap.center.y << "   world.z=" << pointsWorldMap.center.z << std::endl;

			// point in width direction
			double u = (double)dirtDetections[i].center.x+cos(-dirtDetections[i].angle*3.14159265359/180.f)*dirtDetections[i].size.width/2.f;
			double v = (double)dirtDetections[i].center.y-sin(-dirtDetections[i].angle*3.14159265359/180.f)*dirtDetections[i].size.width/2.f;
			//std::cout << "dd: " << dirtDetections[i].center.x << " " << dirtDetections[i].center.y << "  u:" << u << "  v:" << v;
			if (warpImage_ == true)
				pc = (cv::Mat_<double>(3,1) << u, v, 1.0);
			else
				//pc = (cv::Mat_<double>(3,1) << (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].x, (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].y, (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].z);
				pc = (cv::Mat_<double>(3,1) << (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].x, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].y, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].z);
			transformPointFromCameraWarpedToWorld(pc, R, t, cameraImagePlaneOffset, transformMapCamera, pointsWorldMap.p1);

			// point in height direction
			u = (double)dirtDetections[i].center.x-cos((-dirtDetections[i].angle-90)*3.14159265359/180.f)*dirtDetections[i].size.height/2.f;
			v = (double)dirtDetections[i].center.y-sin((-dirtDetections[i].angle-90)*3.14159265359/180.f)*dirtDetections[i].size.height/2.f;
			//std::cout << "   uh:" << u << "   vh:" << v << std::endl;
			if (warpImage_ == true)
				pc = (cv::Mat_<double>(3,1) << u, v, 1.0);
			else
				//pc = (cv::Mat_<double>(3,1) << (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].x, (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].y, (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].z);
				pc = (cv::Mat_<double>(3,1) << (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].x, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].y, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].z);
			transformPointFromCameraWarpedToWorld(pc, R, t, cameraImagePlaneOffset, transformMapCamera, pointsWorldMap.p2);

			putDetectionIntoGrid(gridPositiveVotes_, pointsWorldMap);
		}

		if (debug_["showDirtGrid"] == true)
		{
			cv::Mat gridPositiveVotesDisplay;
			cv::normalize(gridPositiveVotes_, gridPositiveVotesDisplay, 0., 255*256., cv::NORM_MINMAX);
			cv::imshow("dirt grid", gridPositiveVotesDisplay);
			cvMoveWindow("dirt grid", 0, 0);
		}

		// create occupancy grid map from detections
		nav_msgs::OccupancyGrid detectionMap;
		detectionMap.header.stamp = ros::Time::now();
		detectionMap.header.frame_id = "/map";
		detectionMap.info.resolution = 1.0/gridResolution_;
		detectionMap.info.width = gridPositiveVotes_.cols;
		detectionMap.info.height = gridPositiveVotes_.rows;
		detectionMap.info.origin.position.x = -gridPositiveVotes_.cols/2 / (-gridResolution_) + gridOrigin_.x;
		detectionMap.info.origin.position.y = -gridPositiveVotes_.rows/2 / gridResolution_ + gridOrigin_.y;
		detectionMap.info.origin.position.z = -0.05;
		btQuaternion rot(0,3.14159265359,0);
		detectionMap.info.origin.orientation.x = rot.getX();
		detectionMap.info.origin.orientation.y = rot.getY();
		detectionMap.info.origin.orientation.z = rot.getZ();
		detectionMap.info.origin.orientation.w = rot.getW();
		detectionMap.data.resize(gridPositiveVotes_.cols*gridPositiveVotes_.rows);
		for (int v=0, i=0; v<gridPositiveVotes_.rows; v++)
			for (int u=0; u<gridPositiveVotes_.cols; u++, i++)
				detectionMap.data[i] = (int8_t)(100.*(double)gridPositiveVotes_.at<int>(v,u)/((double)gridNumberObservations_.at<int>(v,u)));
		detection_map_pub_.publish(detectionMap);

		// publish image
		cv_bridge::CvImage cv_ptr;
		cv_ptr.image = new_plane_color_image;
		cv_ptr.encoding = "bgr8";
		dirt_detection_image_pub_.publish(cv_ptr.toImageMsg());

		if (debug_["showObservationsGrid"] == true)
		{
			cv::Mat gridObservationsDisplay;
			cv::normalize(gridNumberObservations_, gridObservationsDisplay, 0., 255*256., cv::NORM_MINMAX);
			cv::imshow("observations grid", gridObservationsDisplay);
			cvMoveWindow("observations grid", 340, 0);
		}

		if (debug_["showWarpedOriginalImage"] == true)
		{
			cv::imshow("warped original image", plane_color_image_warped);
			//cvMoveWindow("dirt grid", 0, 0);
		}

		if (debug_["showDirtDetections"] == true)
		{
			cv::imshow("dirt detections", new_plane_color_image);
			cvMoveWindow("dirt detections", 0, 530);
		}

		if (debug_["showPlaneColorImage"] == true)
		{
			cv::imshow("original color image", plane_color_image);
			cvMoveWindow("original color image", 650, 0);
		}
	}
	rosbagMessagesProcessed_++;

	cv::waitKey(10);
}

void DirtDetection::planeLabelingCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg)
{
//	if ((ros::Time::now() - lastIncomingMessage_).toSec() < 1.0)
//		return;
//
//	lastIncomingMessage_ = ros::Time::now();

	// get tf between camera and map as well
	tf::StampedTransform transformMapCamera;
	transformMapCamera.setIdentity();
	try
	{
		ros::Time time = point_cloud2_rgb_msg->header.stamp;
		std::string err;
		//std::cout << "Latest common time: " << transform_listener_.getLatestCommonTime("/map", point_cloud2_rgb_msg->header.frame_id, time, &err) << std::endl;
		transform_listener_.getLatestCommonTime("/map", point_cloud2_rgb_msg->header.frame_id, time, &err);
		transform_listener_.lookupTransform("/map", point_cloud2_rgb_msg->header.frame_id, time, transformMapCamera);
//		std::cout << "xyz: " << transformMapCamera.getOrigin().getX() << " " << transformMapCamera.getOrigin().getY() << " " << transformMapCamera.getOrigin().getZ() << "\n";
//		std::cout << "abcw: " << transformMapCamera.getRotation().getX() << " " << transformMapCamera.getRotation().getY() << " " << transformMapCamera.getRotation().getZ() << " " << transformMapCamera.getRotation().getW() << "\n";
//		std::cout << "frame_id: " << transformMapCamera.frame_id_ << "  child_frame_id: " << transformMapCamera.child_frame_id_ << std::endl;
	}
	catch (tf::TransformException ex)
	{
		ROS_WARN("%s",ex.what());
		return;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	convertPointCloudMessageToPointCloudPcl(point_cloud2_rgb_msg, input_cloud);

	// find ground plane
	cv::Mat plane_color_image = cv::Mat();
	cv::Mat plane_mask = cv::Mat();
	pcl::ModelCoefficients plane_model;
	bool found_plane = planeSegmentation(input_cloud, plane_color_image, plane_mask, plane_model, transformMapCamera, gridNumberObservations_);

//	// verify that plane is a valid ground plane
//	tf::StampedTransform rotationMapCamera = transformMapCamera;
//	rotationMapCamera.setOrigin(btVector3(0,0,0));
//	btVector3 planeNormalCamera(plane_model.values[0], plane_model.values[1], plane_model.values[2]);
//	btVector3 planeNormalWorld = rotationMapCamera * planeNormalCamera;
	//std::cout << "normCam: " << planeNormalCamera.getX() << ", " << planeNormalCamera.getY() << ", " << planeNormalCamera.getZ() << "  normW: " << planeNormalWorld.getX() << ", " << planeNormalWorld.getY() << ", " << planeNormalWorld.getZ() << std::endl;

	// check if a ground plane could be found
	if (found_plane == true)
	{
		// remove perspective from image
		cv::Mat H;			// homography between floor plane in image and bird's eye perspective
		cv::Mat R,t;		// transformation between world and floor plane coordinates, i.e. [xw,yw,zw] = R*[xp,yp,0]+t and [xp,yp,0] = R^T*[xw,yw,zw] - R^T*t
		cv::Point2f cameraImagePlaneOffset;		// offset in the camera image plane. Conversion from floor plane to  [xc, yc]
		cv::Mat plane_color_image_warped;
		cv::Mat plane_mask_warped;
		bool transformSuccessful = computeBirdsEyePerspective(input_cloud, plane_color_image, plane_mask, plane_model, H, R, t, cameraImagePlaneOffset, plane_color_image_warped, plane_mask_warped);
		if (transformSuccessful == false)
			return;

//		//btVector3 pointWorldMapBt(0.45, -4.25, 0.20);
//		//btVector3 pointWorldMapBt(0.11, -1.23, 0.11);
//		btVector3 pointWorldMapBt(2.77, 1.61, 0.11);
//		btVector3 pointWorldCameraBt = transformMapCamera.inverse() * pointWorldMapBt;
//		cv::Mat pw = (cv::Mat_<double>(3,1) << pointWorldCameraBt.getX(), pointWorldCameraBt.getY(), pointWorldCameraBt.getZ());
//		cv::Mat pp = (R.t()*pw-R.t()*t);
//		pp.at<double>(0) = birdEyeResolution_*(pp.at<double>(0)-cameraImagePlaneOffset.x);
//		pp.at<double>(1) = birdEyeResolution_*(pp.at<double>(1)-cameraImagePlaneOffset.y);
//		pp.at<double>(2) = 1.;
//		cv::Mat pc = H.inv()*pp;
//		cv::Point po(pc.at<double>(0)/pc.at<double>(2), pc.at<double>(1)/pc.at<double>(2));
//		//std::cout << "pp: " << pp.at<double>(0) << ", " << pp.at<double>(1) << ", " << pp.at<double>(2) << "   po: " << po.x << ", " << po.y << std::endl;
//		cv::circle(plane_color_image, po, 3, CV_RGB(0,255,0), 2);
//		cv::circle(plane_color_image_warped, cv::Point((int)pp.at<double>(0),(int)pp.at<double>(1)), 3, CV_RGB(0,255,0), 2);


		cv::imshow("original color image", plane_color_image);
		cvMoveWindow("original color image", 680, 0);
		cv::imshow("birds eye perspective", plane_color_image_warped);
		cvMoveWindow("birds eye perspective", 0, 520);
		int key = cv::waitKey(20);

		//std::cout << "key: " << key << std::endl;

		std::string winName = "labeling image";
		if (key == 'l' || key == 1048684)
		{
			// start labeling at current image
			cv::imshow(winName.c_str(), plane_color_image_warped);
			cvMoveWindow(winName.c_str(), 0, 0);

			cv::Mat plane_color_image_warped_copy = plane_color_image_warped.clone();
			if (labelingStarted_ == false)
			{
				labelImage labelImg(winName, plane_color_image_warped_copy, plane_color_image_warped, cv::Scalar(0, 0, 0));
				labeledImages_.push_back(labelImg);
				labelingStarted_ = true;
			}
			else
				labeledImages_[labeledImages_.size()-1].img = plane_color_image_warped_copy;
			int numberLabelBefore = labeledImages_[labeledImages_.size()-1].allRects.size();
			labeledImages_[labeledImages_.size()-1].labelingLoop();

			// after labeling, convert labeled image coordinates to floor plane coordinates
			labeledImages_[labeledImages_.size()-1].allRects3d.resize(labeledImages_[labeledImages_.size()-1].allRects.size());
			for (int i=numberLabelBefore; i<(int)labeledImages_[labeledImages_.size()-1].allRects.size(); i++)
			{
				cv::RotatedRect rectCameraCoordinates = labeledImages_[labeledImages_.size()-1].allRects[i];
				labelImage::RegionPointTriple& pointsWorldMap = labeledImages_[labeledImages_.size()-1].allRects3d[i];

				// center point
				cv::Mat pc = (cv::Mat_<double>(3,1) << (double)rectCameraCoordinates.center.x, (double)rectCameraCoordinates.center.y, 1.0);
				transformPointFromCameraWarpedToWorld(pc, R, t, cameraImagePlaneOffset, transformMapCamera, pointsWorldMap.center);
				std::cout << "---------- world.x=" << pointsWorldMap.center.x << "   world.y=" << pointsWorldMap.center.y << "   world.z=" << pointsWorldMap.center.z << std::endl;

				// point in width direction
				pc = (cv::Mat_<double>(3,1) << (double)rectCameraCoordinates.center.x+cos(-rectCameraCoordinates.angle*3.14159265359/180.f)*rectCameraCoordinates.size.width/2.f,
											   (double)rectCameraCoordinates.center.y-sin(-rectCameraCoordinates.angle*3.14159265359/180.f)*rectCameraCoordinates.size.width/2.f, 1.0);
				transformPointFromCameraWarpedToWorld(pc, R, t, cameraImagePlaneOffset, transformMapCamera, pointsWorldMap.p1);

				// point in height direction
				pc = (cv::Mat_<double>(3,1) << (double)rectCameraCoordinates.center.x-cos((-rectCameraCoordinates.angle-90)*3.14159265359/180.f)*rectCameraCoordinates.size.height/2.f,
											   (double)rectCameraCoordinates.center.y-sin((-rectCameraCoordinates.angle-90)*3.14159265359/180.f)*rectCameraCoordinates.size.height/2.f, 1.0);
				transformPointFromCameraWarpedToWorld(pc, R, t, cameraImagePlaneOffset, transformMapCamera, pointsWorldMap.p2);
			}
		}
		else if (key == 'f' || key == 1048678)
		{
			// finish labeling, write label file
			time_t t;
			t = time(NULL);
			std::stringstream ss;
			std::string currentTime;
			ss << t;
			ss >> currentTime;
			labelImage::writeTxt3d(labeledImages_, labelingFilePath_, currentTime);
			labelingStarted_ = false;
		}
		else if (key == 'q' || key == 1048689)
		{
			// quit
			ros::shutdown();
		}
	}
}


/////////////////////////////////////////////////
// Convert functions
/////////////////////////////////////////////////

void DirtDetection::convertPointCloudMessageToPointCloudPcl(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_XYZRGB)
{
	//conversion Ros message->Pcl point cloud
	pcl::fromROSMsg(*point_cloud2_rgb_msg, *point_cloud_XYZRGB);
}


unsigned long DirtDetection::convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& color_image_msg, cv_bridge::CvImageConstPtr& color_image_ptr, cv::Mat& color_image)
{
	try
	{
		color_image_ptr = cv_bridge::toCvShare(color_image_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("DirtDetection: cv_bridge exception: %s", e.what());
		return 1;
	}
	color_image = color_image_ptr->image;

	return 0;
}


bool DirtDetection::planeSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, cv::Mat& plane_color_image, cv::Mat& plane_mask, pcl::ModelCoefficients& plane_model, const tf::StampedTransform& transform_map_camera, cv::Mat& grid_number_observations)
{

	//recreate original color image from point cloud
	if (debug_["showOriginalImage"] == true)
	{
		cv::Mat color_image = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC3);
		int index = 0;
		for (int v=0; v<(int)input_cloud->height; v++)
		{
			for (int u=0; u<(int)input_cloud->width; u++, index++)
			{
				pcl::PointXYZRGB point = (*input_cloud)[index];
				bgr bgr_ = {point.b, point.g, point.r};
				color_image.at<bgr>(v, u) = bgr_;
			}
		}
		//display original image
		cv::imshow("color image", color_image);
		cvMoveWindow("color image", 650, 0);
		//cvMoveWindow("color image", 0, 520);
	}


	// try several times to find the ground plane
	bool found_plane = false;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	*filtered_input_cloud = *input_cloud;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	for (int trial=0; trial<floorSearchIterations_; trial++)
	{
		// Create the segmentation object for the planar model and set all the parameters
		inliers->indices.clear();
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (100);
		seg.setDistanceThreshold (0.05);

		seg.setInputCloud(filtered_input_cloud);
		seg.segment (*inliers, plane_model);

		// keep plane_normal upright
		if (plane_model.values[2] < 0.)
		{
			plane_model.values[0] *= -1;
			plane_model.values[1] *= -1;
			plane_model.values[2] *= -1;
			plane_model.values[3] *= -1;
		}

		// verify that plane is a valid ground plane
		if (inliers->indices.size()!=0)
		{
			tf::StampedTransform rotationMapCamera = transform_map_camera;
			rotationMapCamera.setOrigin(btVector3(0,0,0));
			btVector3 planeNormalCamera(plane_model.values[0], plane_model.values[1], plane_model.values[2]);
			btVector3 planeNormalWorld = rotationMapCamera * planeNormalCamera;

			pcl::PointXYZRGB point = (*filtered_input_cloud)[(inliers->indices[inliers->indices.size()/2])];
			btVector3 planePointCamera(point.x, point.y, point.z);
			btVector3 planePointWorld = transform_map_camera * planePointCamera;
//			std::cout << "normCam: " << planeNormalCamera.getX() << ", " << planeNormalCamera.getY() << ", " << planeNormalCamera.getZ() << "  normW: " << planeNormalWorld.getX() << ", " << planeNormalWorld.getY() << ", " << planeNormalWorld.getZ() << "   point[half]: " << planePointWorld.getX() << ", " << planePointWorld.getY() << ", " << planePointWorld.getZ() << std::endl;
//			std::cout << "pointCount: " << inliers->indices.size() << " minPlanePoints: " << minPlanePoints_ << "\n"
//					<< "planeNormalWorld.getZ(): " << planeNormalWorld.getZ() << " planeNormalMaxZ_: " << planeNormalMaxZ_ << "\n"
//					<< "abs(planePointWorld.getZ()): " << abs(planePointWorld.getZ()) << " planeMaxHeight_: " << planeMaxHeight_ << std::endl;

			// verify that the found plane is a valid ground plane
			if (inliers->indices.size()>minPlanePoints_ && planeNormalWorld.getZ()<planeNormalMaxZ_ && abs(planePointWorld.getZ())<planeMaxHeight_)
			{
				found_plane=true;
				break;
			}
			else
			{
				// the plane is not the ground plane -> remove that plane from the point cloud
				for (size_t i=0; i<inliers->indices.size(); i++)
				{
					// set all inliers to invalid
					pcl::PointXYZRGB& point = (*filtered_input_cloud)[(inliers->indices[i])];
					point.x = 0;
					point.y = 0;
					point.z = 0;
				}

//				// Extract the planar inliers from the input cloud
//				pcl::ExtractIndices<pcl::PointXYZRGB> extract;
//				extract.setInputCloud (filtered_input_cloud);
//				extract.setIndices (inliers);
//
//				// Remove the planar inliers, extract the rest
//				pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
//				extract.setNegative (true);
//				extract.filter(*temp);
//				filtered_input_cloud = temp;
			}
		}
	}

	// if the ground plane was found, write the respective data to the images
	if (found_plane == true)
	{
//		pcl::PointCloud<pcl::PointXYZRGB> floor_plane;

		plane_color_image = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC3);
		plane_mask = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC1);
		std::set<cv::Point2i, lessPoint2i> visitedGridCells;	// secures that no two observations can count twice for the same grid cell
		cv::Point2i grid_offset(grid_number_observations.cols/2, grid_number_observations.rows/2);
		for (size_t i=0; i<inliers->indices.size(); i++)
		{
			int v = inliers->indices[i]/input_cloud->width;	// check ob das immer abrundet ->noch offen!!!
			int u = inliers->indices[i] - v*input_cloud->width;

			// cropped color image and mask
			pcl::PointXYZRGB point = (*filtered_input_cloud)[(inliers->indices[i])];
			bgr bgr_ = {point.b, point.g, point.r};
			plane_color_image.at<bgr>(v, u) = bgr_;
			plane_mask.at<uchar>(v, u) = 255;

			// populate visibility grid
			btVector3 planePointCamera(point.x, point.y, point.z);
			btVector3 planePointWorld = transform_map_camera * planePointCamera;
			cv::Point2i co(-(planePointWorld.getX()-gridOrigin_.x)*gridResolution_+grid_offset.x, (planePointWorld.getY()-gridOrigin_.y)*gridResolution_+grid_offset.y);
			if (visitedGridCells.find(co)==visitedGridCells.end() && co.x>=0 && co.x<grid_number_observations.cols && co.y>=0 && co.y<grid_number_observations.rows)
			{
				// grid cell has not been incremented, yet
//				for (std::set<cv::Point2i, lessPoint2i>::iterator it=visitedGridCells.begin(); it!=visitedGridCells.end(); it++)
//				{
//					if (it->x==co.x && it->y==co.y)
//					{
//						std::cout << "co: " << co.x << " " << co.y << std::endl;
//						std::cout << "p:" << it->x << " " << it->y << std::endl;
//						getchar();
//					}
//				}
				visitedGridCells.insert(co);
				grid_number_observations.at<int>(co) = grid_number_observations.at<int>(co) + 1;
			}

//			point.z = -(plane_model.values[0]*point.x+plane_model.values[1]*point.y+plane_model.values[3])/plane_model.values[2];
//			floor_plane.push_back(point);
		}

		//display detected floor
		//cv::imshow("floor cropped", ground_image);


//		sensor_msgs::PointCloud2 floor_cloud;
//		pcl::toROSMsg(floor_plane, floor_cloud);
//		floor_cloud.header.stamp = input_cloud->header.stamp;
//		floor_cloud.header.frame_id = input_cloud->header.frame_id;
//		floor_plane_pub_.publish(floor_cloud);

		found_plane = true;
	}

	return found_plane;
}


bool DirtDetection::computeBirdsEyePerspective(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, cv::Mat& plane_color_image, cv::Mat& plane_mask, pcl::ModelCoefficients& plane_model, cv::Mat& H, cv::Mat& R, cv::Mat& t, cv::Point2f& cameraImagePlaneOffset, cv::Mat& plane_color_image_warped, cv::Mat& plane_mask_warped)
{
	// 1. compute parameter representation of plane, construct plane coordinate system and compute transformation from camera frame (x,y,z) to plane frame (x,y,z)
	// a) parameter form of plane equation
	// choose two arbitrary points on the plane
	cv::Point3d p1, p2;
	double a=plane_model.values[0], b=plane_model.values[1], c=plane_model.values[2], d=plane_model.values[3];
	if (a==0. && b==0.)
	{
		p1.x = 0;
		p1.y = 0;
		p1.z = -d/c;
		p2.x = 1;
		p2.y = 0;
		p2.z = -d/c;
	}
	else if (a==0. && c==0.)
	{
		p1.x = 0;
		p1.y = -d/b;
		p1.z = 0;
		p2.x = 1;
		p2.y = -d/b;
		p2.z = 0;
	}
	else if (b==0. && c==0.)
	{
		p1.x = -d/a;
		p1.y = 0;
		p1.z = 0;
		p2.x = -d/a;
		p2.y = 1;
		p2.z = 0;
	}
	else if (a==0.)
	{
		p1.x = 0;
		p1.y = 0;
		p1.z = -d/c;
		p2.x = 1;
		p2.y = 0;
		p2.z = -d/c;
	}
	else if (b==0.)
	{
		p1.x = 0;
		p1.y = 0;
		p1.z = -d/c;
		p2.x = 0;
		p2.y = 1;
		p2.z = -d/c;
	}
	else if (c==0.)
	{
		p1.x = -d/a;
		p1.y = 0;
		p1.z = 0;
		p2.x = -d/a;
		p2.y = 0;
		p2.z = 1;
	}
	else
	{
		p1.x = 0;
		p1.y = 0;
		p1.z = -d/c;
		p2.x = 1;
		p2.y = 0;
		p2.z = (-d-a)/c;
	}
	// compute two normalized directions
	cv::Point3d dirS, dirT, normal(a,b,c);
	double lengthNormal = cv::norm(normal);
//	if (c<0.)
//		lengthNormal *= -1;
	normal.x /= lengthNormal;
	normal.y /= lengthNormal;
	normal.z /= lengthNormal;
	dirS = p2-p1;
	double lengthS = cv::norm(dirS);
	dirS.x /= lengthS;
	dirS.y /= lengthS;
	dirS.z /= lengthS;
	dirT.x = normal.y*dirS.z - normal.z*dirS.y;
	dirT.y = normal.z*dirS.x - normal.x*dirS.z;
	dirT.z = normal.x*dirS.y - normal.y*dirS.x;
	double lengthT = cv::norm(dirT);
	dirT.x /= lengthT;
	dirT.y /= lengthT;
	dirT.z /= lengthT;

	// b) construct plane coordinate system
	// plane coordinate frame has center p1 and x-axis=dirS, y-axis=dirT, z-axis=normal

	// c) compute transformation from camera frame (x,y,z) to plane frame (x,y,z)
	t = (cv::Mat_<double>(3,1) << p1.x, p1.y, p1.z);
	R = (cv::Mat_<double>(3,3) << dirS.x, dirT.x, normal.x, dirS.y, dirT.y, normal.y, dirS.z, dirT.z, normal.z);

//		std::cout << "t: " << p1.x << ", " << p1.y << ", " << p1.z << std::endl;
//		std::cout << "dirS: " << dirS.x << ", " << dirS.y << ", " << dirS.z << std::endl;
//		std::cout << "dirT: " << dirT.x << ", " << dirT.y << ", " << dirT.z << std::endl;
//		std::cout << "normal: " << normal.x << ", " << normal.y << ", " << normal.z << std::endl;

	// 2. select data segment and compute final transformation of camera coordinates to scaled and centered plane coordinates
	std::vector<cv::Point2f> pointsCamera, pointsPlane;
	cv::Point2f minPlane(1e20,1e20), maxPlane(-1e20,-1e20);
	cv::Mat RTt = R.t()*t;
	for (int v=0; v<plane_color_image.rows; v++)
	{
		for (int u=0; u<plane_color_image.cols; u++)
		{
			// black pixels are not part of the plane
			bgr color = plane_color_image.at<bgr>(v,u);
			if (color.r==0 && color.g==0 && color.b==0)
				continue;

			// distance to camera has to be below a maximum distance
			pcl::PointXYZRGB point = (*input_cloud)[v*plane_color_image.cols+u];
			if (point.x*point.x + point.y*point.y + point.z*point.z > maxDistanceToCamera_*maxDistanceToCamera_)
				continue;

			// determine max and min x and y coordinates of the plane
			cv::Mat pointCamera = (cv::Mat_<double>(3,1) << point.x, point.y, point.z);
			pointsCamera.push_back(cv::Point2f(u,v));
			cv::Mat pointPlane = R.t()*pointCamera - RTt;
			pointsPlane.push_back(cv::Point2f(pointPlane.at<double>(0),pointPlane.at<double>(1)));

			if (minPlane.x>pointPlane.at<double>(0))
				minPlane.x=pointPlane.at<double>(0);
			if (maxPlane.x<pointPlane.at<double>(0))
				maxPlane.x=pointPlane.at<double>(0);
			if (minPlane.y>pointPlane.at<double>(1))
				minPlane.y=pointPlane.at<double>(1);
			if (maxPlane.y<pointPlane.at<double>(1))
				maxPlane.y=pointPlane.at<double>(1);
		}
	}

	// 3. find homography between image plane and plane coordinates
	// a) collect point correspondences
	if (pointsCamera.size()<100)
		return false;
	double step = std::max(1.0, (double)pointsCamera.size()/100.0);
	std::vector<cv::Point2f> correspondencePointsCamera, correspondencePointsPlane;
	cameraImagePlaneOffset = cv::Point2f((maxPlane.x+minPlane.x)/2.f - (double)plane_color_image.cols/(2*birdEyeResolution_), (maxPlane.y+minPlane.y)/2.f - (double)plane_color_image.rows/(2*birdEyeResolution_));
	for (double i=0; i<(double)pointsCamera.size(); i+=step)
	{
		correspondencePointsCamera.push_back(pointsCamera[(int)i]);
		correspondencePointsPlane.push_back(birdEyeResolution_*(pointsPlane[(int)i]-cameraImagePlaneOffset));
	}
	// b) compute homography
	H = cv::findHomography(correspondencePointsCamera, correspondencePointsPlane);
//		correspondencePointsCamera.push_back(cv::Point2f(160,400));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		correspondencePointsCamera.push_back(cv::Point2f(320,400));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		correspondencePointsCamera.push_back(cv::Point2f(480,400));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		correspondencePointsCamera.push_back(cv::Point2f(160,200));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		correspondencePointsCamera.push_back(cv::Point2f(320,200));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		correspondencePointsCamera.push_back(cv::Point2f(480,200));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		for (int i=0; i<(int)correspondencePointsCamera.size(); i++)
//		{
//			cv::Mat pc = (cv::Mat_<double>(3,1) << (double)correspondencePointsCamera[i].x, (double)correspondencePointsCamera[i].y, 1.0);
//			cv::Mat pp = (cv::Mat_<double>(3,1) << (double)correspondencePointsPlane[i].x, (double)correspondencePointsPlane[i].y, 1.0);
//
//			cv::Mat Hp = H.inv()*pp;
//			Hp.at<double>(0) /= Hp.at<double>(2);
//			Hp.at<double>(1) /= Hp.at<double>(2);
//			Hp.at<double>(2) = 1.0;
//			cv::Mat r = pc - Hp;
//
//			std::cout << "H - " << i << ": " << r.at<double>(0) << ", " << r.at<double>(1) << ", " << r.at<double>(2) << std::endl;
//		}

	// 4. warp perspective
	cv::warpPerspective(plane_color_image, plane_color_image_warped, H, plane_color_image.size());
	cv::warpPerspective(plane_mask, plane_mask_warped, H, plane_mask.size());

//		// this example is correct, H transforms world points into the image coordinate system
//		std::vector<cv::Point2f> c1, c2;
//		c1.push_back(cv::Point2f(885,1362));
//		c2.push_back(cv::Point2f(0,0));
//		c1.push_back(cv::Point2f(880,1080));
//		c2.push_back(cv::Point2f(0, 142.7));
//		c1.push_back(cv::Point2f(945,1089));
//		c2.push_back(cv::Point2f(83.7,142.7));
//		c1.push_back(cv::Point2f(948,1350));
//		c2.push_back(cv::Point2f(83.7,0));
//		cv::Mat Hcc = cv::findHomography(c2, c1);
//		std::cout << "H: " << std::endl;
//		for (int v=0;v<3; v++)
//		{
//			for (int u=0; u<3; u++)
//				std::cout << Hcc.at<double>(v,u) << "\t";
//			std::cout << std::endl;
//		}
//		// add test points
//		c1.push_back(cv::Point2f(1010, 1338));
//		c2.push_back(cv::Point2f(167.4,0));
//		c1.push_back(cv::Point2f(1010, 1098));
//		c2.push_back(cv::Point2f(167.4,142.7));
//		for (int i=0; i<(int)c1.size(); i++)
//		{
//			cv::Mat pc = (cv::Mat_<double>(3,1) << (double)c1[i].x, (double)c1[i].y, 1.0);
//			cv::Mat pp = (cv::Mat_<double>(3,1) << (double)c2[i].x, (double)c2[i].y, 1.0);
//			cv::Mat Hccpp = Hcc*pp;
//			Hccpp.at<double>(0) /= Hccpp.at<double>(2);
//			Hccpp.at<double>(1) /= Hccpp.at<double>(2);
//			Hccpp.at<double>(2) = 1.0;
//			cv::Mat r = pc - Hccpp;
//			std::cout << "H - " << i << ": " << r.at<double>(0) << ", " << r.at<double>(1) << ", " << r.at<double>(2) << std::endl;
//		}

	return true;
}


void DirtDetection::transformPointFromCameraImageToWorld(const cv::Mat& pointCamera, const cv::Mat& H, const cv::Mat& R, const cv::Mat& t, const cv::Point2f& cameraImagePlaneOffset, const tf::StampedTransform& transformMapCamera, cv::Point3f& pointWorld)
{
	cv::Mat Hp = H*pointCamera;	// transformation from image plane to floor plane
	cv::Mat pointFloor = (cv::Mat_<double>(3,1) << Hp.at<double>(0)/Hp.at<double>(2)/birdEyeResolution_+cameraImagePlaneOffset.x, Hp.at<double>(1)/Hp.at<double>(2)/birdEyeResolution_+cameraImagePlaneOffset.y, 0.0);
	cv::Mat pointWorldCamera = R*pointFloor + t;	// transformation from floor plane to camera world coordinates
	btVector3 pointWorldCameraBt(pointWorldCamera.at<double>(0), pointWorldCamera.at<double>(1), pointWorldCamera.at<double>(2));
	btVector3 pointWorldMapBt = transformMapCamera * pointWorldCameraBt;
	pointWorld.x = pointWorldMapBt.getX();
	pointWorld.y = pointWorldMapBt.getY();
	pointWorld.z = pointWorldMapBt.getZ();
}

void DirtDetection::transformPointFromCameraWarpedToWorld(const cv::Mat& pointPlane, const cv::Mat& R, const cv::Mat& t, const cv::Point2f& cameraImagePlaneOffset, const tf::StampedTransform& transformMapCamera, cv::Point3f& pointWorld)
{
	btVector3 pointWorldMapBt;
	if (warpImage_ == true)
	{
		cv::Mat pointFloor = (cv::Mat_<double>(3,1) << pointPlane.at<double>(0)/birdEyeResolution_+cameraImagePlaneOffset.x, pointPlane.at<double>(1)/birdEyeResolution_+cameraImagePlaneOffset.y, 0.0);
		cv::Mat pointWorldCamera = R*pointFloor + t;	// transformation from floor plane to camera world coordinates
		btVector3 pointWorldCameraBt(pointWorldCamera.at<double>(0), pointWorldCamera.at<double>(1), pointWorldCamera.at<double>(2));
		pointWorldMapBt = transformMapCamera * pointWorldCameraBt;

	}
	else
	{
		btVector3 pointWorldCameraBt(pointPlane.at<double>(0), pointPlane.at<double>(1), pointPlane.at<double>(2));
		pointWorldMapBt = transformMapCamera * pointWorldCameraBt;
	}
	pointWorld.x = pointWorldMapBt.getX();
	pointWorld.y = pointWorldMapBt.getY();
	pointWorld.z = pointWorldMapBt.getZ();
}


void DirtDetection::putDetectionIntoGrid(cv::Mat& grid, const labelImage::RegionPointTriple& detection)
{
	//// convert three map points to RotatedRect, neglect z-coordinates
	//cv::Point3f lWidth = detection.p1-detection.center;
	//double width = 2.*sqrt(lWidth.x*lWidth.x + lWidth.y*lWidth.y + lWidth.z*lWidth.z);
	//cv::Point3f lHeight = detection.p2-detection.center;
	//double height = 2.*sqrt(lHeight.x*lHeight.x + lHeight.y*lHeight.y + lHeight.z*lHeight.z);
	//double alpha = -atan2(lWidth.y, lWidth.x);
	//cv::RotatedRect detectionRect(cv::Point2f(detection.center.x, detection.center.y), cv::Size(width, height), alpha);

	// sample all points along the principal axis and count dirt detection in each grid cell of the map once
	cv::Point3f lWidth = detection.p1-detection.center;
	cv::Point3f lHeight = detection.p2-detection.center;
	std::set<cv::Point2i, lessPoint2i> visitedGridCells;
	//std::cout << "lW: " << lWidth.x << ", " << lWidth.y << "  lH: " << lHeight.x << ", " << lHeight.y << std::endl;
	for (double scaleW=-1.; scaleW<=1.; scaleW+=0.2)
	{
		for (double scaleH=-1.; scaleH<=1.; scaleH+=0.2)
		{
			cv::Point2i offset(grid.cols/2, grid.rows/2);
			cv::Point2i coordinates[2];
			coordinates[0] = cv::Point2i(-(detection.center.x+scaleW*lWidth.x-gridOrigin_.x)*gridResolution_+offset.x, (detection.center.y+scaleW*lWidth.y-gridOrigin_.y)*gridResolution_+offset.y);
			coordinates[1] = cv::Point2i(-(detection.center.x+scaleH*lHeight.x-gridOrigin_.x)*gridResolution_+offset.x, (detection.center.y+scaleH*lHeight.y-gridOrigin_.y)*gridResolution_+offset.y);
			for (int i=0; i<2; i++)
			{
				for (int v=0; v<=0; v++)
				{
					for (int u=0; u<=0; u++)
					{
						cv::Point2i co(coordinates[i].x+u,coordinates[i].y+v);
						//cv::Point2i co(coordinates[i].x,coordinates[i].y);
						if (visitedGridCells.find(co)==visitedGridCells.end() && co.x>=0 && co.x<grid.cols && co.y>=0 && co.y<grid.rows)
						{
							//std::cout << "sW,sH: " << scaleW << ", " << scaleH << "  co: " << co.x << ", " << co.y << std::endl;
							// grid cell has not been incremented, yet
							visitedGridCells.insert(co);
							grid.at<int>(co) = grid.at<int>(co) + 1;
						}
					}
				}
			}
		}
	}
}


void DirtDetection::SaliencyDetection_C1(const cv::Mat& C1_image, cv::Mat& C1_saliency_image)
{
	//given a one channel image
	//int scale = 6;
	//unsigned int size = (int)floor((float)pow(2.0,scale)); //the size to do the saliency at
	unsigned int size_cols = (int)(C1_image.cols * spectralResidualImageSizeRatio_);
	unsigned int size_rows = (int)(C1_image.rows * spectralResidualImageSizeRatio_);

	//create different images
	cv::Mat bw_im;
	cv::resize(C1_image,bw_im,cv::Size(size_cols,size_rows));

	cv::Mat realInput(size_rows,size_cols, CV_32FC1);	//calculate number of test samples
	//int NumTestSamples = 10; //ceil(NumSamples*percentage_testdata);
	//printf("Anzahl zu ziehender test samples: %d \n", NumTestSamples);
	cv::Mat imaginaryInput(size_rows,size_cols, CV_32FC1);
	cv::Mat complexInput(size_rows,size_cols, CV_32FC2);

	bw_im.convertTo(realInput,CV_32F,1.0/255,0);
	imaginaryInput = cv::Mat::zeros(size_rows,size_cols,CV_32F);

	std::vector<cv::Mat> vec;
	vec.push_back(realInput);
	vec.push_back(imaginaryInput);
	cv::merge(vec, complexInput);

	cv::Mat dft_A(size_rows,size_cols,CV_32FC2);

	cv::dft(complexInput, dft_A, cv::DFT_COMPLEX_OUTPUT,size_rows);
	vec.clear();
	cv::split(dft_A,vec);
	realInput = vec[0];
	imaginaryInput = vec[1];

	// Compute the phase angle
	cv::Mat image_Mag(size_rows,size_cols, CV_32FC1);
	cv::Mat image_Phase(size_rows,size_cols, CV_32FC1);

	//compute the phase of the spectrum
	cv::cartToPolar(realInput, imaginaryInput, image_Mag, image_Phase,0);
	std::vector<DirtDetection::CarpetFeatures> test_feat_vec;
	std::vector<DirtDetection::CarpetClass> test_class_vec;


	std::string name;
	std::vector<std::string> name_vec;

	name = "carpet-gray.tepp";
	name_vec.push_back(name);
	cv::Mat log_mag(size_rows,size_cols, CV_32FC1);
	cv::log(image_Mag, log_mag);

//	cv::Mat log_mag_;
//	cv::normalize(log_mag, log_mag_, 0, 1, NORM_MINMAX);
//	cv::imshow("log_mag", log_mag_);

	//Box filter the magnitude, then take the difference
	cv::Mat log_mag_Filt(size_rows,size_cols, CV_32FC1);

	cv::Mat filt = cv::Mat::ones(3, 3, CV_32FC1) * 1./9.;
	//filt.convertTo(filt,-1,1.0/9.0,0);

	cv::filter2D(log_mag, log_mag_Filt, -1, filt);
	//cv::GaussianBlur(log_mag, log_mag_Filt, cv::Size2i(25,25), 0);
	//log_mag = log_mag_Filt.clone();
	//cv::medianBlur(log_mag, log_mag_Filt, 5);

//	cv::Mat log_mag_filt_;
//	cv::normalize(log_mag_Filt, log_mag_filt_, 0, 1, NORM_MINMAX);
//	cv::imshow("log_mag_filt", log_mag_filt_);

	//cv::subtract(log_mag, log_mag_Filt, log_mag);
	log_mag -= log_mag_Filt;

//	cv::Mat log_mag_sub_a_, log_mag_sub_;
//	cv::normalize(log_mag, log_mag_sub_a_, 0, 1, NORM_MINMAX);
//	cv::GaussianBlur(log_mag_sub_a_, log_mag_sub_, cv::Size2i(21,21), 0);
//	cv::imshow("log_mag_sub", log_mag_sub_);
//	log_mag_Filt = log_mag.clone();
//	cv::GaussianBlur(log_mag_Filt, log_mag, cv::Size2i(21,21), 0);
	void GBTreeEvaluation(	std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
						std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec,
						CvGBTrees &carpet_GBTree);
	cv::exp(log_mag, image_Mag);

	cv::polarToCart(image_Mag, image_Phase, realInput, imaginaryInput,0);

	vec.clear();
	vec.push_back(realInput);
	vec.push_back(imaginaryInput);
	cv::merge(vec, dft_A);

	cv::dft(dft_A, dft_A, cv::DFT_INVERSE,size_rows);

	dft_A = abs(dft_A);
	dft_A.mul(dft_A);

	cv::split(dft_A, vec);

	C1_saliency_image = vec[0];
}	std::vector<DirtDetection::CarpetFeatures> test_feat_vec;


void DirtDetection::SaliencyDetection_C3(const cv::Mat& C3_color_image, cv::Mat& C1_saliency_image, const cv::Mat* mask, int gaussianBlurCycles)
{
	cv::Mat fci; // "fci"<-> first channel image
	cv::Mat sci; // "sci"<-> second channel image
	cv::Mat tci; //"tci"<-> second channel image

	std::vector<cv::Mat> vec;
	vec.push_back(fci);
	vec.push_back(sci);
	vec.push_back(tci);

	cv::split(C3_color_image,vec);

	fci = vec[0];
	sci = vec[1];
	tci = vec[2];

	cv::Mat res_fci; // "fci"<-> first channel image
	cv::Mat res_sci; // "sci"<-> second channel image
	cv::Mat res_tci; //"tci"<-> second channel image

	SaliencyDetection_C1(fci, res_fci);
	SaliencyDetection_C1(sci, res_sci);
	SaliencyDetection_C1(tci, res_tci);

	cv::Mat realInput;

	realInput = (res_fci + res_sci + res_tci)/3;

	cv::Size2i ksize;
	ksize.width = 3;
	ksize.height = 3;
	for (int i=0; i<gaussianBlurCycles; i++)
		cv::GaussianBlur(realInput, realInput, ksize, 0); //necessary!? --> less noise


	cv::resize(realInput,C1_saliency_image,C3_color_image.size());

	// remove borders of the ground plane because of artifacts at the border like lines
	if (mask != 0)
	{
		// maske erodiere
		cv::Mat mask_eroded = mask->clone();
		cv::dilate(*mask, mask_eroded, cv::Mat(), cv::Point(-1, -1), 2);
		//cv::erode(mask_eroded, mask_eroded, cv::Mat(), cv::Point(-1, -1), 25.0/640.0*C3_color_image.cols);
		cv::erode(mask_eroded, mask_eroded, cv::Mat(), cv::Point(-1, -1), 35.0/640.0*C3_color_image.cols);
		cv::Mat temp;
		C1_saliency_image.copyTo(temp, mask_eroded);
		C1_saliency_image = temp;
	}

	// remove saliency at the image border (because of artifacts in the corners)
	int borderX = C1_saliency_image.cols/20;
	int borderY = C1_saliency_image.rows/20;
	if (borderX > 0 && borderY > 0)
	{
		cv::Mat smallImage_ = C1_saliency_image.colRange(borderX, C1_saliency_image.cols-borderX);
		cv::Mat smallImage = smallImage_.rowRange(borderY, smallImage_.rows-borderY);

		copyMakeBorder(smallImage, C1_saliency_image, borderY, borderY, borderX, borderX, cv::BORDER_CONSTANT, cv::Scalar(0));
	}


	// display the individual channels
//	for (int i=0; i<gaussianBlurCycles; i++)
//		cv::GaussianBlur(res_fci, res_fci, ksize, 0); //necessary!? --> less noise
//	for (int i=0; i<gaussianBlurCycles; i++)
//		cv::GaussianBlur(res_sci, res_sci, ksize, 0); //necessary!? --> less noise	//calculate number of test samples
//	int NumTestSamples = 10; //ceil(NumSamples*percentage_testdata);
//	printf("Anzahl zu ziehender test samples: %d \n", NumTestSamples);
//	for (int i=0; i<gaussianBlurCycles; i++)
//		cv::GaussianBlur(res_tci, res_tci, ksize, 0); //necessary!? --> less noise
//
//	cv::resize(res_fci,res_fci,C3_color_image.size());
//	cv::resize(res_sci,res_sci,C3_color_image.size());
//	cv::resize(res_tci,res_tci,C3_color_image.size());
//
//	// scale input_image
//	double minv, maxv;
//	cv::Point2i minl, maxl;
//	cv::minMaxLoc(res_fci,&minv,&maxv,&minl,&maxl);
//	res_fci.convertTo(res_fci, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
//	cv::minMaxLoc(res_sci,&minv,&maxv,&minl,&maxl);
//	res_sci.convertTo(res_sci, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
//	cv::minMaxLoc(res_tci,&minv,&maxv,&minl,&maxl);
//	res_tci.convertTo(res_tci, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
//
//	cv::imshow("b", res_fci);
//	cv::imshow("g", res_sci);
//	cv::imshow("r", res_tci);
}


void DirtDetection::Image_Postprocessing_C1(const cv::Mat& C1_saliency_image, cv::Mat& C1_BlackWhite_image, cv::Mat& C3_color_image)
{
	// scale input_image
	cv::Mat scaled_input_image;
	double minv, maxv;
	cv::Point2i minl, maxl;
	cv::minMaxLoc(C1_saliency_image,&minv,&maxv,&minl,&maxl);
	C1_saliency_image.convertTo(scaled_input_image, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));

	if (debug_["showSaliencyDetection"] == true)
	{
		cv::imshow("saliency detection", scaled_input_image);
		cvMoveWindow("saliency detection", 650, 530);
	}

	//set dirt pixel to white
	C1_BlackWhite_image = cv::Mat::zeros(C1_saliency_image.size(), CV_8UC1);
	cv::threshold(scaled_input_image, C1_BlackWhite_image, dirtThreshold_, 1, cv::THRESH_BINARY);

//	std::cout << "(C1_saliency_image channels) = (" << C1_saliency_image.channels() << ")" << std::endl;

	cv::Mat CV_8UC_image;
	C1_BlackWhite_image.convertTo(CV_8UC_image, CV_8UC1);


//	Mat dst = Mat::zeros(img.rows, img.cols, CV_8UC3);
//	dst = C3_color_image;

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    cv::findContours(CV_8UC_image, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    Scalar color(0, 255, 0);
    cv::RotatedRect rec;

    for (int i = 0; i < (int)contours.size(); i++)
    {
		rec = minAreaRect(contours[i]);
    	cv::ellipse(C3_color_image, rec, color, 2);
    }	//calculate number of test samples
	int NumTestSamples = 10; //ceil(NumSamples*percentage_testdata);
	printf("Anzahl zu ziehender test samples: %d \n", NumTestSamples);

}

void  DirtDetection::ReadDataFromCarpetFile(std::vector<CarpetFeatures>& carp_feat_vec, std::vector<CarpetClass>& carp_class_vec, std::string filepath, std::string filename)
{
	CarpetFeatures features;
	CarpetClass cc;

	std::ifstream indata;
	float data; // variable for input value

	std::string svmpath = filepath + filename;
	indata.open(svmpath.c_str()); // opens the file
	if(!indata) { // file couldn't be opened
	  cerr << "Error: file could not be opened" << endl;
	}

	int hnum = 0;
	int count = 1;
	indata >>data;
	while ( !indata.eof() ) { // keep reading until end-of-file
		switch ( count )
		{
			case 1:
			{
				cc.dirtThreshold = data;
				count++;
			  break;
			}
			case 2:
			{
				features.min = data;
				count++;
			  break;
			}
			case 3:
			{
				features.max = data;
				count++;
			  break;
			}
			case 4:
			{
				features.mean = data;
				count++;
			  break;
			}
			case 5:
			{
				features.stdDev = data;
				count = 1;

				carp_feat_vec.push_back(features);
				carp_class_vec.push_back(cc);

				hnum++;

//				std::cout << "Original Class=" << cc.dirtThreshold << "\tmean= " << features.mean
//						  << "\tstd= " << features.stdDev << std::endl;

			  break;
			}
		} //switch
	  indata >> data; // sets EOF flag if no value found
	} //while

	indata.close();

}


void DirtDetection::ScaleSamples(std::vector<CarpetFeatures>& feat_vec, double & maxMean, double & maxStd)
{
	//determine number of samples
	int NumSamples = feat_vec.size();

	maxMean = feat_vec[0].mean;
	maxStd = feat_vec[0].stdDev;

	for (int i = 1; i<NumSamples; i++)
	{
		if (feat_vec[i].mean > maxMean)
		{
			maxMean = feat_vec[i].mean;
		}//if

		if (feat_vec[i].stdDev > maxStd)
		{
			maxStd = feat_vec[i].stdDev;
		}//if

	} //for-i

	maxMean = maxMean + 30;
	maxStd = maxStd + 30;

	for (int i = 1; i<NumSamples; i++)
	{
		feat_vec[i].mean = feat_vec[i].mean/maxMean;
		feat_vec[i].stdDev = feat_vec[i].stdDev/maxStd;

//		std::cout << "New mean value=" << feat_vec[i].mean << std::endl;
//		std::cout << "New std value=" << feat_vec[i].stdDev << std::endl;

	} //for-i

}


void DirtDetection::SplitIntoTrainAndTestSamples(	int NumTestSamples,
													std::vector<CarpetFeatures>& input_feat_vec, std::vector<CarpetClass>& input_class_vec,
													std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
													std::vector<CarpetFeatures>& test_feat_vec,  std::vector<CarpetClass>& test_class_vec)
{

	//determine number of samples
	int NumSamples = input_feat_vec.size();

	//initialize random seed
	srand ( time(NULL) );

	//helps to save numbers of test samples
	int SampledNumbers [NumTestSamples];
	for (int i = 0; i<NumTestSamples; i++)
	{
		SampledNumbers[i] = -1;
	}

	//loop variables
	int j = 0;
	bool foundflag = 0;

	for (int i = 0; i<NumTestSamples; i++)
	{

		//get valid sample number
		do
		{
			//generate random number in the range 0 to NumSamples-1
			SampledNumbers[i] =   rand() % NumSamples;

			//check if sample already used as test sample
			foundflag = 0;
			j = 0;
			while ( (foundflag == 0) && (j < i) )
			{

				if (SampledNumbers[i] == SampledNumbers[j])
				{
					foundflag = 1;
				} //if-end

				j++;

			} //while-do-end

		} while(foundflag == 1);

		//add test sample
		test_feat_vec.push_back(input_feat_vec[SampledNumbers[i]]);
		test_class_vec.push_back(input_class_vec[SampledNumbers[i]]);
	}

	for (int i = 0; i<NumSamples; i++ )
	{
		//check if sample already used as test sample
		foundflag = 0;
		j = 0;
		while ( (foundflag==0) && (j<NumTestSamples) )
		{
			if (i == SampledNumbers[j])
			{
				foundflag = 1;
			} //if-end

			j++;
		}

		//if sample not used as test sample, then use it as train sample
		if (foundflag == 0)
		{
			//add train sample
			train_feat_vec.push_back(input_feat_vec[i]);
			train_class_vec.push_back(input_class_vec[i]);
		}

	}

}

void DirtDetection::Image_Postprocessing_C1_rmb(const cv::Mat& C1_saliency_image, cv::Mat& C1_BlackWhite_image, cv::Mat& C3_color_image, std::vector<cv::RotatedRect>& dirtDetections, const cv::Mat& mask)
{
	// dirt detection on image with artificial dirt
	cv::Mat color_image_with_artifical_dirt = C3_color_image.clone();
	cv::Mat mask_with_artificial_dirt = mask.clone();
	// add dirt
	int dirtSize = cvRound(3.0/640.0 * C3_color_image.cols);
	cv::Point2f ul(0.4375*C3_color_image.cols, 0.416666667*C3_color_image.rows);
	cv::Point2f ur(0.5625*C3_color_image.cols, 0.416666667*C3_color_image.rows);
	cv::Point2f ll(0.4375*C3_color_image.cols, 0.583333333*C3_color_image.rows);
	cv::Point2f lr(0.5625*C3_color_image.cols, 0.583333333*C3_color_image.rows);
	cv::ellipse(color_image_with_artifical_dirt, cv::RotatedRect(ul, cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::ellipse(mask_with_artificial_dirt, cv::RotatedRect(ul, cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::ellipse(color_image_with_artifical_dirt, cv::RotatedRect(lr, cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::ellipse(mask_with_artificial_dirt, cv::RotatedRect(lr, cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::ellipse(color_image_with_artifical_dirt, cv::RotatedRect(ll, cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(0, 0, 0), dirtSize);
	cv::ellipse(mask_with_artificial_dirt, cv::RotatedRect(ll, cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::ellipse(color_image_with_artifical_dirt, cv::RotatedRect(ur, cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(0, 0, 0), dirtSize);
	cv::ellipse(mask_with_artificial_dirt, cv::RotatedRect(ur, cv::Size2f(dirtSize,dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::Mat C1_saliency_image_with_artifical_dirt;
	SaliencyDetection_C3(color_image_with_artifical_dirt, C1_saliency_image_with_artifical_dirt, &mask_with_artificial_dirt, spectralResidualGaussianBlurIterations_);
	//cv::imshow("ai_dirt", color_image_with_artifical_dirt);

	// display of images with artificial dirt
	if (debug_["showColorWithArtificialDirt"] == true)
		cv::imshow("color with artificial dirt", color_image_with_artifical_dirt);
	cv::Mat C1_saliency_image_with_artifical_dirt_scaled;
	double salminv, salmaxv;
	cv::Point2i salminl, salmaxl;
	cv::minMaxLoc(C1_saliency_image_with_artifical_dirt,&salminv,&salmaxv,&salminl,&salmaxl, mask_with_artificial_dirt);
	C1_saliency_image_with_artifical_dirt.convertTo(C1_saliency_image_with_artifical_dirt_scaled, -1, 1.0/(salmaxv-salminv), -1.0*(salminv)/(salmaxv-salminv));
	if (debug_["showSaliencyWithArtificialDirt"] == true)
		cv::imshow("saliency with artificial dirt", C1_saliency_image_with_artifical_dirt_scaled);

	// scale C1_saliency_image to value obtained from C1_saliency_image with artificially added dirt
//	std::cout << "res_img: " << C1_saliency_image_with_artifical_dirt.at<float>(300,200);
//	C1_saliency_image_with_artifical_dirt = C1_saliency_image_with_artifical_dirt.mul(C1_saliency_image_with_artifical_dirt);	// square C1_saliency_image_with_artifical_dirt to emphasize the dirt and increase the gap to background response
//	std::cout << " -> " << C1_saliency_image_with_artifical_dirt.at<float>(300,200) << std::endl;
	double minv, maxv;
	cv::Point2i minl, maxl;
	cv::minMaxLoc(C1_saliency_image_with_artifical_dirt,&minv,&maxv,&minl,&maxl, mask_with_artificial_dirt);
	cv::Scalar mean, stdDev;
	cv::meanStdDev(C1_saliency_image_with_artifical_dirt, mean, stdDev, mask);
	double newMaxVal = min(1.0, maxv/spectralResidualNormalizationHighestMaxValue_);///mean.val[0] / spectralResidualNormalizationHighestMaxMeanRatio_);
//	std::cout << "dirtThreshold=" << dirtThreshold_ << "\tmin=" << minv << "\tmax=" << maxv << "\tmean=" << mean.val[0] << "\tstddev=" << stdDev.val[0] << "\tnewMaxVal (r)=" << newMaxVal << std::endl;


	//determine ros package path
	// todo: learning part
//	std::string svmpath = ros::package::getPath("autopnp_dirt_detection") + "/common/files/svm/Teppich1.tepp";
//	ofstream teppichfile;
//	teppichfile.open (svmpath.c_str(), ios::out| ios::app);
//	teppichfile << dirtThreshold_ << "\t\t" << minv << "\t\t" << maxv << "\t\t" << mean.val[0] << "\t\t" << stdDev.val[0] << "\n";
//	teppichfile.close();


	////C1_saliency_image.convertTo(scaled_input_image, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
	cv::Mat scaled_C1_saliency_image = C1_saliency_image.clone();	// square C1_saliency_image_with_artifical_dirt to emphasize the dirt and increase the gap to background response
	//scaled_C1_saliency_image = scaled_C1_saliency_image.mul(scaled_C1_saliency_image);
	scaled_C1_saliency_image.convertTo(scaled_C1_saliency_image, -1, newMaxVal/(maxv-minv), -newMaxVal*(minv)/(maxv-minv));

	double newMean = mean.val[0] * newMaxVal/(maxv-minv) - newMaxVal*(minv)/(maxv-minv);
	double newStdDev = stdDev.val[0] * newMaxVal/(maxv-minv);
//	std::cout << "newMean=" << newMean << "   newStdDev=" << newStdDev << std::endl;

//	// scale C1_saliency_image
//	cv::Mat scaled_C1_saliency_image;
//	double minv, maxv;
//	cv::Point2i minl, maxl;
//	cv::minMaxLoc(C1_saliency_image,&minv,&maxv,&minl,&maxl, mask);
	cv::Mat badscale;
	double badminv, badmaxv;
	cv::Point2i badminl, badmaxl;
	cv::minMaxLoc(C1_saliency_image,&badminv,&badmaxv,&badminl,&badmaxl, mask);
	C1_saliency_image.convertTo(badscale, -1, 1.0/(badmaxv-badminv), -1.0*(badminv)/(badmaxv-badminv));
//	std::cout << "bad scale:   " << "\tmin=" << badminv << "\tmax=" << badmaxv << std::endl;
	if (debug_["showSaliencyBadScale"] == true)
	{
		cv::imshow("bad scale", badscale);
		cvMoveWindow("bad scale", 650, 0);
	}
	//cvMoveWindow("bad scale", 650, 520);
//	cv::Scalar mean, stdDev;
//	cv::meanStdDev(C1_saliency_image, mean, stdDev, mask);
//	std::cout << "min=" << minv << "\tmax=" << maxv << "\tmean=" << mean.val[0] << "\tstddev=" << stdDev.val[0] << std::endl;
//
//	double newMaxVal = min(1.0, maxv/mean.val[0] /5.);
//	//C1_saliency_image.convertTo(scaled_C1_saliency_image, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
//	C1_saliency_image.convertTo(scaled_C1_saliency_image, -1, newMaxVal/(maxv-minv), -newMaxVal*(minv)/(maxv-minv));

	// remove responses that lie on lines
	if (removeLines_ == true)
	{
		cv::Mat src, dst, color_dst;

		cv::cvtColor(C3_color_image, src, CV_BGR2GRAY);

		cv::Canny(src, dst, 150, 200, 3);
		cv::cvtColor(dst, color_dst, CV_GRAY2BGR);

		std::vector<cv::Vec4i> lines;
		cv::HoughLinesP(dst, lines, 1, CV_PI/180, 80, 30, 10);
		for( size_t i = 0; i < lines.size(); i++ )
		{
			line(color_dst, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 3, 8);
			line(scaled_C1_saliency_image, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,0), 13, 8);
		}

		if (debug_["showDetectedLines"] == true)
		{
			cv::namedWindow("Detected Lines", 1);
			cv::imshow("Detected Lines", color_dst);
		}
	}

	if (debug_["showSaliencyDetection"] == true)
	{
		cv::imshow("saliency detection", scaled_C1_saliency_image);
		cvMoveWindow("saliency detection", 650, 530);
	}

	//set dirt pixel to white
	C1_BlackWhite_image = cv::Mat::zeros(C1_saliency_image.size(), CV_8UC1);
	cv::threshold(scaled_C1_saliency_image, C1_BlackWhite_image, dirtThreshold_, 1, cv::THRESH_BINARY);
//	cv::threshold(scaled_C1_saliency_image, C1_BlackWhite_image, mean.val[0] + stdDev.val[0] * dirtCheckStdDevFactor_, 1, cv::THRESH_BINARY);

//	std::cout << "(C1_saliency_image channels) = (" << C1_saliency_image.channels() << ")" << std::endl;

	cv::Mat CV_8UC_image;
	C1_BlackWhite_image.convertTo(CV_8UC_image, CV_8UC1);


//	Mat dst = Mat::zeros(img.rows, img.cols, CV_8UC3);
//	dst = C3_color_image;

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    cv::findContours(CV_8UC_image, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    cv::Scalar green(0, 255, 0);
    cv::Scalar red(0, 0, 255);
    for (int i = 0; i < (int)contours.size(); i++)
    {
    	cv::RotatedRect rec = minAreaRect(contours[i]);
    	double meanIntensity = 0;
    	for (int t=0; t<(int)contours[i].size(); t++)
    		meanIntensity += scaled_C1_saliency_image.at<float>(contours[i][t].y, contours[i][t].x);
    	meanIntensity /= (double)contours[i].size();
    	if (meanIntensity > newMean + dirtCheckStdDevFactor_ * newStdDev)
			cv::ellipse(C3_color_image, rec, green, 2);
    	else
    		cv::ellipse(C3_color_image, rec, red, 2);
    	dirtDetections.push_back(rec);
	}
}



//void DirtDetection::SaliencyDetection_C1_old_cv_code(const sensor_msgs::ImageConstPtr& color_image_msg)
//{
//	cv_bridge::CvImageConstPtr color_image_ptr;
//	cv::Mat color_image;
//	convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);
//
//	// color_image is now available with the current image from your camera
//
//	cv::imshow("image", color_image);
//	//cv::waitKey(10);
//
//	//int thresh = 350;
//	int scale = 6;
//
//	//convert color image to one channel gray-scale image
//	//color_image -> gray_image
//    IplImage hsrc = color_image;
//    IplImage *src = &hsrc;
//
//    /* get image properties */
//    int width  = src->width;
//    int height = src->height;
//
//    /* create new image for the grayscale version */
//    IplImage * gray_image = cvCreateImage( cvSize( width, height ), IPL_DEPTH_8U, 1 );
//
//    /* CV_RGB2GRAY: convert RGB image to grayscale */
//    cvCvtColor( src, gray_image, CV_RGB2GRAY );
//
//
//    cv::Mat bild(gray_image);
//
//	cv::imshow("image3", bild);
//	//cv::waitKey(10);
//
//
//	//IplImage imageIpl = (IplImage) gray_image;
//	IplImage* image = gray_image; //& imageIpl;
//
//	//given a one channel image
//	unsigned int size = (int)floor((float)pow(2.0,scale)); //the size to do the saliency at
//
//	IplImage* bw_im = cvCreateImage(cvSize(size,size), IPL_DEPTH_8U,1);
//	cvResize(image, bw_im);
//	IplImage* realInput = cvCreateImage( cvGetSize(bw_im), IPL_DEPTH_32F, 1);
//	IplImage* imaginaryInput = cvCreateImage( cvGetSize(bw_im), IPL_DEPTH_32F, 1);
//	IplImage* complexInput = cvCreateImage( cvGetSize(bw_im), IPL_DEPTH_32F, 2);
//
//	cvScale(bw_im, realInput, 1.0/255.0);
//	cvZero(imaginaryInput);
//	cvMerge(realInput, imaginaryInput, NULL, NULL, complexInput);
//	CvMat* dft_A = cvCreateMat( size, size, CV_32FC2 );
//
//	// copy A to dft_A and pad dft_A with zeros
//	CvMat tmp;
//	cvGetSubRect( dft_A, &tmp, cvRect(0,0, size,size));
//	cvCopy( complexInput, &tmp );
//	// cvZero(&tmp);
//
//	cvDFT( dft_A, dft_A, CV_DXT_FORWARD, size );
//	cvSplit( dft_A, realInput, imaginaryInput, NULL, NULL );
//	// Compute the phase angle
//	IplImage* image_Mag = cvCreateImage(cvSize(size, size), IPL_DEPTH_32F, 1);
//	IplImage* image_Phase = cvCreateImage(cvSize(size, size), IPL_DEPTH_32F, 1);
//
//
//	//compute the phase of the spectrum
//	cvCartToPolar(realInput, imaginaryInput, image_Mag, image_Phase, 0);
//
//	IplImage* log_mag = cvCreateImage(cvSize(size, size), IPL_DEPTH_32F, 1);
//	cvLog(image_Mag, log_mag);
//
//	//Box filter the magnitude, then take the difference
//	IplImage* log_mag_Filt = cvCreateImage(cvSize(size, size), IPL_DEPTH_32F, 1);
//	CvMat* filt = cvCreateMat(3,3, CV_32FC1);
//	cvSet(filt,cvScalarAll(1.0/9.0));
//	cvFilter2D(log_mag, log_mag_Filt, filt);
//	cvReleaseMat(&filt);
//
//	cvSub(log_mag, log_mag_Filt, log_mag);
//
//	cvExp(log_mag, image_Mag);
//
//	cvPolarToCart(image_Mag, image_Phase, realInput, imaginaryInput,0);
//	//cvExp(log_mag, image_Mag);
//
//	cvMerge(realInput, imaginaryInput, NULL, NULL, dft_A);
//	cvDFT( dft_A, dft_A, CV_DXT_INV_SCALE, size);
//
//	cvAbs(dft_A, dft_A);
//	cvMul(dft_A,dft_A, dft_A);
//	cvGetSubRect( dft_A, &tmp, cvRect(0,0, size,size));
//	cvCopy( &tmp, complexInput);
//	cvSplit(complexInput, realInput, imaginaryInput, NULL,NULL);
//
//	IplImage* result_image = cvCreateImage(cvGetSize(image),IPL_DEPTH_32F, 1);
//	double minv, maxv;
//	CvPoint minl, maxl;
//	cvSmooth(realInput,realInput);
//	cvSmooth(realInput,realInput);
//	cvMinMaxLoc(realInput,&minv,&maxv,&minl,&maxl);
//	//printf("Max value %lf, min %lf\n", maxv,minv);
//	maxv= 0.03;
//	cvScale(realInput, realInput, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
//	cvResize(realInput, result_image);
//
//
//	cv::Mat resultImage(result_image);
//
//
//	// remove saliency at the image border
//	int borderX = resultImage.cols/20;
//	int borderY = resultImage.rows/20;
//	if (borderX > 0 && borderY > 0)
//	{
//	cv::Mat smallImage_ = resultImage.colRange(borderX, resultImage.cols-1-borderX);
//	cv::Mat smallImage = smallImage_.rowRange(borderY, smallImage_.rows-1-borderY);
//
//	copyMakeBorder(smallImage, resultImage, borderY, borderY, borderX, borderX, cv::BORDER_CONSTANT, cv::Scalar(0));
//	}
//
//	//CvMat -> cv::Mat
//	//cv::meanStdDev();
//
//	//cv::Mat stainImage;
//	// check if the image makes sense (or convert to 0...255)
//	//resultImage.convertTo(stainImage, -1, 1.0, 0.0);
//
//
//	//mThresholdHou = thresh/100.0*cvAvg(realInput).val[0];
//
//	cv::imshow("image2", resultImage);
//	cv::waitKey(10);
//
//
//	//stainImage.release();
//
//	resultImage.release();
//	cvReleaseImage(&result_image);
//	cvReleaseImage(&realInput);
//	cvReleaseImage(&imaginaryInput);
//	cvReleaseImage(&complexInput);
//	cvReleaseMat(&dft_A);
//	cvReleaseImage(&bw_im);
//
//	cvReleaseImage(&image_Mag);
//	cvReleaseImage(&image_Phase);
//
//	cvReleaseImage(&log_mag);
//	cvReleaseImage(&log_mag_Filt);
//	cvReleaseImage(&bw_im);
//
//}

void DirtDetection::CreateCarpetClassiefierSVM(const std::vector<CarpetFeatures>& carp_feat_vec,
											const std::vector<CarpetClass>& carp_class_vec,
											CvSVM &carpet_SVM)
{

	/////////////////////////////////////////////////////////////////////////////
	///Old style code-> necessary because svm does not work with c++ style!!!///
	///////////////////////////////////////////////////////////////////////////

	//determine number of samples
	int Nges = carp_feat_vec.size();

	//number of features
	int NCarpetFeatures = 2;

	int i;
	float *data = 0;
	data = new float [Nges*2];

	float *res = 0;
	res = new float [Nges];

	CvMat data_mat, res_mat;

	for (i = 0; i < Nges; i++)
	{
	  data[i * 2] = float(carp_feat_vec[i].mean);
	  data[i * 2 + 1] = float(carp_feat_vec[i].stdDev);
	  res[i] = carp_class_vec[i].dirtThreshold;
//	  std::cout << 	"threshold= " << res[i] << std::endl;
	}

	cvInitMatHeader (&data_mat, Nges, NCarpetFeatures, CV_32FC1, data);
	cvInitMatHeader (&res_mat, Nges, 1, CV_32FC1, res);

//    // Set up SVM's parameters
    CvSVMParams params;
    params.svm_type    = CvSVM::NU_SVR;
    params.kernel_type = CvSVM::RBF;//POLY;

    params.degree = 1.0;
    params.gamma = 1.0;
    params.coef0 = 1.0;
    params.C = 3;
    params.nu = 0.99;
    params.p = 1;
    params.class_weights = NULL;
    params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
    params.term_crit = cvTermCriteria (CV_TERMCRIT_EPS, 100, FLT_EPSILON);

	//define testing area for "train_auto()":
    cv::ParamGrid gamma_grid(0.1, 1, 2);
    cv::ParamGrid C_grid(1, 10, 2 );
    cv::ParamGrid nu_grid(0.01, 0.2, 2 );
    cv::ParamGrid degree_grid(0.1, 1, 2);

	//determine optimal parameters and train SVM
	//Comment: It's important to define all input parameters because otherwise "SVM.train_auto" might not work properly!!!
	carpet_SVM.train_auto(	&data_mat, &res_mat, NULL, NULL, params, 10,
							CvSVM::get_default_grid(CvSVM::C), //C_grid,
							CvSVM::get_default_grid(CvSVM::GAMMA), //gamma_grid,
							CvSVM::get_default_grid(CvSVM::P),
							CvSVM::get_default_grid(CvSVM::NU), //nu_grid,
							CvSVM::get_default_grid(CvSVM::COEF),
							CvSVM::get_default_grid(CvSVM::DEGREE),
							false );

	//get SVM parameter
	params = carpet_SVM.get_params();

	//display SVM parameters on screen
	std::cout << 	"C = " << params.C << "\ncoeff0 = " << params.coef0 << "\ndegree = " <<
					params.degree << "\ngamma = " << params.gamma << "\nnu = " << params.nu <<
					"\np = " << params.p << std::endl;

	//freigabe
	delete[] data;
	delete[] res;

}

void DirtDetection::SVMEvaluation(	std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
									std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec,
									CvSVM &carpet_SVM, double ScaleMean, double ScaleStd)
{

	//number of test samples
	int NumTestSamples = test_feat_vec.size();
	int NumTrainSamples = train_feat_vec.size();

	//saves the different class types
	std::vector<CarpetClass> ClassTypes_vec;

	//number of different class types
	int NumClass = 0;

	//max mean value
	double maxmean = -1;
	//max std. dev.
	double maxstd = -1;

	//loop variables
	bool foundflag = 0;
	int j = 0;

	bool imageOnFlag = 1;

	for(int i = 0; i< NumTrainSamples; i++)
	{
		//test if mean or stdDev bigger than saved values
		if (train_feat_vec[i].mean > maxmean ){maxmean = train_feat_vec[i].mean;}
		if (train_feat_vec[i].stdDev > maxstd ){maxstd = train_feat_vec[i].stdDev;}

		//Determine if new class type
		foundflag = 0;
		j = 0;
		while ((foundflag == 0) && (j<NumClass))
		{
			if (ClassTypes_vec[j].dirtThreshold==train_class_vec[i].dirtThreshold)
			{
				foundflag = 1;
			}
			j++;
		}

		//if new class type, then save it in the list
		if (foundflag == 0)
		{
			ClassTypes_vec.push_back(train_class_vec[i]);
			NumClass++;
		}

	}

	//saves the total number and the number of correct classified samples of each class
	std::vector<NumStruc> num_vec;

	//initialise num_vec with void values
	NumStruc num_struc;
	num_struc.totalnum = 0;
	num_struc.correctnum = 0;

	for(int i = 0; i<NumClass; i++)
	{
		num_vec.push_back(num_struc);
	}


	int x,y;

	//save the prediction of the SVM
	//"predicted current class"
	float pcc;

	//Image for visual representation
	int width = 700, height = 700;

	cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
	cv::Mat image2 = cv::Mat::zeros(height, width, CV_8UC3);

	double kx1 = width/(1.2*maxmean);
	double kx2 = height/(1.2*maxstd);

	Vec3b green(0,255,0), blue (255,0,0), red (0,0,255);

//	cv::Mat sampleMat = cv::Mat::zeros(1, 2, CV_32FC1);
	cv::Mat sampleMat(1, 2, CV_32FC1);

	if (imageOnFlag == 1)
	{
		// determine class of each image pixel
		for (int i = 0; i < (image.rows); ++i)
			for (int j = 0; j < (image.cols); ++j)
			{
				x = i/kx1;
				y = j/kx2;


				//predict sample class for image pixel <-> old code style
				CvMat m;
			    float a[] = { float(x/ScaleMean), float(y/ScaleStd) };
				cvInitMatHeader (&m, 1, 2, CV_32FC1, a);
				pcc = 700*carpet_SVM.predict (&m);


				//plot image pixel
				image.at<Vec3b>(j,i) = Vec3b( pcc, pcc, pcc);

				//plot image pixel
				image2.at<Vec3b>(j, i)  = blue;

			}
	}

	//determines the "area" which still belongs to a certain class
	double r = 0.03;

	//"current true class type"
	double ctct;

	for (int i=0; i<NumTestSamples; i++)
	{
		//////////////////
		///image stuff///
		////////////////

			//determine true sample class
			ctct = test_class_vec[i].dirtThreshold;

			if (imageOnFlag == 1)
			{
				x = ceil(kx1*test_feat_vec[i].mean);
				y = ceil(kx2*test_feat_vec[i].stdDev);

				//daw samples if their true class color to the image (as circles)
				circle( image, Point(x,y), 3, Scalar(floor(ctct*700.0), floor(ctct*700.0), floor(ctct*700.0)), -1 , 8);
			}

		///////////////////////////////////
		///percentage calculation stuff///
		/////////////////////////////////


			//predict sample class
			CvMat m;
		    float a[] = { float(test_feat_vec[i].mean/ScaleMean), float(test_feat_vec[i].stdDev/ScaleStd) };
			cvInitMatHeader (&m, 1, 2, CV_32FC1, a);
			float pcc = carpet_SVM.predict (&m);

			//show result
			std::cout << "True threshold=" << ctct << "\tPredicted threshold=" << pcc
					  << "\tmean=" << x << "\tstd=" << y << std::endl;

			//determine class index
			j = 0;
			while (ClassTypes_vec[j].dirtThreshold != ctct)
			{
				j++;
			}

			//increase the total number of test samples for this class
			num_vec[j].totalnum++;

			//check if test sample in class "area"
			if ((pcc > ctct-r) && (pcc < ctct+r))
//			if (((pcc == 1) && (ctct == 0.25)) || ((pcc == 2) && (ctct == 0.35)))
			{
				num_vec[j].correctnum++;
			}

	} //end-for


	///////////////////////////////////////////////
	///plot position and class of train samples///
	/////////////////////////////////////////////

	if (imageOnFlag == 1)
	{
		for (int i=0; i<NumTrainSamples; i++)
		{
			//determine true sample class
			ctct = train_class_vec[i].dirtThreshold;

			x = ceil(kx1*train_feat_vec[i].mean*ScaleMean);
			y = ceil(kx2*train_feat_vec[i].stdDev*ScaleStd);

			//daw samples if their true class color to the image (as circles)
			circle( image2, Point(x,y), 3, Scalar(floor(ctct*700.0), floor(ctct*700.0), floor(ctct*700.0)), -1 , 8);
		}
	}

	////////////////////////////////////////
	///plot percentage calculation stuff///
	//////////////////////////////////////

	double res = 0;

	for (int i=0; i<NumClass; i++)
	{
		res = 100*(double(num_vec[i].correctnum)/double(num_vec[i].totalnum));

		std::cout 	<< "Dirt threshold (class)=" << ClassTypes_vec[i].dirtThreshold
					<< "\tCorrect/total test-samples=" << num_vec[i].correctnum << "/" << num_vec[i].totalnum
					<< "\tPercentage=" << res <<std::endl;

	}


	///////////////////////
	///show image stuff///
	/////////////////////

	if (imageOnFlag == 1)
	{
		//show image on screen
		imshow("SVM evaluation image", image);
		imshow("Sample image", image2);
		waitKey(0);
	}
}

void DirtDetection::CreateCarpetClassiefierRTree(const std::vector<CarpetFeatures>& carp_feat_vec, const std::vector<CarpetClass>& carp_class_vec, CvRTrees &carpet_Tree)
{
	/////////////////////////////////////////////////////////////////////////////
	///Old style code-> necessary because svm does not work with c++ style!!!///
	///////////////////////////////////////////////////////////////////////////

	//determine number of samples
	int Nges = carp_feat_vec.size();

	//number of features
	int NCarpetFeatures = 2;

	int i;
	float *data = 0;
	data = new float [Nges*2];

	float *res = 0;
	res = new float [Nges];

	CvMat data_mat, res_mat;

	for (i = 0; i < Nges; i++)
	{
	  data[i * 2] = float(carp_feat_vec[i].mean);
	  data[i * 2 + 1] = float(carp_feat_vec[i].stdDev);
	  res[i] = carp_class_vec[i].dirtThreshold;
//	  std::cout << 	"threshold= " << res[i] << std::endl;
	}

	cvInitMatHeader (&data_mat, Nges, NCarpetFeatures, CV_32FC1, data);
	cvInitMatHeader (&res_mat, Nges, 1, CV_32FC1, res);

    // define all the attributes as numerical
    // alternatives are CV_VAR_CATEGORICAL or CV_VAR_ORDERED(=CV_VAR_NUMERICAL)
    // that can be assigned on a per attribute basis
	cv::Mat var_type = cv::Mat(NCarpetFeatures+1, 1, CV_8U );
	var_type.setTo(Scalar(CV_VAR_NUMERICAL) ); // all inputs are numerical

	CvMat varT2 = var_type;

	//////////////////
	//Create-Forest//
	////////////////

    CvRTParams params = CvRTParams(10, // max depth
                                   10, // min sample count
                                   0.00000001f, // regression accuracy
                                   false, // compute surrogate split, no missing data
                                   10, // max number of categories (use sub-optimal algorithm for larger numbers)
                                   0, // the array of priors
                                   true,  // calculate variable importance
                                   0,       // number of variables randomly selected at node and used to find the best split(s).
                                   100,	 // max number of trees in the forest
                                   0.1f,				// forrest accuracy
                                   CV_TERMCRIT_ITER | CV_TERMCRIT_EPS// termination cirteria
                                  );

    //train tree
    carpet_Tree.train(&data_mat, CV_ROW_SAMPLE, &res_mat, NULL, NULL, &varT2, NULL, params);

	//freigabe
	delete[] data;
	delete[] res;

}

void DirtDetection::CreateCarpetClassiefierGBTree(const std::vector<CarpetFeatures>& carp_feat_vec,
		const std::vector<CarpetClass>& carp_class_vec, CvGBTrees &carpet_GBTree)
{
	/////////////////////////////////////////////////////////////////////////////
	///Old style code-> necessary because svm does not work with c++ style!!!///
	///////////////////////////////////////////////////////////////////////////

	//determine number of samples
	int Nges = carp_feat_vec.size();

	//number of features
	int NCarpetFeatures = 2;

	int i;
	float *data = 0;
	data = new float [Nges*2];

	float *res = 0;
	res = new float [Nges];

	CvMat data_mat, res_mat;

	for (i = 0; i < Nges; i++)
	{
	  data[i * 2] = float(carp_feat_vec[i].mean);
	  data[i * 2 + 1] = float(carp_feat_vec[i].stdDev);
	  res[i] = carp_class_vec[i].dirtThreshold;
//	  std::cout << 	"threshold= " << res[i] << std::endl;
	}

	cvInitMatHeader (&data_mat, Nges, NCarpetFeatures, CV_32FC1, data);
	cvInitMatHeader (&res_mat, Nges, 1, CV_32FC1, res);

    // define all the attributes as numerical
    // alternatives are CV_VAR_CATEGORICAL or CV_VAR_ORDERED(=CV_VAR_NUMERICAL)
    // that can be assigned on a per attribute basis
	cv::Mat var_type = cv::Mat(NCarpetFeatures+1, 1, CV_8U );
	var_type.setTo(Scalar(CV_VAR_NUMERICAL) ); // all inputs are numerical

	//transform to mat to cvmat
	CvMat varT2 = var_type;

	//////////////////
	//Create-Forest//
	////////////////


    CvGBTreesParams params = CvGBTreesParams(
									CvGBTrees::SQUARED_LOSS, // CvGBTrees::SQUARED_LOSS, CvGBTrees::ABSOLUTE_LOSS, CvGBTrees::HUBER_LOSS  <-> different possible loss functions for regression problems
									10, 	// Count of boosting algorithm iterations. weak_count*K is the total count of trees in the GBT model, where K is the output classes count (equal to one in case of a regression).
									0.3, 	//shrinkage – Regularization parameter: (0,1]
									0.3f, 	//subsample_portion – Portion of the whole training set used for each algorithm iteration. Subset is generated randomly.
									5, 		//max_depth – Maximal depth of each decision tree in the ensemble
									false	//use_surrogates – If true, surrogate splits are built
										    );

    // train random forest classifier (using training data)

    carpet_GBTree.train(&data_mat, CV_ROW_SAMPLE, &res_mat, NULL, NULL, &varT2, NULL, params, false);


	//freigabe
	delete[] data;
	delete[] res;

}


void DirtDetection::RTreeEvaluation(std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
									std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec,
									CvRTrees &carpet_Tree, double ScaleMean, double ScaleStd)
{

	//number of test samples
	int NumTestSamples = test_feat_vec.size();
	int NumTrainSamples = train_feat_vec.size();

	//saves the different class types
	std::vector<CarpetClass> ClassTypes_vec;

	//number of different class types
	int NumClass = 0;

	//max mean value
	double maxmean = -1;
	//max std. dev.
	double maxstd = -1;

	//loop variables
	bool foundflag = 0;
	int j = 0;

	bool imageOnFlag = 1;

	for(int i = 0; i< NumTrainSamples; i++)
	{
		//test if mean or stdDev bigger than saved values
		if (train_feat_vec[i].mean > maxmean ){maxmean = train_feat_vec[i].mean;}
		if (train_feat_vec[i].stdDev > maxstd ){maxstd = train_feat_vec[i].stdDev;}

		//Determine if new class type
		foundflag = 0;
		j = 0;
		while ((foundflag == 0) && (j<NumClass))
		{
			if (ClassTypes_vec[j].dirtThreshold==train_class_vec[i].dirtThreshold)
			{
				foundflag = 1;
			}
			j++;
		}

		//if new class type, then save it in the list
		if (foundflag == 0)
		{
			ClassTypes_vec.push_back(train_class_vec[i]);
			NumClass++;
		}

	}

	//saves the total number and the number of correct classified samples of each class
	std::vector<NumStruc> num_vec;

	//initialise num_vec with void values
	NumStruc num_struc;
	num_struc.totalnum = 0;
	num_struc.correctnum = 0;

	for(int i = 0; i<NumClass; i++)
	{
		num_vec.push_back(num_struc);
	}


	int x,y;

	//save the prediction of the SVM
	//"predicted current class"
	float pcc;

	//Image for visual representation
	int width = 700, height = 700;

	cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
	cv::Mat image2 = cv::Mat::zeros(height, width, CV_8UC3);

	double kx1 = width/(1.2*maxmean);
	double kx2 = height/(1.2*maxstd);

	Vec3b green(0,255,0), blue (255,0,0), red (0,0,255);

//	cv::Mat sampleMat = cv::Mat::zeros(1, 2, CV_32FC1);
	cv::Mat sampleMat(1, 2, CV_32FC1);

	if (imageOnFlag == 1)
	{
		// determine class of each image pixel
		for (int i = 0; i < (image.rows); ++i)
			for (int j = 0; j < (image.cols); ++j)
			{
				x = i/kx1;
				y = j/kx2;


				//predict sample class for image pixel <-> old code style
				CvMat m;
			    float a[] = { float(x/ScaleMean), float(y/ScaleStd) };
				cvInitMatHeader (&m, 1, 2, CV_32FC1, a);
				pcc = 700*carpet_Tree.predict(&m);


				//plot image pixel
				image.at<Vec3b>(j,i) = Vec3b( pcc, pcc, pcc);

				//plot image pixel
				image2.at<Vec3b>(j, i)  = blue;

			}
	}

	//determines the "area" which still belongs to a certain class
	double r = 0.03;

	//"current true class type"
	double ctct;

	for (int i=0; i<NumTestSamples; i++)
	{
		//////////////////
		///image stuff///
		////////////////

			//determine true sample class
			ctct = test_class_vec[i].dirtThreshold;

			if (imageOnFlag == 1)
			{
				x = ceil(kx1*test_feat_vec[i].mean);
				y = ceil(kx2*test_feat_vec[i].stdDev);

				//daw samples if their true class color to the image (as circles)
				circle( image, Point(x,y), 3, Scalar(floor(ctct*700.0), floor(ctct*700.0), floor(ctct*700.0)), -1 , 8);
			}

		///////////////////////////////////
		///percentage calculation stuff///
		/////////////////////////////////


			//predict sample class
			CvMat m;
		    float a[] = { float(test_feat_vec[i].mean/ScaleMean), float(test_feat_vec[i].stdDev/ScaleStd) };
			cvInitMatHeader (&m, 1, 2, CV_32FC1, a);
			float pcc = carpet_Tree.predict (&m);

			//show result
			std::cout << "True threshold=" << ctct << "\tPredicted threshold=" << pcc
					  << "\tmean=" << x << "\tstd=" << y << std::endl;

			//determine class index
			j = 0;
			while (ClassTypes_vec[j].dirtThreshold != ctct)
			{
				j++;
			}

			//increase the total number of test samples for this class
			num_vec[j].totalnum++;

			//check if test sample in class "area"
			if ((pcc > ctct-r) && (pcc < ctct+r))
//			if (((pcc == 1) && (ctct == 0.25)) || ((pcc == 2) && (ctct == 0.35)))
			{
				num_vec[j].correctnum++;
			}

	} //end-for


	///////////////////////////////////////////////
	///plot position and class of train samples///
	/////////////////////////////////////////////

	if (imageOnFlag == 1)
	{
		for (int i=0; i<NumTrainSamples; i++)
		{
			//determine true sample class
			ctct = train_class_vec[i].dirtThreshold;

			x = ceil(kx1*train_feat_vec[i].mean*ScaleMean);
			y = ceil(kx2*train_feat_vec[i].stdDev*ScaleStd);

			//daw samples if their true class color to the image (as circles)
			circle( image2, Point(x,y), 3, Scalar(floor(ctct*700.0), floor(ctct*700.0), floor(ctct*700.0)), -1 , 8);
		}
	}

	////////////////////////////////////////
	///plot percentage calculation stuff///
	//////////////////////////////////////

	double res = 0;

	for (int i=0; i<NumClass; i++)
	{
		res = 100*(double(num_vec[i].correctnum)/double(num_vec[i].totalnum));

		std::cout 	<< "Dirt threshold (class)=" << ClassTypes_vec[i].dirtThreshold
					<< "\tCorrect/total test-samples=" << num_vec[i].correctnum << "/" << num_vec[i].totalnum
					<< "\tPercentage=" << res <<std::endl;

	}


	///////////////////////
	///show image stuff///
	/////////////////////

	if (imageOnFlag == 1)
	{
		//show image on screen
		imshow("SVM evaluation image", image);
		imshow("Sample image", image2);
		waitKey(0);
	}
}

void DirtDetection::GBTreeEvaluation(	std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
					std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec,
					CvGBTrees &carpet_GBTree, double ScaleMean, double ScaleStd)
{

	//number of test samples
	int NumTestSamples = test_feat_vec.size();
	int NumTrainSamples = train_feat_vec.size();

	//saves the different class types
	std::vector<CarpetClass> ClassTypes_vec;

	//number of different class types
	int NumClass = 0;

	//max mean value
	double maxmean = -1;
	//max std. dev.
	double maxstd = -1;

	//loop variables
	bool foundflag = 0;
	int j = 0;

	bool imageOnFlag = 1;

	for(int i = 0; i< NumTrainSamples; i++)
	{
		//test if mean or stdDev bigger than saved values
		if (train_feat_vec[i].mean > maxmean ){maxmean = train_feat_vec[i].mean;}
		if (train_feat_vec[i].stdDev > maxstd ){maxstd = train_feat_vec[i].stdDev;}

		//Determine if new class type
		foundflag = 0;
		j = 0;
		while ((foundflag == 0) && (j<NumClass))
		{
			if (ClassTypes_vec[j].dirtThreshold==train_class_vec[i].dirtThreshold)
			{
				foundflag = 1;
			}
			j++;
		}

		//if new class type, then save it in the list
		if (foundflag == 0)
		{
			ClassTypes_vec.push_back(train_class_vec[i]);
			NumClass++;
		}

	}

	//saves the total number and the number of correct classified samples of each class
	std::vector<NumStruc> num_vec;

	//initialise num_vec with void values
	NumStruc num_struc;
	num_struc.totalnum = 0;
	num_struc.correctnum = 0;

	for(int i = 0; i<NumClass; i++)
	{
		num_vec.push_back(num_struc);
	}


	int x,y;

	//save the prediction of the SVM
	//"predicted current class"
	float pcc;

	//Image for visual representation
	int width = 700, height = 700;

	cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
	cv::Mat image2 = cv::Mat::zeros(height, width, CV_8UC3);

	double kx1 = width/(1.2*maxmean);
	double kx2 = height/(1.2*maxstd);

	Vec3b green(0,255,0), blue (255,0,0), red (0,0,255);

//	cv::Mat sampleMat = cv::Mat::zeros(1, 2, CV_32FC1);
	cv::Mat sampleMat(1, 2, CV_32FC1);

	if (imageOnFlag == 1)
	{
		// determine class of each image pixel
		for (int i = 0; i < (image.rows); ++i)
			for (int j = 0; j < (image.cols); ++j)
			{
				x = i/kx1;
				y = j/kx2;


				//predict sample class for image pixel <-> old code style
				CvMat m;
			    float a[] = { float(x/ScaleMean), float(y/ScaleStd) };
				cvInitMatHeader (&m, 1, 2, CV_32FC1, a);
				pcc = 700*carpet_GBTree.predict(&m);


				//plot image pixel
				image.at<Vec3b>(j,i) = Vec3b( pcc, pcc, pcc);

				//plot image pixel
				image2.at<Vec3b>(j, i)  = blue;

			}
	}

	//determines the "area" which still belongs to a certain class
	double r = 0.03;

	//"current true class type"
	double ctct;

	for (int i=0; i<NumTestSamples; i++)
	{
		//////////////////
		///image stuff///
		////////////////

			//determine true sample class
			ctct = test_class_vec[i].dirtThreshold;

			if (imageOnFlag == 1)
			{
				x = ceil(kx1*test_feat_vec[i].mean);
				y = ceil(kx2*test_feat_vec[i].stdDev);

				//daw samples if their true class color to the image (as circles)
				circle( image, Point(x,y), 3, Scalar(floor(ctct*700.0), floor(ctct*700.0), floor(ctct*700.0)), -1 , 8);
			}

		///////////////////////////////////
		///percentage calculation stuff///
		/////////////////////////////////


			//predict sample class
			CvMat m;
		    float a[] = { float(test_feat_vec[i].mean/ScaleMean), float(test_feat_vec[i].stdDev/ScaleStd) };
			cvInitMatHeader (&m, 1, 2, CV_32FC1, a);
			float pcc = carpet_GBTree.predict (&m);

			//show result
			std::cout << "True threshold=" << ctct << "\tPredicted threshold=" << pcc
					  << "\tmean=" << x << "\tstd=" << y << std::endl;

			//determine class index
			j = 0;
			while (ClassTypes_vec[j].dirtThreshold != ctct)
			{
				j++;
			}

			//increase the total number of test samples for this class
			num_vec[j].totalnum++;

			//check if test sample in class "area"
			if ((pcc > ctct-r) && (pcc < ctct+r))
//			if (((pcc == 1) && (ctct == 0.25)) || ((pcc == 2) && (ctct == 0.35)))
			{
				num_vec[j].correctnum++;
			}

	} //end-for


	///////////////////////////////////////////////
	///plot position and class of train samples///
	/////////////////////////////////////////////

	if (imageOnFlag == 1)
	{
		for (int i=0; i<NumTrainSamples; i++)
		{
			//determine true sample class
			ctct = train_class_vec[i].dirtThreshold;

			x = ceil(kx1*train_feat_vec[i].mean*ScaleMean);
			y = ceil(kx2*train_feat_vec[i].stdDev*ScaleStd);

			//daw samples if their true class color to the image (as circles)
			circle( image2, Point(x,y), 3, Scalar(floor(ctct*700.0), floor(ctct*700.0), floor(ctct*700.0)), -1 , 8);
		}
	}

	////////////////////////////////////////
	///plot percentage calculation stuff///
	//////////////////////////////////////

	double res = 0;

	for (int i=0; i<NumClass; i++)
	{
		res = 100*(double(num_vec[i].correctnum)/double(num_vec[i].totalnum));

		std::cout 	<< "Dirt threshold (class)=" << ClassTypes_vec[i].dirtThreshold
					<< "\tCorrect/total test-samples=" << num_vec[i].correctnum << "/" << num_vec[i].totalnum
					<< "\tPercentage=" << res <<std::endl;

	}


	///////////////////////
	///show image stuff///
	/////////////////////

	if (imageOnFlag == 1)
	{
		//show image on screen
		imshow("SVM evaluation image", image);
		imshow("Sample image", image2);
		waitKey(0);
	}
}

void DirtDetection::SVMExampleCode()
{

    // Data for visual representation
    int width = 512, height = 512;
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);


    const int NUMBER_OF_TRAINING_SAMPLES = 10;
    const int ATTRIBUTES_PER_SAMPLE = 2;

    // Set up training data
    float labels [NUMBER_OF_TRAINING_SAMPLES] = {-1.0, 3.0, 3.0, 3.0, 2.0, 2.0, 2.0, 2.0, -1.0, -1.0};

    float trainingData[NUMBER_OF_TRAINING_SAMPLES][ATTRIBUTES_PER_SAMPLE] = { {50, 10}, {150, 130}, {120, 190}, {170, 110}, {470, 410}, {450, 420}, {490, 430}, {480, 440}, {40, 30}, {20, 20} };


    // create data and label matrix
    cv::Mat training_data(NUMBER_OF_TRAINING_SAMPLES, ATTRIBUTES_PER_SAMPLE, CV_32FC1, trainingData);
    cv::Mat training_classifications(NUMBER_OF_TRAINING_SAMPLES, 1, CV_32FC1, labels);


    // Set up SVM's parameters
    CvSVMParams params;
    params.svm_type    = CvSVM::NU_SVR;
    params.kernel_type = CvSVM::RBF;

    params.degree = 3.0;
    params.gamma = 1.0;
    params.coef0 = 1.0;
    params.C = 1;
    params.nu = 0.8;
    params.p = 0.1;
    params.class_weights = NULL;


    params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);


    // Train the SVM
    CvSVM SVM;

    //training if parameters are known
//    SVM.train(training_data, training_classifications, Mat(), Mat(), params);


    //define training area:
    cv::ParamGrid gamma_grid( 0.00012207, 10, 2 );
    cv::ParamGrid C_grid( 0.000976562, 5, 2 );
    cv::ParamGrid nu_grid( 0.01546875, 0.99, 1.5 );
    cv::ParamGrid degree_grid(1.5, 5, 1.5);

    //determine optimal parameters and train SVM
    //Comment: It's important to define all input parameters because otherwise "SVM.train_auto" might not work properly!!!
    SVM.train_auto(	training_data, training_classifications, cv::Mat(), cv::Mat(), params, 10,
					C_grid,
					gamma_grid,
					CvSVM::get_default_grid(CvSVM::P),
					nu_grid, CvSVM::get_default_grid(CvSVM::COEF),
					CvSVM::get_default_grid(CvSVM::DEGREE)	);

    //determine ros package path
    std::string svmpath = ros::package::getPath("autopnp_dirt_detection") + "/common/files/svm/surface.svm";
//    std::cout << svmpath << std::endl;
    //save SVM parameters to file
    SVM.save(svmpath.c_str());
    params = SVM.get_params();
    //display SVM parameters on screen
    std::cout << "C=" << params.C << "\ncoeff0=" << params.coef0 << "\ndegree=" << params.degree << "\ngamma=" << params.gamma << "\nnu=" << params.nu << "\np=" << params.p << std::endl;


    Vec3b green(0,255,0), blue (255,0,0), red (0,0,255);

    // Show the decision regions given by the SVM
    for (int i = 0; i < image.rows; ++i)
        for (int j = 0; j < image.cols; ++j)
        {
            cv::Mat sampleMat = (Mat_<float>(1,2) << i,j);
            float response = SVM.predict(sampleMat);

//            if (response <= 1.5)
//            {
//                image.at<Vec3b>(j, i)  = green;
//            }
//            else
//			{
//				if (response >= 2.5)
//				{
//					image.at<Vec3b>(j, i)  = blue;
//				}
//				else
//				{
//					image.at<Vec3b>(j, i)  = red;
//				}
//
//			}

            image.at<Vec3b>(j, i) = Vec3b((response + 2)/6 * 255, (response + 2)/6 * 255, (response + 2)/6 * 255);

        }

    //show training data
    for (int i=0; i<training_data.rows; i++)
	{
    	float label = training_classifications.at<float>(i);
        if (label <= 1.5)
        {
        	circle( image, Point(training_data.at<float>(i, 0),  training_data.at<float>(i, 1)), 5, Scalar(  0,   255,   0), 3, 8);
        }
        else
		{
			if (label >= 2.5)
			{
				circle( image, Point(training_data.at<float>(i, 0),  training_data.at<float>(i, 1)), 5, Scalar(  255,   0,   0), 3, 8);
			}
			else
			{
				circle( image, Point(training_data.at<float>(i, 0),  training_data.at<float>(i, 1)), 5, Scalar(  0,   0,   255), 3, 8);
			}

		}
	}


    //show image on screen
    imshow("SVM Simple Example", image);
    waitKey(0);

}

void DirtDetection::SVMTestFunction()
{
    const int s = 1000;
    int size = 400;
    int i, j, sv_num;
    IplImage *img;
    CvSVM svm = CvSVM ();
    CvSVMParams param;
    CvTermCriteria criteria;
    CvRNG rng = cvRNG (time (NULL));
    CvPoint pts[s];
    float data[s * 2];
    int res[s];
    CvMat data_mat, res_mat;
    CvScalar rcolor;
    const float *support;

    img = cvCreateImage (cvSize (size, size), IPL_DEPTH_8U, 3);
    cvZero (img);

    for (i = 0; i < s; i++) {
      pts[i].x = cvRandInt (&rng) % size;
      pts[i].y = cvRandInt (&rng) % size;
      if (pts[i].y > 50 * cos (pts[i].x * CV_PI / 100) + 200) {
        cvLine (img, cvPoint (pts[i].x - 2, pts[i].y - 2), cvPoint (pts[i].x + 2, pts[i].y + 2), CV_RGB (255, 0, 0));
        cvLine (img, cvPoint (pts[i].x + 2, pts[i].y - 2), cvPoint (pts[i].x - 2, pts[i].y + 2), CV_RGB (255, 0, 0));
        res[i] = 1;
      }
      else {
        if (pts[i].x > 200) {
          cvLine (img, cvPoint (pts[i].x - 2, pts[i].y - 2), cvPoint (pts[i].x + 2, pts[i].y + 2), CV_RGB (0, 255, 0));
          cvLine (img, cvPoint (pts[i].x + 2, pts[i].y - 2), cvPoint (pts[i].x - 2, pts[i].y + 2), CV_RGB (0, 255, 0));
          res[i] = 2;
        }
        else {
          cvLine (img, cvPoint (pts[i].x - 2, pts[i].y - 2), cvPoint (pts[i].x + 2, pts[i].y + 2), CV_RGB (0, 0, 255));
          cvLine (img, cvPoint (pts[i].x + 2, pts[i].y - 2), cvPoint (pts[i].x - 2, pts[i].y + 2), CV_RGB (0, 0, 255));
          res[i] = 3;
        }
      }
    }

    cvNamedWindow ("SVM", CV_WINDOW_AUTOSIZE);
    cvShowImage ("SVM", img);
    cvWaitKey (0);

    for (i = 0; i < s; i++) {
      data[i * 2] = float (pts[i].x) / size;
      data[i * 2 + 1] = float (pts[i].y) / size;
    }
    cvInitMatHeader (&data_mat, s, 2, CV_32FC1, data);
    cvInitMatHeader (&res_mat, s, 1, CV_32SC1, res);
    criteria = cvTermCriteria (CV_TERMCRIT_EPS, 1000, FLT_EPSILON);
    param = CvSVMParams (CvSVM::C_SVC, CvSVM::RBF, 10.0, 8.0, 1.0, 10.0, 0.5, 0.1, NULL, criteria);

    svm.train (&data_mat, &res_mat, NULL, NULL, param);

    for (i = 0; i < size; i++) {
      for (j = 0; j < size; j++) {
        CvMat m;
        float ret = 0.0;
        float a[] = { float (j) / size, float (i) / size };
        cvInitMatHeader (&m, 1, 2, CV_32FC1, a);
        ret = svm.predict (&m);
        switch ((int) ret) {
        case 1:
          rcolor = CV_RGB (100, 0, 0);
          break;
        case 2:
          rcolor = CV_RGB (0, 100, 0);
          break;
        case 3:
          rcolor = CV_RGB (0, 0, 100);
          break;
        }
        cvSet2D (img, i, j, rcolor);
      }
    }

    for (i = 0; i < s; i++) {
      CvScalar rcolor;
      switch (res[i]) {
      case 1:
        rcolor = CV_RGB (255, 0, 0);
        break;
      case 2:
        rcolor = CV_RGB (0, 255, 0);
        break;
      case 3:
        rcolor = CV_RGB (0, 0, 255);
        break;
      }
      cvLine (img, cvPoint (pts[i].x - 2, pts[i].y - 2), cvPoint (pts[i].x + 2, pts[i].y + 2), rcolor);
      cvLine (img, cvPoint (pts[i].x + 2, pts[i].y - 2), cvPoint (pts[i].x - 2, pts[i].y + 2), rcolor);
    }

    sv_num = svm.get_support_vector_count ();
    for (i = 0; i < sv_num; i++) {
      support = svm.get_support_vector (i);
      cvCircle (img, cvPoint ((int) (support[0] * size), (int) (support[1] * size)), 5, CV_RGB (200, 200, 200));
    }

    cvNamedWindow ("SVM", CV_WINDOW_AUTOSIZE);
    cvShowImage ("SVM", img);
    cvWaitKey (0);

    cvDestroyWindow ("SVM");
    cvReleaseImage (&img);
}




