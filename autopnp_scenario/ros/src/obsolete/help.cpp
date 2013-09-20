#include "ros/ros.h"
#include "accompany_context_aware_planner/GetPotentialProxemicsLocations.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <math.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include <opencv2/ml/ml.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>

/* MySQL Connector/C++ specific headers */
#include <cppconn/driver.h>
#include <cppconn/connection.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>
#include <cppconn/resultset.h>
#include <cppconn/metadata.h>
#include <cppconn/resultset_metadata.h>
#include <cppconn/exception.h>
#include <cppconn/warning.h>

//#define DBHOST "tcp://127.0.0.1:3306"
//#define DBHOST "tcp://localhost:3306"
#define DBHOST "tcp://10.0.1.54:3306"
#define USER "rhUser"
#define PASSWORD "waterloo"
#define DATABASE "Accompany"

#define NUMOFFSET 100
#define COLNAME 200

using namespace std;
using namespace sql;

struct Proxemics_bearing
{
    float distance;
    float orientation;
};

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

///** from here
class ApproachTest
{
public:

	void init(ros::NodeHandle nh)
	{
		node_handle_ = nh;
		map_resolution_ = 0;

		robotRadius = 0.6;	// in [m]
		personRadius = 0.4;	// in [m]

		static_map_sub_ = node_handle_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &ApproachTest::updateMapCallback, this);

		service_server_get_potential_proxemics_locations_ = node_handle_.advertiseService("get_potential_proxemics_locations", &ApproachTest::getPotentialProxemicsLocations, this);
		ROS_INFO("Ready to provide potential proxemics based robot target pose.");
	}

	// this function just copies the received map into opencv's format
	// and creates an inflated version of the map
	void updateMapCallback(const nav_msgs::OccupancyGridConstPtr& map_msg)
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

		// create the inflated map
		int iterations = (int)(robotRadius/map_resolution_);
		//std::cout << "iterations=" << iterations << std::endl;
		cv::erode(map_, expanded_map_, cv::Mat(), cv::Point(-1,-1), iterations);

		// display maps
//		cv::imshow("blown up map", expanded_map_);
//		cv::imshow("map", map_);
//		cv::waitKey(10);


		ROS_INFO("Map received.");
	}

	template <class T>
	T convertFromMeterToPixelCoordinates(const Pose& pose)
	{
		T val;
		val.x = (pose.x - map_origin_.x)/map_resolution_;
		val.y = (pose.y - map_origin_.y)/map_resolution_;
		return val;
	}

	// this function computes whether a given point (potentialApproachPose) is accessible by the robot at location robotLocation
	bool validApproachPosition(Pose personLocation, Pose robotLocation, Pose potentialApproachPose)
	//			    just some obstacle, robots coordinates, the next goal pose for the robot
	{
		// convert coordinates to pixels
		cv::Point potentialApproachPosePixel = convertFromMeterToPixelCoordinates<cv::Point>(potentialApproachPose);
		cv::Point personLocationPixel = convertFromMeterToPixelCoordinates<cv::Point>(personLocation);

		// copy expanded map
		cv::Mat expanded_map_with_person = expanded_map_.clone();

		// draw person into map as obstacle
		int personRadiusPixel = (int)((personRadius+robotRadius)/map_resolution_);
		//std::cout << "cx:" << center.x << "  cy:" << center.y << "  mx:" << map_origin_.x << "  my:" << map_origin_.y << "  resolution:" << map_resolution_ << std::endl;
		cv::circle(expanded_map_with_person, personLocationPixel, personRadiusPixel, cv::Scalar(0,0,0,0), -1);

		// display new inflated map
		//cv::imshow("inflated map", expanded_map_with_person);
		//cv::waitKey(10);

		// find the individual connected areas
		std::vector< std::vector<cv::Point> > contours;		// first index=contour index;  second index=point index within contour
		//std::vector<cv::Vec4i> hierarchy;
		cv::Mat expanded_map_with_person_copy = expanded_map_with_person.clone();
		cv::findContours(expanded_map_with_person_copy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

		// display found contours
		cv::drawContours(expanded_map_with_person, contours, -1, cv::Scalar(128,128,128,128), 2);
		cv::circle(expanded_map_with_person, convertFromMeterToPixelCoordinates<cv::Point>(robotLocation), 3, cv::Scalar(200,200,200,200), -1);
		cv::circle(expanded_map_with_person, potentialApproachPosePixel, 3, cv::Scalar(200,200,200,200), -1);
		//cv::line(expanded_map_with_person, convertFromMeterToPixelCoordinates<cv::Point>(Pose(1.f,0.2f,0.f)), convertFromMeterToPixelCoordinates<cv::Point>(Pose(-1.f,0.2f,0.f)), cv::Scalar(0,0,0,0), 2);
		//cv::line(map_, convertFromMeterToPixelCoordinates<cv::Point>(Pose(1.f,0.2f,0.f)), convertFromMeterToPixelCoordinates<cv::Point>(PoApproachTestse(-1.f,0.2f,0.f)), cv::Scalar(0,0,0,0), 2);
		cv::imshow("contour areas", expanded_map_with_person);
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
		std::cout << "contourIndexPotentialApproachPose=" << contourIndexPotentialApproachPose << "  contourIndexRobot=" << contourIndexRobot << std::endl;
		if (contourIndexRobot != contourIndexPotentialApproachPose || (contourIndexRobot==-1 && contourIndexPotentialApproachPose==-1))
			return false;

		// check whether there is an obstacle in direct line of sight between personLocation and potentialApproachPose
		double dx = personLocationPixel.x - potentialApproachPosePixel.x;
		double dy = personLocationPixel.y - potentialApproachPosePixel.y;
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

		return true;
	}
///*** till here

	bool getPotentialProxemicsLocations(accompany_context_aware_planner::GetPotentialProxemicsLocations::Request &req,
			accompany_context_aware_planner::GetPotentialProxemicsLocations::Response &res)
	{
	    Pose targetPose;
	    Proxemics_bearing procx_bearing;

	//1. Process the request data from the client.
	    //ROS_INFO("MsgSeq = %d, time =  %li, coordinate frame = %s ",req.header.seq, static_cast<long>(ros::Time::now().toNSec()-req.header.stamp.toNSec()), req.header.frame_id.c_str());
	    ROS_INFO("MsgSeq = %d, time = %2f, coordinate frame = %s ",req.header.seq, (ros::Time::now().toSec()-req.header.stamp.toSec()), req.header.frame_id.c_str());
	    ROS_INFO("userId = %d, userPosture = %d", req.userId, req.userPosture);
	    Pose personLocation(req.userPose.position.x, req.userPose.position.y, tf::getYaw(req.userPose.orientation));
	    ROS_INFO("request proxemics targets for: x=%f, y=%f, z=%f yaw = %f", req.userPose.position.x, req.userPose.position.y, req.userPose.position.z, tf::getYaw(req.userPose.orientation));
	    ROS_INFO("robotGenericTask: %d", req.robotGenericTaskId);

	//2. Retreives user's preference
	    procx_bearing = retrieveProxemicsPreferences(req.userId, req.robotGenericTaskId);

	    //To do
	        //multiple poses
	        //search based on distance
	        //search based on orientation


	//3. Calculate the all the target poses from the database, where distance and orientation can be obtain from step2
	    targetPose = calculateRobotPoseFromProxemicsPreference(req.userPose, procx_bearing);
	    // To do
	        //multiple poses

	//4. Elimate poses that are not in the same location as the user (i.e. user is in living room, therefore all potential robot poses have to be in the living room for HRI)



	//5. Eliminate poses that could not be occupied by the robot based on static map (i.e. too close to obstacle or on obstacle)



	//6. Eliminate poses that could not be reach by the robot based on static map
	    Pose robotLocation(2,1,0);		// todo: read in robot location
	    bool validApproach = validApproachPosition(personLocation, robotLocation, targetPose);
	    if (validApproach == true)
	    	ROS_INFO("The approach position is valid.");
	    else
	    	ROS_INFO("The approach position is invalid.");


	//7. Compile the respond message for the client.
		/*The respond vector data from the server are in the following format and contain a list of potential target locations.
		  These potential target poses needed to be varify in the real environment to take into account of dynamic obstacles etc.

			res.targetPoses[i].seq
			res.targetPoses[i].stamp
			res.targetPoses[i].frame_id

			res.targetPoses[i].pose.x
			res.targetPoses[i].pose.y
			res.targetPoses[i].pose.z

			res.targetPoses[i].orientation.x
			res.targetPoses[i].orientation.y
			res.targetPoses[i].orientation.z
			res.targetPoses[i].orientation.w
		*/

	    tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose> (tf::Pose(tf::createQuaternionFromYaw(degree2radian(targetPose.orientation)),
	                                                                                          tf::Point(targetPose.x, targetPose.y, 0.0)),
	                                                              ros::Time::now(),
	                                                              "map");

	    geometry_msgs::PoseStamped pose;	//create a PoseStamped variable to store the StampedPost TF
	    tf:poseStampedTFToMsg(p, pose);	//convert the PoseStamped data into message format and store in pose
	    res.targetPoses.push_back(pose);	//push the pose message into the respond vector to be send back to the client

	    ROS_INFO("Sending request out");
	  //res.sum = req.a + req.b;publishLocalPlan
	  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
	  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
	  return true;
	}

	template <class T>
	inline std::string to_string (const T& t)
	{
	std::stringstream ss;
	ss << t;
	return ss.str();
	}

	float degree2radian(float degree)
	{
		float radian;
		float pi = 4.0 * std::atan2(1.0,1.0);

		return radian = pi * degree/180.0;
	}

	float radian2degree(float radian)
	{
		float degree;
	        float pi = 4.0 * std::atan2(1.0,1.0);
		return degree = 180 * radian/pi;
	}

	/******************************************************************************
	Calculate the the robot's target pose relative to the user's pose, based on
	the user's proxemics preference

		userPose: is the user pose (x,y,z, Quaternion).

		prefOrientation: is the user prefered robot's approach direction with respect
			to the user's coordinate frame in radian.

		prefDistance is: the user preferred robot's approach distance with respect
			to the user's coordinate frame in meter.
	******************************************************************************/
	Pose calculateRobotPoseFromProxemicsPreference(geometry_msgs::Pose &userPose, Proxemics_bearing prefBearing)
	{

	float x_tar, y_tar, theta_tar, x_usr, y_usr, theta_usr, d_x, d_y;
	float x_tar_temp, y_tar_temp;
	float prefDistance;
	float prefOrientation;
	Pose targetPose;

	    prefDistance = prefBearing.distance;
	    prefOrientation = degree2radian(prefBearing.orientation);

	    ROS_INFO("Calculate proxemics targets for: x=%f, y=%f, z=%f yaw = %f",
	            userPose.position.x, userPose.position.y, userPose.position.z, tf::getYaw(userPose.orientation));


	// Determines the robot target coordinate in user's coordinate frame
	    x_tar = prefDistance * cos(prefOrientation);
	    y_tar = prefDistance * sin(prefOrientation);

	//Determines the robot target coordinate in map's coordinate frame
	    //1. Retrieve user's coordinate in map's coordinate frame
	    x_usr = userPose.position.x;
	    y_usr = userPose.position.y;
	    theta_usr = tf::getYaw(userPose.orientation);

	    //2.Calculate the robot target's relative to the user, taking into account the orientation of the user in map frame.
	    //i.e. Rotation
	    x_tar_temp = x_tar*cos(theta_usr) - y_tar*sin(theta_usr);
	    y_tar_temp = x_tar*sin(theta_usr) + y_tar*cos(theta_usr);

	    //3. Calculate the robot target's position in map coordinate frame.
	    //i.e. Translation
	    x_tar = x_usr + x_tar_temp;
	    y_tar = y_usr + y_tar_temp;

	    //4. Calculate the robot target's orientation in map coordinate frame.
	    d_x = x_usr - x_tar;
	    d_y = y_usr - y_tar;
	    theta_tar = atan2(d_y,d_x);

	    ROS_INFO("Robot Target x=%f, y=%f, theta = %f",x_tar, y_tar, radian2degree(theta_tar));

	    targetPose.x = x_tar;
	    targetPose.y = y_tar;
	    targetPose.orientation = radian2degree(theta_tar);


	return targetPose;
	}

	/******************************************************************************
	Retrieve user's proxemics preferences from the database

	******************************************************************************/
	Proxemics_bearing retrieveProxemicsPreferences(int userId, int robotGenericTaskId)
	{
	    string test, temp1, temp2;
	    stringstream out;

	    Proxemics_bearing procx;

	    Driver *driver;
	    Connection *con;
	    Statement *stmt;
	    ResultSet *result;

	        driver = get_driver_instance();
	        con = driver->connect(DBHOST, USER, PASSWORD); // create a database connection using the Driver
	        con->setAutoCommit(0);                      // turn off the autocommit
	        con->setSchema(DATABASE);                   // select appropriate database schema
	        stmt = con->createStatement();              // create a statement object


	        float distance = 0;
	        float orientation = 0;

	        test = "SELECT * FROM UserProxemicPreferences where UserProxemicPreferences.userId = ";
	        test += to_string(userId);
	        test += " and UserProxemicPreferences.robotGenericTaskId = ";
	        test += to_string(robotGenericTaskId);
	        cout<<test<<endl;
	        result = stmt->executeQuery(test);

	        test = "SELECT * FROM Accompany.Proxemics where Accompany.Proxemics.proxemicId = ";
	        while (result->next())
	                test +=  result -> getString("proxemicId");
	        cout<<test<<endl;
	        result = stmt->executeQuery(test);

	        test = "SELECT * FROM RobotApproachDistance where RobotApproachDistance.robotApproachDistanceId = ";
	        while (result->next()) {
	                temp1 = result -> getString("robotApproachDistanceId");
	                temp2 = result -> getString("robotApproachOrientationId");
	        }
	        test += temp1;
	        cout<<test<<endl;
	        result = stmt->executeQuery(test);

	        while (result->next())
	                distance = (float) result -> getDouble("distance");


	        test = "SELECT * FROM RobotApproachOrientation where RobotApproachOrientation.robotApproachOrientationId = ";
	        test += temp2;
	        cout<<test<<endl;
	        result = stmt->executeQuery(test);

	        while (result->next())
	                orientation = result -> getDouble("orientation");

	        cout<< "Distance = " << distance << " Orientation = " << orientation << endl;

	        procx.distance = distance;
	        procx.orientation = orientation; //in degree

	        /* Simulated user's pose. The actual user's pose will be provided by the caller*/
	        // geometry_msgs::Pose userPose;
	        // userPose.orientation = tf::createQuaternionMsgFromYaw( degree2radian(87.6) ); //create Quaternion Msg from Yaw
	        // tf::pointTFToMsg(tf::Point(1, 1, 0), userPose.position);

	        delete result;
	        delete stmt;
	        con -> close();
	        delete con;

	    return procx;
	      //  return true; //stmt->executeQuery(test);
	}


protected:

	ros::NodeHandle node_handle_;
	ros::Subscriber static_map_sub_;

	ros::ServiceServer service_server_get_potential_proxemics_locations_; 	// Service server providing proxemics locations

	double robotRadius;		// in [m]
	double personRadius;	// in [m]

	cv::Mat map_;
	cv::Mat expanded_map_;
	double map_resolution_;		// in [m/cell]
	cv::Point2d map_origin_;	// in [m]

};


/*
bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}
*/




/******************************************************************************
This will be the main function for Proxemics.

  - The request data from the client to determine potential proxemics locations are:
        req.header.seq		- can be used to stored id of the client and be return to the client for identification etc. if needed  -needed to be varify if this is nessessary
        req.header.stamp	- time stamped when the request is created i.e. ros::Time::now()
        req.header.frame_id - the user coordinate's reference frame i.e. "map"

        req.userId			- the user's Id in the database
        req.userPosture		- the user's posture i.e. standing/seating

        //the user's coordinate i.e. in map's coordinate frame in meter
        req.userPose.position.x		- the x coordinate of the user
        req.userPose.position.y		- the y coordinate of the user
        req.userPose.position.z		- the z coordinate of the user

        //the user's orientation in Quaternion. the heading direction of the user can be extract by using tf::getYaw(req.userPose.orientation) function
        // which return yaw in radians
        req.userPose.orientation.x
        req.userPose.orientation.y
        req.userPose.orientation.z
        req.userPose.orientation.w

        //Type of task the robot is going to perform using this information
        req.robotGenericTaskId          - the type of task the robot is going to perform using this proxemics

******************************************************************************/


static void retrieve_data_and_print (ResultSet *rs, int type, int colidx, string colname) {

	/* retrieve the row count in the result set */
	cout << "\nRetrieved " << rs -> rowsCount() << " row(s)." << endl;

	cout << "--------" << endl;

	/* fetch the data : retrieve all the rows in the result set */
	while (rs->next()) {
		if (type == NUMOFFSET) {
                       cout << rs -> getString(colidx) << "Numoffset" << endl;	//retrieve data using column id
		} else if (type == COLNAME) {
                       cout << rs -> getString(colname) << "Colname" << endl;	//retrieve data using column name
		} // if-else
	} // while

	cout << endl;

} // retrieve_data_and_print()

static void retrieve_dbmetadata_and_print (Connection *dbcon) {

	if (dbcon -> isClosed()) {
		throw runtime_error("DatabaseMetaData FAILURE - database connection closed");
	}

	cout << "\nDatabase Metadata" << endl;
	cout << "-----------------" << endl;

	cout << boolalpha;

	/* The following commented statement won't work with Connector/C++ 1.0.5 and later */
	//auto_ptr < DatabaseMetaData > dbcon_meta (dbcon -> getMetaData());

	DatabaseMetaData *dbcon_meta = dbcon -> getMetaData();

	cout << "Database Product Name: " << dbcon_meta -> getDatabaseProductName() << endl;
	cout << "Database Product Version: " << dbcon_meta -> getDatabaseProductVersion() << endl;
	cout << "Database User Name: " << dbcon_meta -> getUserName() << endl << endl;

	cout << "Driver name: " << dbcon_meta -> getDriverName() << endl;
	cout << "Driver version: " << dbcon_meta -> getDriverVersion() << endl << endl;

	cout << "Database in Read-Only Mode?: " << dbcon_meta -> isReadOnly() << endl;
	cout << "Supports Transactions?: " << dbcon_meta -> supportsTransactions() << endl;
	cout << "Supports DML Transactions only?: " << dbcon_meta -> supportsDataManipulationTransactionsOnly() << endl;
	cout << "Supports Batch Updates?: " << dbcon_meta -> supportsBatchUpdates() << endl;
	cout << "Supports Outer Joins?: " << dbcon_meta -> supportsOuterJoins() << endl;
	cout << "Supports Multiple Transactions?: " << dbcon_meta -> supportsMultipleTransactions() << endl;
	cout << "Supports Named Parameters?: " << dbcon_meta -> supportsNamedParameters() << endl;
	cout << "Supports Statement Pooling?: " << dbcon_meta -> supportsStatementPooling() << endl;
	cout << "Supports Stored Procedures?: " << dbcon_meta -> supportsStoredProcedures() << endl;
	cout << "Supports Union?: " << dbcon_meta -> supportsUnion() << endl << endl;

	cout << "Maximum Connections: " << dbcon_meta -> getMaxConnections() << endl;
	cout << "Maximum Columns per Table: " << dbcon_meta -> getMaxColumnsInTable() << endl;
	cout << "Maximum Columns per Index: " << dbcon_meta -> getMaxColumnsInIndex() << endl;
	cout << "Maximum Row Size per Table: " << dbcon_meta -> getMaxRowSize() << " bytes" << endl;

	cout << "\nDatabase schemas: " << endl;

	auto_ptr < ResultSet > rs ( dbcon_meta -> getSchemas());

	cout << "\nTotal number of schemas = " << rs -> rowsCount() << endl;
	cout << endl;

	int row = 1;

	while (rs -> next()) {
		cout << "\t" << row << ". " << rs -> getString("TABLE_SCHEM") << endl;
		++row;
	} // while

	cout << endl << endl;

} // retrieve_dbmetadata_and_print()

static void retrieve_rsmetadata_and_print (ResultSet *rs) {

	if (rs -> rowsCount() == 0) {
		throw runtime_error("ResultSetMetaData FAILURE - no records in the result set");
	}

	cout << "ResultSet Metadata" << endl;
	cout << "------------------" << endl;

	/* The following commented statement won't work with Connector/C++ 1.0.5 and later */
	//auto_ptr < ResultSetMetaData > res_meta ( rs -> getMetaData() );

	ResultSetMetaData *res_meta = rs -> getMetaData();

	int numcols = res_meta -> getColumnCount();
	cout << "\nNumber of columns in the result set = " << numcols << endl << endl;

	cout.width(20);
	cout << "Column Name/Label";
	cout.width(20);
	cout << "Column Type";
	cout.width(20);
	cout << "Column Size" << endl;

	for (int i = 0; i < numcols; ++i) {
		cout.width(20);
		cout << res_meta -> getColumnLabel (i+1);
		cout.width(20);
		cout << res_meta -> getColumnTypeName (i+1);
		cout.width(20);
		cout << res_meta -> getColumnDisplaySize (i+1) << endl << endl;
	}

	cout << "\nColumn \"" << res_meta -> getColumnLabel(1);
	cout << "\" belongs to the Table: \"" << res_meta -> getTableName(1);
	cout << "\" which belongs to the Schema: \"" << res_meta -> getSchemaName(1) << "\"" << endl << endl;

} // retrieve_rsmetadata_and_print()


int main(int argc, char **argv)
{
	ros::init(argc, argv, "context_aware_planner_server");
	ros::NodeHandle n;

	ApproachTest ap;
	ap.init(n);

//       Proxemics procx;


//-----
//	Driver *driver;
//	Connection *con;
//	Statement *stmt;
//	ResultSet *res;
//	PreparedStatement *prep_stmt;
//	Savepoint *savept;



/* initiate url, user, password and database variables */
//	string url(argc >= 2 ? argv[1] : DBHOST);
//	const string user(argc >= 3 ? argv[2] : USER);
//	const string password(argc >= 4 ? argv[3] : PASSWORD);
//	const string database(argc >= 5 ? argv[4] : DATABASE);
//
//	int updatecount = 0;

	/*
	try {
		driver = get_driver_instance();

                // create a database connection using the Driver
		con = driver->connect(url, user, password);

                // turn off the autocommit
		con->setAutoCommit(0);
		cout << "\nDatabase connection\'s autocommit mode = " << con -> getAutoCommit() << endl;

                // select appropriate database schema
		con->setSchema(database);

                // retrieve and display the database metadata
		retrieve_dbmetadata_and_print(con);

                // create a statement object
                stmt = con->createStatement();

        //retreives user's preferences
		cout << "Executing the Query: ...." << endl;
                //retrieveProxemicsPreferences(stmt, 1, 2); // determine the proxemicsPreference for userId 1, with robotGenericTask 2


                cout << "Demonstrating Prepared Statements .. " << endl << endl;

                // insert couple of rows of data into City table using Prepared Statements
                prep_stmt = con -> prepareStatement ("INSERT INTO City (CityName) VALUES (?)");
		cout << "\tInserting \"London, UK\" into the table, City .." << endl;

		prep_stmt -> setString (1, "London, UK");
		updatecount = prep_stmt -> executeUpdate();

		cout << "\tCreating a save point \"SAVEPT1\" .." << endl;
		savept = con -> setSavepoint ("SAVEPT1");

		cout << "\tInserting \"Paris, France\" into the table, City .." << endl;

		prep_stmt -> setString (1, "Paris, France");
		updatecount = prep_stmt -> executeUpdate();

		cout << "\tRolling back until the last save point \"SAVEPT1\" .." << endl;
		con -> rollback (savept);
		con -> releaseSavepoint (savept);

		cout << "\tCommitting outstanding updates to the database .." << endl;
		con -> commit();

		cout << "\nQuerying the City table again .." << endl;

                // re-use result set object
                res = NULL;
                res = stmt -> executeQuery ("SELECT * FROM City");

                // retrieve the data from the result set and display on stdout
                retrieve_data_and_print (res, COLNAME, 1, string ("CityName"));

                cout << "Cleaning up the resources .." << endl;

                // Clean up
		delete res;
		delete stmt;
		delete prep_stmt;
		con -> close();
		delete con;

	} catch (SQLException &e) {
		cout << "ERROR: SQLException in " << __FILE__;
		cout << " (" << __func__<< ") on line " << __LINE__ << endl;
		cout << "ERROR: " << e.what();
		cout << " (MySQL error code: " << e.getErrorCode();
		cout << ", SQLState: " << e.getSQLState() << ")" << endl;

		if (e.getErrorCode() == 1047) {

                        //Error: 1047 SQLSTATE: 08S01 (ER_UNKNOWN_COM_ERROR)
                        //Message: Unknown command

			cout << "\nYour server does not seem to support Prepared Statements at all. ";
			cout << "Perhaps MYSQL < 4.1?" << endl;
		}

		return EXIT_FAILURE;

	} catch (std::runtime_error &e) {

		cout << "ERROR: runtime_error in " << __FILE__;
		cout << " (" << __func__ << ") on line " << __LINE__ << endl;
		cout << "ERROR: " << e.what() << endl;

		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
*/
//-----

  ros::spin();

  return 0;
}
