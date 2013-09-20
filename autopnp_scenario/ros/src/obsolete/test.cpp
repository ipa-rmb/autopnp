#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
  ros::init(argc, argv,"hello");

  tf::TransformListener listener;

  ros::Rate rate(10.0);
	tf::StampedTransform transform;
	int node = 0;
	while (node !=10)
	{
	try
		{
		  listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);
		  ROS_INFO("\nTranslation of the Base Link with respect to Map in x-direction: %.2f,"
				   "\nTranslation of the Base Link with respect to Map in y-direction: %.2f,"
				   "\nRotation of the Base Link with respect to Map in z-direction: %.2f .",
					 transform.getOrigin().x(), transform.getOrigin().y(), transform.getRotation().z());
		}

	catch (tf::TransformException ex)
		{
		  ROS_ERROR("%s",ex.what());
		}
	node++;
	rate.sleep();
	}
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal Goal;

  geometry_msgs::PoseStamped goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = transform.getOrigin().x();
  goal.pose.position.y = transform.getOrigin().y();
  //goal.target_pose.pose.orientation.z = tf::createQuaternionFromYaw(0.0);
  tf::Quaternion quat = tf::createQuaternionFromYaw(3.1416/2);
  tf::quaternionTFToMsg(quat, goal.pose.orientation);

  //goal.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  Goal.target_pose = goal;
  ac.sendGoal(Goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  //goal.target_pose.pose.position.y = -1;
  //goal.target_pose.pose.orientation.z = 0;
  //goal.target_pose.pose.orientation.w = 1.0;
  //ROS_INFO("Sending goal");
  //ac.sendGoal(goal);

  //ac.waitForResult();



  return 0;
}
