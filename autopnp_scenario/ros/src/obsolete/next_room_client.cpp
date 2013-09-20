#include <iostream>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <autopnp_scenario/NextRoomAction.h>

int main (int argc, char **argv)
	{
		ros::init(argc, argv, "test_Find_Next_Room");

		actionlib::SimpleActionClient<autopnp_scenario::NextRoomAction> ac("Find_Next_Room", true);

		ROS_INFO("Waiting for action server to start.");

		// wait for the action server to start
		ac.waitForServer(); //will wait for infinite time

		ROS_INFO("Action server started, sending goal.");
		// send a goal to the action

		autopnp_scenario::NextRoomActionGoal goal;

		//goal.map_resolution =
		//goal.Map_Origin_x =
		//goal.Map_Origin_y =

		//goal.input_img =
		//goal.room_center_x =
		//goal.room_center_y =

		//ac.sendGoal(goal);

		//wait for the action to return
		bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

		if (finished_before_timeout)
			{
				actionlib::SimpleClientGoalState state = ac.getState();
				ROS_INFO("Action finished: %s",state.toString().c_str());
			}

		else
		ROS_INFO("Action did not finish before the time out.");

		//exit
		return 0;
	}
