#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
	{
		ros::init(argc, argv, "Listener");

		ros::NodeHandle node;

		tf::TransformListener listener;

		ros::Rate rate(10.0);
		while (node.ok())
			{
				tf::StampedTransform transform;

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

				rate.sleep();
			}

		return 0;
	}
