#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>


class Inflation_Point
{
	public:
	std::vector<double> I_X;
	std::vector<double> I_Y;

	ros::NodeHandle m_n;
	ros::Subscriber m_sub;

	void chatterCallback(const nav_msgs::GridCells::ConstPtr& msg);
	void init(ros::NodeHandle nh);
};

void Inflation_Point::chatterCallback(const nav_msgs::GridCells::ConstPtr& msg)
{
	for(unsigned int i=0; i< msg->cells.size() ; i++)
	{

		I_X.push_back(msg->cells[i].x);
		I_Y.push_back(msg->cells[i].y);
		//if((int)(I_X[i] - (-22.40))/0.05 != 500 && (int)(I_X[i] - (-22.40))/0.05 != 480)

		//{
			//std::cout<< "\nX : " << I_X[i]; //- (-22.40))/0.05 <<"\n";
			std::cout<< "\nY : " << I_Y[i]; //- (-22.40))/0.05 <<"\n";
		//}
		//if(((int)(I_Y[i] - (-22.40))/0.05) > 400)
		//std::cout<< "\nY : " << (int)(I_Y[i] - (-19.20))/0.05 <<"\n";
	}

	//std::cout<< "\nCell_Width : " << msg->cell_width <<"\n";
	//std::cout<< "\nCell_height : " << msg->cell_height <<"\n";
	//std::cout<< "\nCell_Width : " << (int)(msg->cell_width - (-22.40))/0.05 <<"\n";
	//std::cout<< "\nCell_Height : " << (int)(msg->cell_width - (-22.40))/0.05 <<"\n";

}

void Inflation_Point::init(ros::NodeHandle nh)
	{
		m_n = nh;
		m_sub = m_n.subscribe("/move_base/local_costmap/inflated_obstacles", 1000, &Inflation_Point::chatterCallback,this);
	}

int main(int argc, char **argv)
{

	  ros::init(argc, argv, "Get_Inflation_Point");
	  ros::NodeHandle n;

	  Inflation_Point My;

	  My.init(n);

	  ros::spin();

	  return 0;
}
