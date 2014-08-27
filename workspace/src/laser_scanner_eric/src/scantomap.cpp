#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include <sstream>





class oGridMaker {
	private: 
		ros::NodeHandle n;
		ros::Publisher mapper_pub;
		ros::Subscriber sub;
		nav_msgs::MapMetaData mmd;
		nav_msgs::OccupancyGrid mapGrid;
	public:
		void initialize(ros::NodeHandle &nh)
		{
			n = nh
			mapper_pub = n.advertise<nav_msgs::OccupancyGrid>("map_data", 1, true);

			// or this->scannerCallBack
	  		sub = n.subscribe("scan_data", 10, scannerCallback);
		}
		//this is where the work gets done
		nav_msgs::OccupancyGrid OGridFromLScan(const sensor_msgs::LaserScan::ConstPtr& msg, nav_msgs::MapMetaData mapmetad)
		{	


		}

	
		void scannerCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
		{
	  		//Figure out if I need to use another pointer for this msg

	  		mapGrid = OGridFromLScan(msg, mmd);

	 	 	//remake publisher? or just constantly update a global map that then gets pushed each spin? 
	  		mapper_pub.publish(mapGrid);

		}


} ogmaker;






int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "scantomap");
  
  ros::NodeHandle nh;

  ogmaker.initialize(nh);

  //ros::Publisher chatter_pub = n.advertise<nav_msgs::OccupancyGrid>("map_data", 1, true);


  //ros::Subscriber sub = n.subscribe("scan_data", 10, chatterCallback);

  

  
 
  ros::spin();



  return 0;
}
