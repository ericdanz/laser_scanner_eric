#include "laser_scanner_eric/oGridMaker.h"




int main(int argc, char **argv)
{
  
  	ros::init(argc, argv, "scantomap");
  
    	oGridMaker ogm = oGridMaker();

  	//ogmaker.initialize(nh);

  	//ros::Publisher chatter_pub = n.advertise<nav_msgs::OccupancyGrid>("map_data", 1, true);


  	//ros::Subscriber sub = n.subscribe("scan_data", 10, chatterCallback);

 
  	ros::spin();



  return 0;
}
