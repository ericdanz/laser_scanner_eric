#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include <sstream>

//Global var of map meta data
nav_msgs::MapMetaData mmd;





void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //Figure out if I need to use another pointer for this msg

  nav_msgs::OccupancyGrid newGrid = OGridFromLScan(msg, mmd);

  //remake publisher? or just constantly update a global map that then gets pushed each spin? 
  chatter_pub.publish(newGrid);

}

//This is where the work gets done
nav_msgs::OccupancyGrid OGridFromLScan(sensor_msgs::LaserScan lmsg, nav_msgs::MapMetaData mapmetad)
{


}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "scantomap");

  ros::NodeHandle n;

 

  ros::Subscriber sub = n.subscribe("scan_data", 10, chatterCallback);

  ros::Publisher chatter_pub = n.advertise<nav_msgs::OccupancyGrid>("map_data", 1, true);

  
 
  ros::spin();



  return 0;
}
