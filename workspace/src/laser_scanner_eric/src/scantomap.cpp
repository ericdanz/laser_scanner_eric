#include "ros/ros.h"
#include "nav_msgs/MapMetaData"
#include "nav_msgs/OccupancyGrid"
#include "sensor_msgs/LaserScan"
#include <sstream>

//Global var of map meta data
nav_msgs::MapMetaData mmd;





void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //Figure out if I need to use another pointer for this msg

  nav_msgs::OccupancyGrid newGrid = OGridFromLScan(msg, mmd);

  chatter_pub.publish(newGrid);

}

nav_msgs::OccupancyGrid OGridFromLScan(sensor_msgs::LaserScan lmsg, nav_msgs::MapMetaData mapmetad)
{


}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "scantomap");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

 

  ros::Subscriber sub = n.subscribe("scan_data", 10, chatterCallback);

  ros::Publisher chatter_pub = n.advertise<nav_msgs::OccupancyGrid>("map_data", 1, true);

  
 
  ros::spin();



  return 0;
}
