#include "ros/ros.h"
#include "std_msgs/Time.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include <sstream>
#include "math.h"
#include "stdio.h"



class oGridMaker {
	private: 
		ros::NodeHandle n;
		ros::Publisher mapper_pub;
		ros::Subscriber sub;
		nav_msgs::MapMetaData mmd;
		nav_msgs::OccupancyGrid mapGrid;
	public:
		oGridMaker(ros::NodeHandle &nh)
		{
            ROS_INFO("inside constructor");
            n = nh;
            mapper_pub = n.advertise<nav_msgs::OccupancyGrid>("map_eric", 10, true);

            // or scannerCallBack
            sub = n.subscribe("scan", 10, &oGridMaker::scannerCallBack, this);

            //make map meta data
            mmd.map_load_time = ros::Time::now();
            mmd.resolution = 1.0;
            mmd.width = 10;
            mmd.height = 5;
            mmd.origin = createPose();

            mapGrid.info = mmd;
            std::vector<signed char> unkdata (50,-1);
            mapGrid.data = unkdata;
            ROS_INFO("filling in with unknown");

		}

        geometry_msgs::Pose createPose()
        {
            geometry_msgs::Point thisp;
            thisp.x = -1.0;
            thisp.y = -5.0;
            thisp.z = 0.0;

            geometry_msgs::Quaternion thisq;
            thisq.x = 0.0;
            thisq.y = 0.0;
            thisq.z = 0.0;
            thisq.w = 0.0;

            geometry_msgs::Pose thispose;
            thispose.position = thisp;
            thispose.orientation = thisq;
            ROS_INFO("created a pose");
            return thispose;
        }

		//this is where the work gets done
        void OGridFromLScan(const sensor_msgs::LaserScan::ConstPtr& msg)
		{	
            //ROS_INFO("inside gfroms");
            //update time
            mmd.map_load_time = msg->header.stamp;



            //angle to this scan, with distance, should determine which tiles it hit on as filled
              //number of
            for (int i=0; i < msg->ranges.size(); i++)
            {
                float thisangle = msg->angle_min + i*msg->angle_increment;
                float thisrange = msg->ranges[i];
                float x = sin(thisangle)*thisrange;
                float y = cos(thisangle)*thisrange;

                x = x + 1.0;
                y = y + 5.0;
                int ix = int(x);
                int iy = int(y);

                //find the position on the occupancy grid
                updatePosOccGrid(ix,iy);

                //downgrade grids between that range and my position
                updateNegOccGrid(thisrange,thisangle);
            }
            mapGrid.info = mmd;

		}


        void updatePosOccGrid(int x, int y)
        {
            ROS_INFO("inside pos");
          updateOccGrid(x,y,1);
        }

        void updateNegOccGrid(float range, float angle)
        {
            ROS_INFO("inside neg");
           //subtract confidence from grids between that grid and my position

            //get the new range (subtract one grid length from my range based on the angle)
            float subd = 1.0 / sin(angle);
            range -= subd;

            float x = sin(angle)*range;
            float y = cos(angle)*range;

            x = x + 1.0;
            y = y + 5.0;
            int ix = int(x);
            int iy = int(y);

            //this grid is between the hit range and my position, so I think it is empty
            updateOccGrid(ix,iy,-1);

            //recursively call until I get to my position
            if (ix > 1 && iy > 5)
            {
                updateNegOccGrid(range,angle);
            }

        }

        void updateOccGrid(int x, int y, int change)
        {
           //add confidence to grid that range hit in
           int grid = x*10 + y;
           mapGrid.data[grid] += change;
           ROS_INFO("updating map grids");
           if (mapGrid.data[grid] > 100)
           {
               mapGrid.data[grid] = 100;
           }
           else if (mapGrid.data[grid] < 0)
           {
               mapGrid.data[grid] = 0;
           }
        }

	
        void scannerCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
		{

            //ROS_INFO("Inside Callback");

            OGridFromLScan(msg);


	  		mapper_pub.publish(mapGrid);

		}


};



int main(int argc, char **argv)
{
  
  	ros::init(argc, argv, "scantomap");
  
  	ros::NodeHandle nh;

    oGridMaker ogm = oGridMaker(nh);

  	//ogmaker.initialize(nh);

  	//ros::Publisher chatter_pub = n.advertise<nav_msgs::OccupancyGrid>("map_data", 1, true);


  	//ros::Subscriber sub = n.subscribe("scan_data", 10, chatterCallback);

 
  	ros::spin();



  return 0;
}
