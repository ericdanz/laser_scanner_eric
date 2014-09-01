//oGridMaker Class
//Laser Scanner -> Occupancy Grid

#ifndef OGRIDMAKER_H_
#define OGRIDMAKER_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
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
        oGridMaker()
		{
            ROS_INFO("inside constructor");

            mapper_pub = n.advertise<nav_msgs::OccupancyGrid>("map_eric", 10, true);

            // or scannerCallBack
            sub = n.subscribe("scan", 1, &oGridMaker::scannerCallBack, this);

            //make map meta data
            mmd.map_load_time = ros::Time::now();
            mmd.resolution = .10;
            mmd.width = 50;
            mmd.height = 50;
            mmd.origin.position.x = -2;
	    //createPose();

            mapGrid.info = mmd;
            std::vector<signed char> unkdata (mmd.width*mmd.height,-1);
            mapGrid.data = unkdata;
            mapper_pub.publish(mapGrid);
            ROS_INFO("filling in with unknown");
		}

        geometry_msgs::Pose createPose()
        {
            geometry_msgs::Point thisp;
            thisp.x = -2.0;
            thisp.y = 0.0;
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
            float x;
            float y;
            int ix;
            int iy;

            //angle to this scan, with distance, should determine which tiles it hit on as filled
              //number of
            for (int i=0; i < msg->ranges.size(); i++)
            {
                float thisangle = msg->angle_min + i*msg->angle_increment;
                ROS_INFO("this angle %f", thisangle);
                float thisrange = msg->ranges[i];
                if (thisrange > msg->range_min && thisrange < msg->range_max)
                {
                    ROS_INFO("this range %f", thisrange);
                    x = sin(thisangle)*thisrange *10;
                    y = cos(thisangle)*thisrange *10;

                    x += 10.0;
                    y += 20.0;
                    ix = int(x);
                    iy = int(y);

                    //find the position on the occupancy grid
                    updatePosOccGrid(ix,iy);

                    //downgrade grids between that range and my position
                    //updateNegOccGrid(thisrange,thisangle);
                }
            }
            mapGrid.info = mmd;

		}


        void updatePosOccGrid(int x, int y)
        {
            ROS_INFO("inside pos");
          updateOccGrid(x,y,10, mapGrid.data);
        }

        void updateNegOccGrid(float range, float angle)
        {
            ROS_INFO("inside neg");
           //subtract confidence from grids between that grid and my position

            //get the new range (subtract one grid length from my range based on the angle)
            range -= .1;
            if (range > 0)
            {
                ROS_INFO("this is the range %f",range);
                ROS_INFO("this is the angle %f", angle);
                float x = sin(angle)*range;
                float y = cos(angle)*range;


                int ix = int(x);
                int iy = int(y);

                //this grid is between the hit range and my position, so I think it is empty
                updateOccGrid(ix,iy,-10, mapGrid.data);

                //recursively call until I get to my position
                if (ix > mmd.origin.position.x && iy > mmd.origin.position.y)
                {
                    updateNegOccGrid(range,angle);
                }
            }

        }

        void updateOccGrid(int x, int y, int change, std::vector<signed char> thisdata)
        {
           //add confidence to grid that range hit in
           int grid = x*100 + y;
           thisdata[grid]+= change;
           ROS_INFO("updating map grids %d", grid);
           ROS_INFO("this is the val %d", thisdata[grid]);
           if (thisdata[grid] > 100)
           {
               thisdata[grid] = 100;
           }
           else if (thisdata[grid] < 0)
           {
               thisdata[grid] = 0;
           }
           mapGrid.data = thisdata;
        }

	
        void scannerCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
		{
            //ROS_INFO("Inside Callback");
            OGridFromLScan(msg);
            mapper_pub.publish(mapGrid);
		}


};

#endif