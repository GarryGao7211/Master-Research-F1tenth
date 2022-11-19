#ifndef PACKAGE_PATH_FILE_H
#define PACKAGE_PATH_FILE_H
/////////////////////////////////////////////////
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include <sstream>
#include "std_msgs/Float32.h"
#include "yifei_roslab/scan_range.h"


std_msgs::Float64 ranges_max;
std_msgs::Float64 ranges_min; // max and min for range array sensor values
yifei_roslab::scan_range combine_ranges;

std_msgs::Float64 findMax(sensor_msgs::LaserScan msg) //find maximum range point
{
   std_msgs::Float64 max;
   if(!(msg.ranges.empty()))
   {
      int max_index = 0;
      for(int i = 0;i<msg.ranges.size();i++)
      {
        if(msg.ranges[i] > max.data)
        {
	    max.data = msg.ranges[i];
        }
      }
   
   }
   else
   {
	ROS_INFO("oops for max");

   }

  return max;
}

std_msgs::Float64 findMin(sensor_msgs::LaserScan msg) // find minimum range point 
{

   std_msgs::Float64 min;
   min.data = 999.99;
   if(!(msg.ranges.empty()))
   {
      for(int j = 0;j<msg.ranges.size();j++)
      {
        if(msg.ranges[j] < min.data)
        {
	    min.data = msg.ranges[j];
        }
      }
   }
   else
   {
	ROS_INFO("oops for min");

   }

   return min;
}

void lidarCallback(sensor_msgs::LaserScan msg) // define a callback function
{
 ranges_max = findMax(msg);
 ranges_min = findMin(msg); // extract ranges max and min
 combine_ranges.far_point = ranges_max.data;
 combine_ranges.close_point = ranges_min.data;
 

 ROS_INFO_STREAM("The farthest point is " << ranges_max.data);
 ROS_INFO_STREAM("The closest point is " << ranges_min.data);

 ROS_INFO_STREAM("combined far point is " << combine_ranges.far_point);
 ROS_INFO_STREAM("combined close point is " << combine_ranges.close_point);
 
}


int main (int argc, char ** argv)
{
  ros::init(argc,argv,"lidar_processing"); // initialize ros arguments and node name
  ros::NodeHandle n;//create a node handler


  ros::Subscriber lidar_sub = n.subscribe("scan",1000,lidarCallback); 

  ros::Publisher lidar_closest_pub = n.advertise<std_msgs::Float64>("closest_point",1000);
  ros::Publisher lidar_farthest_pub = n.advertise<std_msgs::Float64>("farthest_point",1000);

  ros::Publisher lidar_combine_pub = n.advertise<yifei_roslab::scan_range>("scan_range",1000); // combined topic

  ros::Rate loopRate(10); // 10Hz loop frequency

  int count = 0;
  while(ros::ok())
  {
    sensor_msgs::LaserScan msg; // create a msg object
    ros::spinOnce();

    lidar_farthest_pub.publish(ranges_max);
    lidar_closest_pub.publish(ranges_min);

    lidar_combine_pub.publish(combine_ranges);
    

    loopRate.sleep();

    ++count;

  }



  
  //ros::spin(); // pump callbacks for each loop

  return 0;

}
#endif

