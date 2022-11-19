#ifndef PACKAGE_PATH_FILE_H
#define PACKAGE_PATH_FILE_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
// TODO: include ROS msg type headers and libraries
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <algorithm>
class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double close_point;
    double speed;
    double threshhold = 0.4; // threshhold for collision to apply TTC
    double time_to_collision;
    int index; // index for closed beam 
    double ang_increment;
    double ang_min; // angle min for lidar
    double ang_max; //angle max for lidar
    // TODO: create ROS subscribers and publishers
    ros::Subscriber odom_sub;
    ros::Subscriber scan_sub;
    ros::Publisher brake_bool_pub;
    ros::Publisher brake_pub;

    
public:
    Safety() {
        n = ros::NodeHandle();
        speed = 0.0;
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages
        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        */

        // TODO: create ROS subscribers and publishers
        odom_sub = n.subscribe("odom",1000,&Safety::odom_callback,this);
        scan_sub = n.subscribe("scan",1000,&Safety::scan_callback,this);
        brake_bool_pub = n.advertise<std_msgs::Bool>("brake_bool",1000);
        brake_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("brake",1000);
        
    }

     double findMin(const sensor_msgs::LaserScan::ConstPtr &msg) // find minimum range point 
     {

       std_msgs::Float64 min;
       min.data = 999.99;
       if(!(msg->ranges.empty()))
       {
          for(int j = 0;j<msg->ranges.size();j++)
          {
            if(msg->ranges[j] < min.data)
            {
	        min.data = msg->ranges[j];
                index = j;
            }
          }
       }
       else
       {
	    ROS_INFO("oops for min");

       }

       return min.data;
     }
     
     void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed
        speed = odom_msg->twist.twist.linear.x; // obtain forward linear velocity
        //ROS_INFO_STREAM("the forward speed is " << speed);
    }

     void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // TODO: calculate TTC
        close_point = findMin(scan_msg); // obtain the closest range
        ang_increment = scan_msg->angle_increment;  
        ang_min = scan_msg->angle_min;
        ang_max = scan_msg->angle_max;
        double epo = 0.0001;
        time_to_collision = close_point/std::max((-1)*speed*cos(index*ang_increment),epo); // calculate TTC
        
        
        
          //ROS_INFO_STREAM("size " << scan_msg->ranges.size());
          //ROS_INFO_STREAM("closet point meters " << close_point);
          //ROS_INFO_STREAM("close point index " << index);
          //(index*ang_increment)/M_PI*180
          //ROS_INFO_STREAM("test calculated range rate " << speed*cos(index*ang_increment));
          //ROS_INFO_STREAM("time to collision " << time_to_collision);

        // TODO: publish drive/brake message
        

        if(time_to_collision < threshhold)
        {
          //double brake_speed = 0.0;
          bool brake_cmd = true;

	  ackermann_msgs::AckermannDriveStamped brake_msg;
          std_msgs::Bool true_brake;

          brake_msg.drive.speed = 0.0;
          brake_msg.drive.acceleration = 0.0;
          true_brake.data = brake_cmd; // std_msgs/Bool = true
          
          
          //publish brake and true brake cmd
          
          brake_pub.publish(brake_msg);
          brake_bool_pub.publish(true_brake);
        }
        
        
    }


     

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}

#endif
