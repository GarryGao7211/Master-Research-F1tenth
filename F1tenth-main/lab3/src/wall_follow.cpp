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

class WallFollow {
  private:
    // PID control parameters
    double kp;
    double ki;
    double kd;
    double servo_offset;
    double prev_error;
    double error;
    double integral;
    //wall follow parameters
    double ANGLE_RANGE = 270.0; // hokuyo 10LX
    double DESIRED_DISTANCE_RIGHT = 1.10;
    double DESIRED_DISTANCE_LEFT = 0.65;
    double VELOCITY = 2.00;
    double CAR_LENGTH = 0.5; 
    ros::Subscriber lidar_sub;
    ros::Publisher drive_pub;
    ros::Subscriber brake_sub;

    int angle_b_index = 270;//for simulation lidar angle 0 degree index (approximate)
    int angle_a_index = 406; // for simulation lidar angle 45 degree index (approximate)

    double b; //angle distance b
    double a; //angle distance a 
    double predict_constant = 2.0;


    double D_t; //distance to the wall
    double D_t1; //distance one step ahead
    double steering; // steering angle
    double dt = 0.1;

    bool brake_call = false; // check if there is a brake bool cmd published 

    public:
    WallFollow(){
      ros::NodeHandle n;
      lidar_sub = n.subscribe("scan",1000,&WallFollow::lidar_callback,this);
      drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1000); // publish to the drive topic
      brake_sub = n.subscribe("brake_bool",1000,&WallFollow::brake_callback,this);
    }


    void brake_callback(const std_msgs::Bool::ConstPtr& br){
      if(br->data == true)
        brake_call = true;
      else{
        brake_call = false;
      }

    }

    void PID_controller(double error) {
        kp = 1;
        ki = 0.0;
        kd = 0.09;
        
        steering = kp*error + ki*integral + kd*(error-prev_error); // get a steering angle
        ackermann_msgs::AckermannDriveStamped ack_msg; // make a ackermann drive msg
        ack_msg.header.stamp = ros::Time::now();
        ack_msg.header.frame_id = "laser";
        ack_msg.drive.steering_angle = steering;
        double degree = steering *(180.0/M_PI); // convert to actual degrees 
        if(brake_call == false){
          if(abs(degree) > 0.0 && abs(degree) < 10.0) {
            ack_msg.drive.speed = 2.0;//1.5 meters per second
            //ROS_INFO_STREAM("speed 1.5");
          }
          else if(abs(degree) >= 10.0 && abs(degree) < 20.0){
            //ROS_INFO_STREAM("speed 1.0");
            ack_msg.drive.speed = 1.7; //1.0 meters per second
          }
          else {
            //ROS_INFO_STREAM("speed 0.5");
            //ROS_INFO_STREAM(degree);
            ack_msg.drive.speed = 1.2; //
          }
        }
        else{   
          //ROS_INFO_STREAM(brake_call);  
          ack_msg.drive.speed = 0.0;
        }
        // publish the msg to ackerman drive
        drive_pub.publish(ack_msg); // publish the drive message

        integral = integral + error;
        prev_error = error;  
    }



    void lidar_callback (const sensor_msgs::LaserScan::ConstPtr &lidar_msg) {
        b = lidar_msg->ranges[angle_b_index];
        a = lidar_msg->ranges[angle_a_index];
        //ROS_INFO_STREAM("distance a " << a);
        //ROS_INFO_STREAM("distance b " << b);
        double alpha = atan((a*cos(M_PI/4.0)-b)/a*sin(M_PI/4.0)); // in radians 
        //ROS_INFO_STREAM("alpha " << alpha);
        D_t = b*cos(alpha); // update distance to the wal brake ros
        D_t1 = D_t + predict_constant*sin(alpha);
        error = DESIRED_DISTANCE_RIGHT - D_t1;
        PID_controller(error);

    }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "wall_follow_node");
    ros::NodeHandle nc;
    //ros::Rate loopRate(100); // 10Hz
    WallFollow wf;
    ros::spin();
    return 0;
}

#endif