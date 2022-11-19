#ifndef PACKAGE_PATH_FILE_H
#define PACKAGE_PATH_FILE_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <algorithm>

class reactive_follow_gap {

private:
ros::Subscriber lidar_sub;
ros::Publisher drive_pub;

double pre_range[1080]; // give preprocess ranges array
double CAR_LENGTH = 0.5;
double close_point;
int close_point_index;
double width = 0.25;
double angle_incre = 0.005823;

int start_index;
int end_index; // index stored for max gap

const int start_scope = 360;
const int end_scope = 720;

public:
    reactive_follow_gap(){

        ros::NodeHandle n;
        lidar_sub = n.subscribe("scan",1000,&reactive_follow_gap::lidar_callback,this);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1000);
    }

    void preprocess_lidar(double array[]){
        //obtain a range of -1/3pi - 1/3pi
        int first_half = 360;
        for(int i = 0; i < first_half;i++){
            array[i] = 0.0;
        } 
        int second_half = 721;
        int size = 1080;
        for(int j = second_half; j < size;j++){
            array[j] = 0.0;
        }
        //cliping all values over 3m to 3m
        for(int k = first_half; k < second_half;k++){
            if(array[k] > 3.0){
                array[k] = 3.0;
            }
        }
    }

     void findMin(double arr[]){
         int index_start = 360;
         int index_end = 720;
         int min_index;

         close_point = 999.9;
         //find close_point and its index
         for(int i = index_start; i < index_end + 1; i++){
             if(arr[i] < close_point){
                 close_point = arr[i];
                 min_index = i;
             }
         }
         close_point_index = min_index; // extract close_point index
     }

     void find_max_gap(double arr[]){ 
        int k = 5; // gap length
        double max_sum = 0.0;
        //get sum of first element
        start_index = start_scope;
        //ROS_INFO_STREAM("scope: " << start_index);
        end_index = start_scope + k - 1; //first sliding window index 

         for(int i = start_scope;i < start_scope + k;i++){
             max_sum+=arr[i];
         }
         //ROS_INFO_STREAM("max sum: " << max_sum);
         
         double window_sum = max_sum;
         for(int j = start_scope + k;j < end_scope + 1;j++){
             window_sum += arr[j] - arr[j - k];
             if(window_sum > max_sum){
                 max_sum = window_sum;
                 //update start and end index
                 start_index = j - k + 1;
                 //end_index = j;
             }
         }
         end_index = start_index + k;
         //ROS_INFO_STREAM("start: " << start_index);
         

     }

///////////////////////////////////////////////////////////     
    // return best point of the gap   
     int find_best_point(double pre_range[], int start, int end){
         double max_index;
         double max;
         for(int i = start;i < end + 1;i++){
            if(pre_range[i] > max){
                max = pre_range[i];
                max_index = i;
            }  
         }
         return max_index;
     }

    void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& lidar_msg){
        for(int i = 0; i<lidar_msg->ranges.size(); i++){
            pre_range[i] = lidar_msg->ranges[i]; // copy array
        }
        //ROS_INFO_STREAM("0 " << pre_range[0]);
        //ROS_INFO_STREAM(pre_range[540]);
        //ROS_INFO_STREAM("-1/3 pi" << pre_range[361]);
        //ROS_INFO_STREAM("1/3 pi" << pre_range[720]);

        preprocess_lidar(pre_range); // filter ranges larger than 3.0m and range scope from -1/3pi ~ 1/3pi
        /*ROS_INFO_STREAM("0 degree: " << pre_range[100]);
        ROS_INFO_STREAM(pre_range[540]);
        ROS_INFO_STREAM("-1/3 pi: " << pre_range[360]);
        ROS_INFO_STREAM("1/3 pi: " << pre_range[700]);*/


        findMin(pre_range); // store close point and its index in private number
        //ROS_INFO_STREAM("close index: " << close_point_index);
        //ROS_INFO_STREAM(close_point);
        // eliminate all points to zero inside the safety bubble
        double safe_r = 0.7;
        //calculate half radius angle 
        double angle = atan(safe_r/close_point); // extract half angle
        int num = ceil(angle/angle_incre);
        //ROS_INFO_STREAM("num: " << num);


        //set safety bubble around close point
        for(int i = close_point_index - num; i < close_point_index;i++){
            pre_range[i] = 0.0;
        }
        for(int j = close_point_index; j < close_point_index + num; j++){
            pre_range[j] = 0.0;
        }

        //ROS_INFO_STREAM("400: " << pre_range[400]);
        //ROS_INFO_STREAM("550: " << pre_range[550]);

        find_max_gap(pre_range); // update index start and end

        //ROS_INFO_STREAM("start " << start_index);
        //ROS_INFO_STREAM("end " << end_index);
        int best_index = find_best_point(pre_range,start_index,end_index);
        //ROS_INFO_STREAM("best index " << best_index);
        double test_num = best_index * angle_incre;

        //publish drive message
        ackermann_msgs::AckermannDriveStamped drive_msg;
        double steering = -3.14159 + test_num;
        //ROS_INFO_STREAM("steering angle " << steering);
        drive_msg.drive.steering_angle = steering;
        drive_msg.drive.speed = 2.2;
        drive_pub.publish(drive_msg);
    }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "wall_follow_node");
    reactive_follow_gap rf;
    ros::spin();
    return 0;
}

#endif