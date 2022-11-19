#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <fstream>
#include <iostream>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <Eigen/Geometry>


class visualCSV {
  private:
    
    public:
    visualCSV(){
      ros::NodeHandle n;
      drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1000);
        vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
        particle_sub = n.subscribe("/pf/pose/odom",1000,&visualCSV::pose_callback,this);

    }


    void pose_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {


    }

    

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "visualCSV_node");
    visualCSV wf;
    ros::spin();
    return 0;
}

#endif