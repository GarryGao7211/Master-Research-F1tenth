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


// TODO: include ROS msg type headers and libraries you need

class PurePursuit {
private:
    ros::NodeHandle n;
    // TODO: create ROS subscribers and publishers
    ros::Publisher vis_pub;
    ros::Publisher drive_pub;
    ros::Subscriber particle_sub;
    // parameters
    double lookhead;
    double robot_x;
    double robot_y;
    double robot_heading;
    std::vector<double> waypoint_x;
    std::vector<double> waypoint_y;
    std::vector<double> waypoint_angle;
    std::vector<double> waypoint_speed;
    
    

public:
    PurePursuit() {
        n = ros::NodeHandle();
        // TODO: create ROS subscribers and publishers
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1000);
        vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0);
        particle_sub = n.subscribe("/pf/pose/odom",1000,&PurePursuit::pose_callback,this);

        // read csv file data
        std::ifstream myfile;
        myfile.open("/home/gary/Desktop/data_file.csv",std::ios::in);
        //myfile.open("/home/gary/Desktop/test_data.csv",std::ios::in);
        std::vector<std::string> row;
        std::string line,word,temp;
        if(!myfile.is_open()){
             ROS_INFO_STREAM("cannot open file");
        }
        std::string c;
        double num;
        int count = 0;
        while(std::getline(myfile, line, ',')){
            // get x coordinate
            num  = std::stod(line);
            waypoint_x.push_back(num);
            //ROS_INFO_STREAM(num);
             
            std::getline(myfile,line,',');
            num = std::stod(line);
            waypoint_y.push_back(num);
            //ROS_INFO_STREAM(num);
         
            std::getline(myfile,line,',');
            num = std::stod(line);
            waypoint_angle.push_back(num);
            //ROS_INFO_STREAM(num);

            std::getline(myfile,line,'\n');
            num = std::stod(line);
            waypoint_speed.push_back(num);
            //ROS_INFO_STREAM(num);
            
            //if(count == 1){
            //    break;
            //}
            count++;
        }



    }

    //void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
      void pose_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) { 
        // TODO: find the current waypoint to track using methods mentioned in lecture

        tf::Quaternion q(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        robot_heading = yaw;
        //ROS_INFO_STREAM("robot heading: " << yaw);

        double current_x;
        double current_y;
        double current_angle;
        double robot_angle;
        lookhead = 1.0;
        //extract current robot position using particle filter
        robot_x = odom_msg->pose.pose.position.x;
        robot_y = odom_msg->pose.pose.position.y;
        //robot_angle = atan2(robot_y,robot_x);
    

        // current goal point to steering in world frame
        double goal_x;
        double goal_y;
        // current goal point to steering in robot frame
        double goal_x_R;
        double goal_y_R;

        
        for(int i = 0; i < waypoint_x.size();i++){
          // extract one coordinate
          current_x = waypoint_x[i];
          current_y = waypoint_y[i];

          double d = sqrt(pow(current_x-robot_x,2.0)+pow(current_y-robot_y,2.0));

          // first distance check
          if(d > lookhead && d < (lookhead + 0.1)){
            // transform goal point candidate to robot frame of reference

            //ROS_INFO_STREAM("currentx at beginning: " << current_x);
            //ROS_INFO_STREAM("currenty at beginning: " << current_y);

            Eigen::Matrix3d m_B2W;
            m_B2W << cos(yaw),-sin(yaw),0,sin(yaw),cos(yaw),0,0,0,1;

            //ROS_INFO_STREAM("m_B2W: " << m_B2W);

            //ROS_INFO_STREAM("robot heading: " << yaw);
        
            Eigen::Matrix3d m_W2B;
            // transform 
            m_W2B = m_B2W.transpose();

            // obtain robot global coordinates
            Eigen:: Vector3d robot;
            robot[0] = robot_x;
            robot[1] = robot_y;
            robot[2] = 1.0;

            // obtain 3d translation in homogenous form
            Eigen:: Vector3d translation;
            translation = m_W2B * robot;
        
            //obtain H2B rotation matrix in homo form
            Eigen:: Matrix3d m_H2B;
            m_H2B << 1.0, 0.0, translation[0], 0.0, 1.0, translation[1], 0.0, 0.0, 1.0;

            Eigen:: Matrix3d m_W2H;

            m_W2H = (m_B2W* m_H2B).inverse();

            // calculate current waypoint of interest with respect to robot frame
            Eigen:: Vector3d goal_W;
            goal_W[0] = current_x;
            goal_W[1] = current_y;
            goal_W[2] = 1.0;

            Eigen:: Vector3d goal_R;
            goal_R = m_W2H * goal_W;

            double goal_robotx = goal_R[0];
            double goal_roboty = goal_R[1];

            //ROS_INFO_STREAM("goal_robotx: " << goal_robotx);
            //ROS_INFO_STREAM("goal_roboty: " << goal_roboty);

            double angle_in_R = atan2(goal_roboty,goal_robotx);
            //ROS_INFO_STREAM("angle_in_R: " << angle_in_R);

            if(abs(angle_in_R) < (M_PI/2)){
                //store right waypoint both world frame and robot frame
                goal_x_R = goal_robotx;
                goal_y_R = goal_roboty;
                goal_x = current_x;
                goal_y = current_y;
                //ROS_INFO_STREAM("currentx at end: " << current_x);
                //ROS_INFO_STREAM("currenty at end: " << current_y);

                //ROS_INFO_STREAM("goal_x_R: " << goal_robotx);
                //ROS_INFO_STREAM("goal_y_R: " << goal_roboty);
                //ROS_INFO_STREAM("goalx: " << goal_x);
                //ROS_INFO_STREAM("goaly: " << goal_y);
                break;
            }
          }
        }
        //ROS_INFO_STREAM("goal_x: " << goal_x);
        //ROS_INFO_STREAM("goal_y: " << goal_y);
        //ROS_INFO_STREAM("robot_x: " << robot_x);
        //ROS_INFO_STREAM("robot_y: " << robot_y);

        
       //ROS_INFO_STREAM("currentx at end: " << current_x);
       //ROS_INFO_STREAM("currenty at end: " << current_y);
       
        
        vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;

        marker.pose.position.x = goal_x;
        marker.pose.position.y = goal_y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;  
        vis_pub.publish(marker);
        

        // TODO: transform goal point to vehicle frame of reference
 
        // TODO: calculate curvature/steering angle
        if(abs(goal_y_R) < 0.1){
            goal_y_R = 0.0;
        }

        double steering = 0.8*2*goal_y_R/(pow(lookhead,2.0));

        ROS_INFO_STREAM("goal_x_R: " << goal_x_R);
        ROS_INFO_STREAM("goal_y_R: " << goal_y_R);
        

        if(steering <= -0.3189){
            steering = -0.3189;

        }
        if(steering >= 0.3189){
            steering = 0.3189;
        }
        ROS_INFO_STREAM("steering: " << steering);
        

        // TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.drive.steering_angle = steering;
        drive_msg.drive.speed = 1.7;
        drive_pub.publish(drive_msg); 
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}