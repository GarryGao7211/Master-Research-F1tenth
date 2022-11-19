#!/usr/bin/env python
from cmath import cos, pi
#from sympy import Point
#import pytorch as torch
import rospy
import csv
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
import cvxpy as cp
import numpy as np
from nav_msgs.msg import Odometry,OccupancyGrid
from ackermann_msgs.msg import AckermannDrive
from collections import deque
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
import math
import track
#from tf.transformations import euler_from_quaternion



class high_level_plan(object):

    def __init__(self):



        ## define all publisher and subscriber
        self.center_rviz_pub = rospy.Publisher("centerline",visualization_msgs.msg.MarkerArray,queue_size=2)
        self.trajectories_rviz_pub = rospy.Publisher("trajectories",visualization_msgs.msg.MarkerArray,queue_size=2)
        self.max_rviz_pub = rospy.Publisher("max_progress_point",visualization_msgs.msg.MarkerArray,queue_size=2)
        #rospy.Subscriber("odom",Odometry,self.odom_callback,queue_size=1)

        #rospy.Subscriber("odom",Odometry,self.select_trajectory)

        ## point and track initialization
        self.track_ = track.Track()
        
        #print(self.track_.centerline.shape)

        ## define all parameters
        self.car_pos_x = 0.0
        self.car_pos_y = 0.0
        self.yaw_ = 0.0
        self.p_d = np.array([-45*pi/180,-20*pi/180,0,20*pi/180,45*pi/180])
        self.N = 20
        self.Ts = 0.04
        self.num_direction = 5
        self.num_states = 3
        self.V = 1.5
        self.state = np.empty((3,))
        self.new_state = np.empty((3,))
        self.trajectory_table = np.empty((5,3,self.N+1))
        self.max_index = 0
        self.map_ = OccupancyGrid()
        self.map_update = OccupancyGrid()
        self.non_collision_index = []
        #self.traj = np.empty((3,21))


        self.threshold = 50
        self.MAP_MARGIN = 0.19
        self.init_occupancy_grid()

        ## trajectory picked
        #self.trajectory_ref = []
        

        self.compute_trajectory_table()
    
    
    ## --------------------------------------------------------> all visualization (define here)
    def visualize_centerline(self):
        # plot waypoints
        center_marker = visualization_msgs.msg.Marker()
        center_marker.header.frame_id = 'map'
        center_marker.ns = 'center_line_points'
        center_marker.type = center_marker.POINTS
        center_marker.scale.x = 0.05
        center_marker.scale.y = 0.05
        center_marker.scale.z = 0.05
        center_marker.action = center_marker.ADD
        center_marker.pose.orientation.w = 1.0
        center_marker.color.r = 1.0
        center_marker.color.a = 1.0
        
        range_field = 10
        steps = 8
        
        # append all points to marker type (should be changed later)
        for i in range(len(self.track_.centerline)):
            p = Point()
            p.x = float(self.track_.centerline[i].x)
            p.y = float(self.track_.centerline[i].y)
            center_marker.points.append(p)

        #print(len(center_marker.points))
        # create new marker type for line stripe
        markers = visualization_msgs.msg.MarkerArray()
        center_marker_line = visualization_msgs.msg.Marker()
        center_marker_line.header.frame_id = 'map'
        center_marker_line.ns = 'center_line'
        center_marker_line.type = center_marker_line.LINE_STRIP
        center_marker_line.scale.x = 0.02
        center_marker_line.scale.y = 0.02
        center_marker_line.scale.z = 0.02
        center_marker_line.action = center_marker.ADD
        center_marker_line.pose.orientation.w = 1.0
        center_marker_line.color.b = 1.0
        center_marker_line.color.a = 1.0

        center_marker_line.points = center_marker.points
        markers = visualization_msgs.msg.MarkerArray()
        markers.markers.append(center_marker)
        markers.markers.append(center_marker_line)
        
        
        # publish to rviz markers topic
        markers.markers.append(center_marker)
        self.center_rviz_pub.publish(markers)

    ## obtain RPY
    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
    """
    def odom_callback(self,odom_msg):
        
        self.visualize_centerline()
        self.visualize_trajectory()

        ## obtain position x y and yaw
        self.car_pos_x= odom_msg.pose.pose.position.x
        self.car_pos_y = odom_msg.pose.pose.position.y

        #print("car_pos_x in callback")
        #print(self.car_pos_x)
        #print("car_pos_y in callback")
        #print(self.car_pos_y)

        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        roll_x,pitch_y,yaw_z = self.euler_from_quaternion(qx,qy,qz,qw) 
        self.yaw_ = yaw_z


        #print("yaw_angle")
        #print(self.yaw_)

        ## select the best trajectory
        self.select_trajectory()
        #print("!!!!!!!")
        #print("here")
        ## visualize the best trajectory
        self.visualize_max_trajectory()
    """
        

    
    ## -------------------------------------------------> Occupancy grid (define here)
    def xy_ind2ind(self,grid,x_ind,y_ind):
        #return max(0, min(y_ind * int(grid.info.width) + x_ind, int(grid.data.size())-1))
        
        
        return max(0, min(y_ind * int(grid.info.width) + x_ind, int(len(grid.data))-1))

    def xy2_ind(self,grid,x,y):
        #print("origin x")
        #print(grid.info.origin.position.x)
        #print("position x")
        #print(x)
        x_ind = int(math.ceil((x-grid.info.origin.position.x)/0.05)) - 1   
        y_ind = int(math.ceil((y-grid.info.origin.position.y)/0.05)) - 1

        #print("x_ind")
        #print(x_ind)
        #print("y_ind")
        #print(y_ind)


        return self.xy_ind2ind(grid, x_ind, y_ind)

    # convert index to xy coordinate 
    def ind2xy_ind(self,grid,ind):
        y_ind = int(ind/grid.info.width)
        x_ind = int(ind - y_ind*grid.info.width)
        pair_res = (x_ind,y_ind)
        return pair_res

    def is_xy_occupied(self,grid,x,y):
        #print("list index")
        #print(self.xy2_ind(grid,x,y))
        #print("grid data length")
        #print(len(grid.data))
        #print("value")
        #print(int(grid.data[self.xy2_ind(grid,x,y)]))

        return int(grid.data[self.xy2_ind(grid,x,y)]) > self.threshold

    def inflate_cell(self,grid,ind,margin,val):
        #print(grid.info.resolution)
        margin_cells = int(math.ceil(margin/grid.info.resolution))
        pair_res = self.ind2xy_ind(grid,ind)
        
        grid_data_list = list(grid.data)
        for x in range(max(0, pair_res[0]-margin_cells),min(int(grid.info.width-1), pair_res[0]+margin_cells),1):
            for y in range(max(0, pair_res[1]-margin_cells),min(int(grid.info.height-1), pair_res[1]+margin_cells),1):
                ## change to list and convert back to tuple
                grid_data_list[self.xy_ind2ind(grid,x,y)] = val
        
        grid.data = grid_data_list


    def inflate_map(self,grid,margin):
        occupy_ind = []
        # store occupied index
        for i in range(len(grid.data)):
            if grid.data[i] > self.threshold:
                occupy_ind = np.append(occupy_ind,i)
        for i in range(len(occupy_ind)):
            self.inflate_cell(grid,i,margin,100)


    def init_occupancy_grid(self):
        self.map_ = rospy.wait_for_message("map",OccupancyGrid,timeout = 5)
        #print("resolution")
        #print(self.map_.info.resolution)
        if self.map_ is None:
            print("No map received")
        else:
            self.map_update = self.map_
            print("map received!")
        
        print("Initializing occupancy grid map.....")

        #print(self.map_update.info.height)
        #print(self.map_update.info.width)

        ## inflate map
        #self.inflate_map(self.map_,self.MAP_MARGIN)
        #print("!!!")
        

        
        #print(map_received)
    
    ## --------------------------------------------------> dynamics modeling function and linearization (define here)
    def simulate_dynamics(self,state,input_,dt):
        # nonlinear dynamics
        dynamics = np.zeros((3,))
        dynamics[0] = input_[0] * math.cos(state[2])
        dynamics[1] = input_[0] * math.sin(state[2])
        dynamics[2] = math.tan(input_[1]) * input_[0] / 0.35
        new_state = state + dynamics * dt
        return new_state

    def compute_trajectory_table(self):
        K = 50
        dt = 0.04
        input_ = np.empty((2,))

        # simulate all possible trajectories for 5 directions
        for i in range(self.num_direction):
            speed = self.V
            steer = self.p_d[i]
            # constant input for dynamics
            input_[0] = speed
            input_[1] = steer
            # add initial state
            state = np.zeros((3,))
            trajectory = state
            # for one direction
            for j in range(self.N):
                new_state = self.simulate_dynamics(state,input_,dt)
                trajectory = np.append(trajectory,new_state)
                state = new_state

            ## 3*21 trajectory shape
            trajectory = np.reshape(trajectory,(self.N+1,3)).transpose()
            # store into trajectory table
            self.trajectory_table[i,:,:] = trajectory


    def visualize_trajectory(self):
        traj_list = visualization_msgs.msg.MarkerArray()
        traj_points0 = visualization_msgs.msg.Marker()
        traj_points0.header.frame_id = "base_link"
        traj_points0.type = visualization_msgs.msg.Marker.LINE_STRIP
        traj_points0.ns = "trajectory_points0"
        traj_points0.scale.x = 0.04
        traj_points0.scale.y = 0.04
        traj_points0.scale.z = 0.04
        traj_points0.action = visualization_msgs.msg.Marker.ADD
        traj_points0.pose.orientation.w = 1.0
        traj_points0.color.r = 1.0
        traj_points0.color.a = 1.0


        for j in range(self.N+1):
            p = Point()
            p.x = self.trajectory_table[0][0][j]
            p.y = self.trajectory_table[0][1][j]
            traj_points0.points.append(p)

        traj_list.markers.append(traj_points0)
        
        ## -20pi/180
        traj_points1 = visualization_msgs.msg.Marker()
        traj_points1.header.frame_id = "base_link"
        traj_points1.type = visualization_msgs.msg.Marker.LINE_STRIP
        traj_points1.ns = "trajectory_points1"
        traj_points1.scale.x = 0.04
        traj_points1.scale.y = 0.04
        traj_points1.scale.z = 0.04
        traj_points1.action = visualization_msgs.msg.Marker.ADD
        traj_points1.pose.orientation.w = 1.0
        traj_points1.color.r = 1.0
        traj_points1.color.a = 1.0

        for j in range(self.N+1):
            p = Point()
            p.x = self.trajectory_table[1][0][j]
            p.y = self.trajectory_table[1][1][j]
            traj_points1.points.append(p)

        traj_list.markers.append(traj_points1)

        ## 0
        traj_points2 = visualization_msgs.msg.Marker()
        traj_points2.header.frame_id = "base_link"
        traj_points2.type = visualization_msgs.msg.Marker.LINE_STRIP
        traj_points2.ns = "trajectory_points2"
        traj_points2.scale.x = 0.04
        traj_points2.scale.y = 0.04
        traj_points2.scale.z = 0.04
        traj_points2.action = visualization_msgs.msg.Marker.ADD
        traj_points2.pose.orientation.w = 1.0
        traj_points2.color.r = 1.0
        traj_points2.color.a = 1.0

        for j in range(self.N+1):
            p = Point()
            p.x = self.trajectory_table[2][0][j]
            p.y = self.trajectory_table[2][1][j]
            traj_points2.points.append(p)

        traj_list.markers.append(traj_points2)

        ## 20pi/180
        traj_points3 = visualization_msgs.msg.Marker()
        traj_points3.header.frame_id = "base_link"
        traj_points3.type = visualization_msgs.msg.Marker.LINE_STRIP
        traj_points3.ns = "trajectory_points3"
        traj_points3.scale.x = 0.04
        traj_points3.scale.y = 0.04
        traj_points3.scale.z = 0.04
        traj_points3.action = visualization_msgs.msg.Marker.ADD
        traj_points3.pose.orientation.w = 1.0
        traj_points3.color.r = 1.0
        traj_points3.color.a = 1.0

        for j in range(self.N+1):
            p = Point()
            p.x = self.trajectory_table[3][0][j]
            p.y = self.trajectory_table[3][1][j]
            traj_points3.points.append(p)

        traj_list.markers.append(traj_points3)

        ## 45pi/180
        traj_points4 = visualization_msgs.msg.Marker()
        traj_points4.header.frame_id = "base_link"
        traj_points4.type = visualization_msgs.msg.Marker.LINE_STRIP
        traj_points4.ns = "trajectory_points4"
        traj_points4.scale.x = 0.04
        traj_points4.scale.y = 0.04
        traj_points4.scale.z = 0.04
        traj_points4.action = visualization_msgs.msg.Marker.ADD
        traj_points4.pose.orientation.w = 1.0
        traj_points4.color.r = 1.0
        traj_points4.color.a = 1.0

        for j in range(self.N+1):
            p = Point()
            p.x = self.trajectory_table[4][0][j]
            p.y = self.trajectory_table[4][1][j]
            traj_points4.points.append(p)

        traj_list.markers.append(traj_points4)

        self.trajectories_rviz_pub.publish(traj_list)

    
    ## ----------------------------------------------------------------------->select best trajectory which maximizes the progress
    def select_trajectory(self):
        max_theta = 0.0
        #car_pos_x = odom_msg.pose.pose.position.x
        #car_pos_y = odom_msg.pose.pose.position.y
        car_theta = self.track_.findTheta(self.car_pos_x,self.car_pos_y,0,True)
        #print("car_theta")
        #print(car_theta)
        #pos_in_map_x = 0.0
        #pos_in_map_y = 0.0

        traj = np.empty((3,21))
        non_collision_arr = []
        yaw = self.yaw_

        for i in range(self.num_direction):
            occupied = False
            for k in range(self.N+1):
                # conversion from base_link to map
                traj[0][k] = math.cos(yaw)*self.trajectory_table[i][0][k] - math.sin(yaw)*self.trajectory_table[i][1][k] + self.car_pos_x
                traj[1][k] = math.sin(yaw)*self.trajectory_table[i][0][k] + math.cos(yaw)*self.trajectory_table[i][1][k] + self.car_pos_y
                traj[2][k] = self.trajectory_table[i][2][k] + yaw
                    
                if self.is_xy_occupied(self.map_,traj[0][k],traj[1][k]):
                    occupied = True
                    break
            
            if occupied:           
                continue

            else:
                ## store noncollision index
                if i == 1 or i == 2 or i == 3:
                    non_collision_arr = np.append(non_collision_arr,i)

                theta = self.track_.findTheta(traj[0][self.N],traj[1][self.N],0,True)
            
                if theta - car_theta < -self.track_.length/2:
                    theta += self.track_.length
                if theta > max_theta:
                    max_theta = theta
                    self.max_index = i

        self.non_collision_index = non_collision_arr

        #print("non_collision_index:")
        #print(self.non_collision_index)

        #print("theta")
        #print(theta)
            
        #print("max_theta")
        #print(max_theta)
        #print("max index trajectory")
        #print(self.max_index)
        
        
    
    def visualize_max_trajectory(self):
        max_point = visualization_msgs.msg.Marker()
        max_point.header.frame_id = "base_link"
        max_point.type = visualization_msgs.msg.Marker.POINTS
        max_point.ns = "max_end_point"
        max_point.scale.x = 0.1
        max_point.scale.y = 0.1
        max_point.scale.z = 0.1
        max_point.action = visualization_msgs.msg.Marker.ADD
        max_point.pose.orientation.w = 1.0
        max_point.color.g = 1.0
        max_point.color.a = 1.0

        p = Point()
        ## need to be changed -----------------------------------------------
        p.x = self.trajectory_table[self.max_index][0][self.N]
        p.y = self.trajectory_table[self.max_index][1][self.N]

        max_point.points.append(p)
        max_point_list = visualization_msgs.msg.MarkerArray()
        max_point_list.markers.append(max_point)
        
        self.max_rviz_pub.publish(max_point_list)



        

"""
def main():
    rospy.init_node('high_level_plan',anonymous=True)
    pp = high_level_plan()
    
    #rospy.sleep(1000)
    rospy.spin()
if __name__ == '__main__':
    main()

"""