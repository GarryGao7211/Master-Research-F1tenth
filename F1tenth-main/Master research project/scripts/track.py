# track package for determining theta

#from tkinter import Y
import rospy
import csv
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped
import math


class Point_ref(object):
    x:float
    y:float
    theta:float
    left_half_width:float
    right_half_width:float

    
class Track(object):
    def __init__(self):

        ## track parameters
        self.center_rows = []
        self.centerline = []
        self.waypoints = []
        self.length = 0.0
        self.space = 0.4
        self.HALF_WIDTH_MAX = 0.8
        self.SEARCH_RANGE = 10
        

        ## extract data file
        file = open('/home/gary/Desktop/data_file.csv')
        csvreader = csv.reader(file)
        header = next(csvreader)
        for row in csvreader:
            self.center_rows.append(row)
        self.center_rows = np.array(self.center_rows)

        for i in range(len(self.center_rows)):
            wp = Point()
            wp.x = float(self.center_rows[i][0])
            wp.y = float(self.center_rows[i][1])
            self.waypoints.append(wp)

        ## extract equally spaced points
        curr_ = 0
        next_ = 1
        p_start = Point_ref()
        p_start.x = self.waypoints[0].x
        p_start.y = self.waypoints[0].y
        p_start.theta = 0.0
        self.centerline.append(p_start)
        theta = 0.0


        for _ in range(1,len(self.waypoints)):
            dist = ((self.waypoints[next_].x - self.waypoints[curr_].x)**2 + (self.waypoints[next_].y - self.waypoints[curr_].y)**2)**(1/2)
            dist_to_start = ((self.waypoints[next_].x - self.waypoints[0].x)**2 + (self.waypoints[next_].y - self.waypoints[0].y)**2)**(1/2)
            if dist > self.space:
                theta += dist
                p = Point_ref()
                p.x = self.waypoints[next_].x
                p.y = self.waypoints[next_].y
                p.theta = theta
                p.left_half_width = self.HALF_WIDTH_MAX
                p.right_half_width = self.HALF_WIDTH_MAX
                self.centerline.append(p)
                curr_ = next_

            next_ += 1
            ## terminate when finish a lap
            if next_ > len(self.waypoints)/2 and dist_to_start < self.space:
                break
        
        centerline_len = len(self.centerline)
        last_space = ((self.centerline[centerline_len - 1].x - self.waypoints[0].x)**2 + (self.centerline[centerline_len - 1].y - self.waypoints[0].y)**2)**(1/2)
        self.length = theta + last_space
        p_last = Point_ref()
        p_second_last = Point_ref
        p_last.x = self.waypoints[0].x
        p_last.y = self.waypoints[0].y
        p_last.theta = self.length
        p_second_last.x = 0.5*(self.centerline[centerline_len - 1].x + p_last.x)
        p_second_last.y = 0.5*(self.centerline[centerline_len - 1].y + p_last.y)
        p_second_last.theta = self.length - 0.5*last_space

        if(last_space > self.space):
            self.centerline.append(p_second_last)
        self.centerline.append(p_last)
        self.centerline = np.array(self.centerline)

        print(self.centerline.shape)   

        #for i in range(len(self.centerline)):
        #    print(i)
        #    print(self.centerline[i].theta)

        ## X Y spline

    
    ## return: projected theta along centerline, theta is between [0, length]
    def findTheta(self,x,y,theta_guess,global_search):
        start = 0

        if global_search:
            end = len(self.centerline) - 1
        else:
            mid = int(math.floor(theta_guess/self.space))
            start = mid - self.SEARCH_RANGE
            end = mid + self.SEARCH_RANGE
        
        min_ind = 0
        second_min_ind = 0
        min_dist2 = 10000000.0

        #### change here
        
        for i in range(start,end):
            if i > len(self.centerline) - 1:
                i = 0
            if i < 0:
                i = len(self.centerline) - 1
            ## distance between robot and centerline
            dist2 = ((x - self.centerline[i].x)**2 + (y - self.centerline[i].y)**2)**(1/2)

            if dist2 < min_dist2:
                min_dist2 = dist2
                min_ind = i
        #print("minimum index")
        #print(min_ind)

        ## perform global search
        if (min_dist2)**(1/2) > self.HALF_WIDTH_MAX and not global_search:
            return self.findTheta(self,x,y,theta_guess,True)

        p = np.empty((2,))
        p0 = np.empty((2,))
        p1 = np.empty((2,))

        min_ind_prev = min_ind-1
        min_ind_next = min_ind+1

        ## account for loop
        if min_ind_next > len(self.centerline) - 1:
            min_ind_next -= len(self.centerline)
        if min_ind_prev < 0:
            min_ind_prev += len(self.centerline)

        ## important: find closest line segment: either [min_ind ,min_ind+1] or [min_ind,min_ind-1]
        if pow(x-self.centerline[min_ind_next].x, 2) + pow(y-self.centerline[min_ind_next].y, 2) < pow(x-self.centerline[min_ind_prev].x, 2) + pow(y-self.centerline[min_ind_prev].y, 2):
            second_min_ind = min_ind_next
        else:
            second_min_ind = min_ind_prev
        ## determine theta
        p[0] = x
        p[1] = y
        p0[0] = self.centerline[min_ind].x
        p0[1] = self.centerline[min_ind].y
        p1[0] = self.centerline[second_min_ind].x
        p1[1] = self.centerline[second_min_ind].y

        projection = abs(np.dot(p-p0,p1-p0)/np.linalg.norm(p1-p0))
        theta = 0.0

        if min_ind > second_min_ind:
                theta = self.centerline[min_ind].theta - projection
            
        else:
            if min_ind == 0 and second_min_ind == len(self.centerline) - 1: 
                theta = self.length - projection
            else: 
                theta = self.centerline[min_ind].theta + projection
                
        return theta
        
    

        
        







                







        






