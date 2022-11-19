#!/usr/bin/env python
#from tracemalloc import start
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
import cvxpy as cp
import numpy as np
import sympy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from collections import deque
from sensor_msgs.msg import LaserScan
#from SumOfSquares import SOSProblem, poly_opt_prob
#import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import math
#import tf as TransformFrames
from std_msgs.msg import Header
#import tf

class SOS(object):
    """
    The class that calculates control outputs for obstacle avoidance
    """
    def __init__(self):
        # initialize all the 
        # defined fix V
        self.fixed_velocity = 1.0
        # define parameters and desired angle for control
        # desired rad
        self.p_d = np.array([-45*np.pi/180,-20*np.pi/180,0,20*np.pi/180,45*np.pi/180])
        # states
        self.car_pos_x = 0.0
        self.car_pos_y = 0.0
        self.vehicle_heading_qx = 0.0
        self.vehicle_heading_qy = 0.0
        self.vehicle_heading_qz = 0.0
        self.vehicle_heading_qw = 0.0
        self.min_laser_idx = 0
        self.max_laser_idx = 1080
        self.obs1_point_idx = 0
        # scan angle for obstacles
        self.angle_right_bound = 250
        self.angle_left_bound = 760
        self.detect_radius = 3.0
        # max size for lidar points to
        #self.max_obstacle_size = 100

        # store obstacles in deque and array idx
        self.laser_range = []
        self.obstacle1_found = False
        self.obstacle2_found = False
        self.obstacle3_found = False
        self.count = 0
        self.epsilon = 1e-4
        self.gamma = 1e-4
        self.control_count = 5
        #self.A_f = []
        
        # get obstacle information
        # first obstacle 
        #self.obs1_dis = deque(maxlen = self.max_obstacle_size)
        self.obs1_dis = []
        self.obs_array_idx1 = []
        self.linspace_obs1 = []
        self.linspace_idx1 = []
        self.yaw_ = 0.0
        #print(len(self.obs1_dis))
        
        # second obstacle
        #self.obs2_dis = deque(maxlen = self.max_obstacle_size)
        #self.obs2_dis = []
        #self.obs_array_idx2 = []
        #self.linspace_obs2 = []
        #self.linspace_idx2 = []


        # third obstacle
        #self.obs3_dis = deque(maxlen = self.max_obstacle_size)
        #self.obs3_dis = []
        #self.obs_array_idx3 = []
        #self.linspace_obs3 = []
        #self.linspace_idx3 = []

        ## create subscribers for identification and SOS_computation
        rospy.Subscriber("scan",LaserScan,self.identify_obs,queue_size=1)
        rospy.Subscriber("odom",Odometry,self.odom_callback,queue_size=1)

        ## create publisher for driving and steering
        drive_control = rospy.Publisher("drive",AckermannDrive,queue_size=1)


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


    def identify_obs(self,laser_scan):
        self.laser_range = laser_scan.ranges

        # scan angle for obstacles
        start_counting = False
        normal_start = True
        left_close = False

        start_idx = 0
        end_idx = 0

        if self.laser_range[self.angle_right_bound] < self.detect_radius:
            normal_start = False
            start_counting = True
            start_idx = self.angle_right_bound
            
        for i in range(self.angle_right_bound,self.angle_left_bound):
            
            # store first obstacle and its index into 
            if not self.obstacle1_found and not self.obstacle2_found and not self.obstacle3_found:

                # start counting for first one normal start
                if self.laser_range[i-2] > self.detect_radius and self.laser_range[i] < self.detect_radius and normal_start and start_counting == False:
                    start_counting = True
                    start_idx = i+2
                    print("starting counting index")
                    print(start_idx)
    

                if start_counting == True:
                    self.obs1_dis.append(self.laser_range[i])
                    self.obs_array_idx1 = np.append(self.obs_array_idx1,i)
                    
                
                # special condition check for left
                if i == (self.angle_left_bound - 1) and self.laser_range[i] < self.detect_radius:
                    left_close = True
                    end_idx = self.angle_left_bound
                    self.linspace_idx1 = np.linspace(start_idx,end_idx,5,dtype=int)
                    self.obstacle1_found = True
                    start_counting = False
                    print("end counting index for special")
                    print(end_idx)
                    print("first obstacle index")
                    print(self.linspace_idx1)

                    
                if self.laser_range[i] < self.detect_radius and self.laser_range[i+2] > self.detect_radius and not left_close:
                    end_idx = i
                    self.obstacle1_found = True
                    start_counting = False
                    print("end counting index for normal")
                    print(end_idx)
                    self.linspace_idx1 = np.linspace(start_idx,end_idx,5,dtype=int)
                    print("first obstacle index")
                    print(self.linspace_idx1)
                    #print(self.obs1_dis)

            
            #########################################################################################
            # avoid index error
            #empty_check = i
            #print(empty_check)
            #i = i + 2
            """
            # second obstacle
            if self.obstacle1_found and not self.obstacle2_found and not self.obstacle3_found:

                if self.laser_range[i-2] > self.detect_radius and self.laser_range[i] < self.detect_radius and not start_counting:
                    start_counting = True
                    start_idx = i+2
                    print("start index2")
                    print(start_idx)

                if start_counting == True:
                    self.obs2_dis.append(self.laser_range[i])
                    self.obs_array_idx2 = np.append(self.obs_array_idx2,i)

                if self.laser_range[i] < self.detect_radius and self.laser_range[i+2] > self.detect_radius:
                    end_idx = i
                    # if empty
                    #if start_idx == end_idx + 2:
                       
                    self.linspace_idx2 = np.linspace(start_idx,end_idx+1,5,dtype=int)
                    #print("second obstacle index2")
                    #print(self.linspace_idx2)

                    self.obstacle2_found = True
                    start_counting = False

           # avoid index error
           # i = i + 2

            
            # third obstacle
            if self.obstacle1_found and self.obstacle2_found and not self.obstacle3_found:

                if self.laser_range[i-1] > self.detect_radius and self.laser_range[i] < self.detect_radius:
                    start_counting = True
                    start_idx = i
                    print("start index obs3")
                    print(start_idx)
                    
                if start_counting == True:
                    self.obs3_dis.append(self.laser_range[i])
                    self.obs_array_idx3 = np.append(self.obs_array_idx3,i)

                elif self.laser_range[i] > self.detect_radius and self.laser_range[i+1] > self.detect_radius:
                    end_idx = i
                    print("end index obs3")
                    print(end_idx)
                    self.obstacle3_found = True
                    start_counting = False
                    self.linspace_idx3 = np.linspace(start_idx,end_idx+1,5,dtype=int)
                    print("third obstacle index")
                    print(self.linspace_idx3)
        
            """

        # perfrom coordinate transformation from laser frame to 
        # get angle increment from laser scan
        angle_inc = laser_scan.angle_increment
        laser_x_pos = []
        laser_y_pos = []
        #homo_ones = np.ones(5)
        if(len(self.linspace_idx1) > 0):
            # obtain laser point in robot frame
            for k in range(len(self.linspace_idx1)):
                x_pos_laser = math.cos(-np.pi + angle_inc * self.linspace_idx1[k]) * self.laser_range[self.linspace_idx1[k]]
                y_pos_laser = math.sin(-np.pi + angle_inc * self.linspace_idx1[k]) * self.laser_range[self.linspace_idx1[k]]
                laser_x_pos = np.append(laser_x_pos,x_pos_laser)
                laser_y_pos = np.append(laser_y_pos,y_pos_laser)
            
            #print(laser_x_pos)
            #print(laser_y_pos)
            homo_coordinates = np.stack((laser_x_pos,laser_y_pos),axis=0)

            #print("homo_coordinates")
            #print(homo_coordinates)
            #print(homo_coordinates.shape)



        roll_x,pitch_y,yaw_z = self.euler_from_quaternion(self.vehicle_heading_qx,self.vehicle_heading_qy,self.vehicle_heading_qz,self.vehicle_heading_qw)

        self.yaw_ = yaw_z

        ## convert from laser_frame to map_frame
        laser2map_x = []
        laser2map_y = []
        for i in range(len(homo_coordinates[0])):
            x_pos_map_laser = math.cos(yaw_z) * laser_x_pos[i] - math.sin(yaw_z) * laser_y_pos[i] + self.car_pos_x
            y_pos_map_laser = math.sin(yaw_z) * laser_x_pos[i] + math.cos(yaw_z) * laser_y_pos[i] + self.car_pos_y
            laser2map_x = np.append(laser2map_x,x_pos_map_laser)
            laser2map_y = np.append(laser2map_y,y_pos_map_laser)
        
        laser2map_pos = np.stack((laser2map_x,laser2map_y),axis = 0)
        
        #print("laser2map_pos")
        #print(laser2map_pos)


        
        # create data matrix for 5 points in homogeneous coordinates
        #print("roll pitch yaw")
        #print(yaw_z)

        #distance = np.sqrt(self.vehicle_x**2 + self.vehicle_y**2)
        #print('distance')
        #print(distance)

        # setup the rotation matrix and translation matrix 
        #R_1 = np.array([[math.cos(yaw_z),-math.sin(yaw_z),0],[math.sin(yaw_z),math.cos(yaw_z),0],[0,0,1]])
        #angle_p = math.atan2(self.vehicle_x,self.vehicle_y)
        #angle_psi = angle_p + yaw_z
        #t_1 = np.array([[1,0,distance*math.sin(angle_psi)],[0,1,distance*math.cos(angle_psi)],[0,0,1]])
        
        
        #points_in_map = R_1 @ t_1 @ homo_coordinates
        #print("points in map coordinates and shape")
        #print(points_in_map)
        #print(points_in_map.shape)


        ## construct convex hull
        points_in_map_r = laser2map_pos[0:2,:]
        points = points_in_map_r.transpose()
        #print("points")
        #print(points)
        #print(len(points))

        
        hull = ConvexHull(points)


        # get point from convexhull for polytopic inequ
        vertice_points = hull.vertices
        #print("vertice points")
        #print(vertice_points)

        # store points in counter-clockwise
        convex_points = []
        for i in range(len(vertice_points)):
            convex_points = np.append(convex_points,points[vertice_points[i]])

        convex_points = np.reshape(convex_points,(len(vertice_points),2))

        #print("convex points")
        #print(convex_points)
        
        # get every line equation for generating half space inequaility
        a,b,c = self.lineFromPoints(convex_points[0],convex_points[1])

        a_arr = []
        b_arr = []
        c_arr = []

        for i in range(len(convex_points)-1):
            a,b,c = self.lineFromPoints(convex_points[i],convex_points[i+1])
            a_arr = np.append(a_arr,a)
            b_arr = np.append(b_arr,b)
            c_arr = np.append(c_arr,c)

        # store last one
        a,b,c = self.lineFromPoints(convex_points[0],convex_points[len(convex_points)-1])
        a_arr = np.append(a_arr,a)
        b_arr = np.append(b_arr,b)
        c_arr = np.append(c_arr,c)
        #print(a_arr)

        A = np.zeros((len(convex_points),2))
        b = np.zeros((len(convex_points),1))

        for i in range(len(a_arr)):
            A[i][0] = a_arr[i]
            A[i][1] = b_arr[i]
            b[i][0] = c_arr[i]

        #print("before")
        #print(A)

        # make to ax + by + c = 0 
        c = - b
        #print(c)
        vector_x = np.array([convex_points[1]]).transpose()
        print(A[0] @ vector_x + c[0])
        # loop through any other point not on line
        # basicially check if ax + by + c >= 0 for all points
        for i in range(len(a_arr)):
            for j in range(len(a_arr)):
                if not A[i][0] * convex_points[j][0] + A[i][1] * convex_points[j][1] + c[i] >= -1e-5:
                    # if there is one point does not satisfies the inequaility, change sign
                    A[i][0] = -A[i][0]
                    A[i][1] = -A[i][1]
                    c[i] = - c[i]
                    break
        
        #print("after A")
        #print(A)
        #print("vertice points length")
        #print(len(vertice_points))

        ## generate constraints
        self.generate_constraints(A,a_arr,c)


        ## compute barrier function
        self.SOS_computation(self.car_pos_x,self.car_pos_y,self.yaw_,len(vertice_points))

        ## apply control 




    def generate_constraints(self,A,a_arr,c):
        # dynamics changed
        # same version but using polytope constraints
        x,y,p = sympy.symbols('x y p')
        K = 50
        V = 1.0
        p_d1 = -45*np.pi/180
        p_d2 = -20*np.pi/180
        p_d3 = 0
        p_d4 = 20*np.pi/180
        p_d5 = 45*np.pi/180

        # use current angle
        angle = 0

        # taylor approximation around 0 (should be changed later)
        # change dynamics to match with global 
        f = sympy.Matrix([[V*math.cos(angle) - (V*math.cos(angle)/2)*p**2],[V*math.cos(angle)*p - (V*math.cos(angle))/6*p**3],[V / 0.35 * math.tan(p_d1)]])
        #print(f_1)
        #print(f_2)
        # polynomial basis
        b1 = sympy.Matrix([[1,x,y,p,x**2,x*y,y**2,x*p,y*p,p**2]]).transpose()
        #b1 = sympy.Matrix([[x,y,p,x**2,x*y,y**2,x*p,y*p,p**2]]).transpose()
        #b1 = sympy.Matrix([[1,x,y,p]]).transpose()

        Q = sympy.MatrixSymbol('Q',10,10)
        #Q = sympy.MatrixSymbol('Q',4,4)
        #Q = sympy.MatrixSymbol('Q',9,9)

        # partial derivative
        db1_dx = sympy.diff(b1,x)
        #print(db1_dx)
        db1_dy = sympy.diff(b1,y)
        db1_dp = sympy.diff(b1,p)


        ## parameterization step
        # V
        V = (b1.T @ Q @ b1).as_explicit()
        SOS_V_para_poly = sympy.Poly(V[0],x,y,p)
        B = sympy.Matrix([
            [db1_dx, db1_dy,db1_dp]])
        #print(B.shape)
        ne_Vdot_compute = (- 2 * b1.T @ Q @ B @ f).as_explicit()
        ne_Vdot_compute_poly = sympy.Poly(ne_Vdot_compute[0],x, y, p)

        #print('check coeff of ne_Vdot_compute_poly:')
        #print(ne_Vdot_compute_poly)
    

        # -V_dot
        #b2 = sympy.Matrix([[x,y,p,x**2,x*y,y**2,x*p,y*p,p**2]]).transpose()
        b2 = sympy.Matrix([[1,x,y,p,x**2,x*y,y**2,x*p,y*p,p**2]]).transpose()
        #b2 = sympy.Matrix([[1,x,y,p]]).transpose()
        #G = sympy.MatrixSymbol('G', 4, 4)
        G = sympy.MatrixSymbol('G', 10, 10)
        #G = G = sympy.MatrixSymbol('G', 9, 9)
        SOS_of_ne_V_dot = (b2.T @ G @ b2).as_explicit()
        SOS_of_ne_V_dot_poly = sympy.Poly(SOS_of_ne_V_dot[0], x, y, p)

        #print('\ncheck coeff of SOS_of_ne_V_dot_poly:')
        #print(SOS_of_ne_V_dot_poly)

        b2_sqr = [b*b for b in b2]
        for b_sqr in b2_sqr:
            coeff = SOS_of_ne_V_dot_poly.coeff_monomial(b_sqr)
            if coeff is sympy.S.Zero:
                print('WARNING: digonal coeff is zero for',b_sqr, ' not PSD for sure!')


        # define a vector of largrange multiplier
        multi_vector = []
        J = []
        b3 = sympy.Matrix([[1,x,y,p]]).transpose()
        SOS_mutiplier_para_v = []
        for i in range(len(a_arr)):
            J = sympy.MatrixSymbol('J{}'.format(i),4,4)
            SOS_mutiplier_para = (b3.T @ J @ b3).as_explicit()
            SOS_mutiplier_para_v = np.append(SOS_mutiplier_para_v,SOS_mutiplier_para)
    
        SOS_mutiplier_para_v = np.reshape(SOS_mutiplier_para_v,(1,len(a_arr)))
        #print(SOS_mutiplier_para_v[0][3])

        ### dealing with obstacle constraint SOS

        # obstacle constraint SOS paratrization
        b4 = sympy.Matrix([[1,x,y,p,x**2,x*y,y**2,x*p,y*p,p**2]]).transpose()
        #b4 = sympy.Matrix([[x,y,p,x**2,x*y,y**2,x*p,y*p,p**2]]).transpose()
        #b4 = sympy.Matrix([[1,x,y,p]]).transpose()
        #M = sympy.MatrixSymbol('M',10,10)
        M = sympy.MatrixSymbol('M',10,10)
        #M = sympy.MatrixSymbol('M',4,4)
        SOS_constraint_para = (b4.T @ M @ b4).as_explicit()
        SOS_constraint_para_poly = sympy.Poly(SOS_constraint_para[0],x,y,p)


        #print('\ncheck coeff of SOS_constraint_para_poly:{}'.format(SOS_constraint_para_poly))


        vector_x = sympy.Matrix([[x,y]]).transpose()
        # obstacle constraint SOS computed from V and multiplier
        epsilon_1 = 1e-6
        SOS_mutiplier_para_poly_v = sympy.Poly((SOS_mutiplier_para_v @ (A @ vector_x + c))[0],x,y,p)

        SOS_constraint_compute_poly = SOS_V_para_poly - (1 + epsilon_1) - SOS_mutiplier_para_poly_v
        #print(SOS_constraint_compute_poly)
        constraint_list_V_Vdot = []
        constraint_list_obstacle = []


        #print('\ncheck coeff of SOS_constraint_compute_poly:{}'.format(SOS_constraint_compute_poly))

        max_order = 5
        # deal with V and -V_dot adding coeff matching constraint
        for x_order in range(0,max_order+1):
            for y_order in range(0,max_order + 1):
                for p_order in range(0,max_order + 1):
                    monomial = (x**x_order)*(y**y_order)*(p**p_order)
                    # check if b2.T @ G b2 has coeff
                    coeff_V_dot_para = SOS_of_ne_V_dot_poly.coeff_monomial(monomial)
                    #print('coeff for {} in SOS_of_ne_V_dot_poly is {}'.format(monomial,coeff_V_dot_para))
                    # check if - 2 * b1.T @ Q @ B @ f has coeff
                    coeff_V_dot_compu = ne_Vdot_compute_poly.coeff_monomial(monomial)
                    #print('coeff for {} in ne_Vdot_compute_poly is {}'.format(monomial,coeff_V_dot_compu))
            
                    # continue if both do not contain
                    if coeff_V_dot_para is sympy.S.Zero and coeff_V_dot_compu is sympy.S.Zero:
                        continue
            
                    ## should be modified
                    if coeff_V_dot_para is sympy.S.Zero and not coeff_V_dot_compu is sympy.S.Zero:
                        constrain = '{}==0.'.format(coeff_V_dot_compu)
                        constraint_list_V_Vdot.append(constrain)
                    ## should be modified
                    if not coeff_V_dot_para is sympy.S.Zero and coeff_V_dot_compu is sympy.S.Zero:
                        constrain = '{}==0.'.format(coeff_V_dot_para)
                        constraint_list_V_Vdot.append(constrain)
                
                    if not coeff_V_dot_para is sympy.S.Zero and not coeff_V_dot_compu is sympy.S.Zero:
                        constrain = '{}=={}'.format(coeff_V_dot_compu,coeff_V_dot_para)
                        constraint_list_V_Vdot.append(constrain)

            
        #print('\nConstraints V_Vdot (copy):', ','.join(constraint_list_V_Vdot))                

        # delta with obstacle SOS coeff matching
        for x_order in range(0,max_order+1):
            for y_order in range(0,max_order + 1):
                for p_order in range(0,max_order + 1):
                    monomial = (x**x_order)*(y**y_order)*(p**p_order)
            
                    coeff_constraint_para = SOS_constraint_para_poly.coeff_monomial(monomial)
                    coeff_constraint_compute = SOS_constraint_compute_poly.coeff_monomial(monomial)
            
                    if coeff_constraint_para is sympy.S.Zero and coeff_constraint_compute is sympy.S.Zero:
                        #print(monomial)
                        continue
            
                    ## should be modified
                    if coeff_constraint_para is sympy.S.Zero and not coeff_constraint_compute is sympy.S.Zero:
                        constrain = '{}==0.'.format(coeff_constraint_compute)
                        constraint_list_obstacle.append(constrain)
                    ## should be modified
                    if not coeff_constraint_para is sympy.S.Zero and coeff_constraint_compute is sympy.S.Zero:
                        constrain = '{}==0.'.format(coeff_constraint_para)
                        constraint_list_obstacle.append(constrain)
                
                    if not coeff_constraint_para is sympy.S.Zero and not coeff_constraint_compute is sympy.S.Zero:
                        constrain = '{}=={}'.format(coeff_constraint_compute,coeff_constraint_para)
                        constraint_list_obstacle.append(constrain)

        print('\nConstraints obstacle SOS (copy):', ','.join(constraint_list_obstacle))

        #print("constraint generated")


        ## run barrier function computation
        #self.SOS_computation(self.car_pos_x,self.car_pos_y,self.yaw_)


    def odom_callback(self,odom_msg):
        self.car_pos_x = odom_msg.pose.pose.position.x
        self.car_pos_y = odom_msg.pose.pose.position.y
        self.vehicle_heading_qx = odom_msg.pose.pose.orientation.x
        self.vehicle_heading_qy = odom_msg.pose.pose.orientation.y
        self.vehicle_heading_qz = odom_msg.pose.pose.orientation.z
        self.vehicle_heading_qw = odom_msg.pose.pose.orientation.w


    def lineFromPoints(self,P,Q):
        a = Q[1] - P[1]
        b = P[0] - Q[0]
        c = a*(P[0]) + b*(P[1])
    
        return a,b,c


    # store vehicle current position and orientation
    def SOS_computation(self,vehicle_x,vehicle_y,yaw,vertice_length):

        # solve SOS as SDP based on constraint obtained earilier
        Q = cp.Variable((10,10), symmetric=True)
        G = cp.Variable((10,10), symmetric=True)
        M = cp.Variable((10,10), symmetric = True)


        # create multipliers (should be changed later based on number of convex points)
        J0 = cp.Variable((4,4), symmetric = True)
        J1 = cp.Variable((4,4), symmetric = True)
        J2 = cp.Variable((4,4), symmetric = True)
        J3 = cp.Variable((4,4), symmetric = True)
        constraints = [G >> 0]
        constraints += [M >> 0]
        constraints += [J0 >> 0]
        constraints += [J1 >> 0]
        
        if vertice_length == 3:
            J2 = cp.Variable((4,4), symmetric = True)
            constraints += [J2 >> 0]

        if vertice_length == 4:
            J3 = cp.Variable((4,4), symmetric = True)
            constraints += [J3 >> 0]

        if vertice_length == 5:
            J4 = cp.Variable((4,4), symmetric = True)
            constraints += [J4 >> 0]
        

        # make sure positive definite , sufficient condition
        #epsilon = 1e-8
        #constraints = [Q >> 0]


        # equaility constraint for initial condition
        x_0 = vehicle_x
        y_0 = vehicle_y
        p_0 = yaw

        b_0 = np.array([[1,x_0,y_0,p_0,x_0**2,x_0*y_0,y_0**2,x_0*p_0,y_0*p_0,p_0**2]]).transpose()
        #print(b_0.T @ Q @ b_0)
        #b1 = sympy.Matrix([[1,x,y,p,x**2,x*y,y**2,x*p,y*p,p**2]]).transpose()
        #constraints += 

        constraints += [b_0.T @ Q @ b_0 == 0]

        # V Vdot coeff matching constraint

        # p_d1 
        constraints += [-2.0*Q[0, 1] + 5.71428571428571*Q[0, 3]==G[0, 0],-2.0*Q[0, 2] - 2.0*Q[0, 7] + 11.4285714285714*Q[0, 9] - 2.0*Q[3, 1] + 5.71428571428571*Q[3, 3]==G[0, 3] + G[3, 0],1.0*Q[0, 1] - 2.0*Q[0, 8] - 2.0*Q[3, 2] - 2.0*Q[3, 7] + 11.4285714285714*Q[3, 9] - 2.0*Q[9, 1] + 5.71428571428571*Q[9, 3]==G[0, 9] + G[3, 3] + G[9, 0],0.333333333333333*Q[0, 2] + 1.0*Q[0, 7] + 1.0*Q[3, 1] - 2.0*Q[3, 8] - 2.0*Q[9, 2] - 2.0*Q[9, 7] + 11.4285714285714*Q[9, 9]==G[3, 9] + G[9, 3],0.333333333333333*Q[0, 8] + 0.333333333333333*Q[3, 2] + 1.0*Q[3, 7] + 1.0*Q[9, 1] - 2.0*Q[9, 8]==G[9, 9],0.333333333333333*Q[3, 8] + 0.333333333333333*Q[9, 2] + 1.0*Q[9, 7]==0.,0.333333333333333*Q[9, 8]==0.,-2.0*Q[0, 5] + 5.71428571428571*Q[0, 8] - 2.0*Q[2, 1] + 5.71428571428571*Q[2, 3]==G[0, 2] + G[2, 0],-4.0*Q[0, 6] - 2.0*Q[2, 2] - 2.0*Q[2, 7] + 11.4285714285714*Q[2, 9] - 2.0*Q[3, 5] + 5.71428571428571*Q[3, 8] - 2.0*Q[8, 1] + 5.71428571428571*Q[8, 3]==G[0, 8] + G[2, 3] + G[3, 2] + G[8, 0],1.0*Q[0, 5] + 1.0*Q[2, 1] - 2.0*Q[2, 8] - 4.0*Q[3, 6] - 2.0*Q[8, 2] - 2.0*Q[8, 7] + 11.4285714285714*Q[8, 9] - 2.0*Q[9, 5] + 5.71428571428571*Q[9, 8]==G[2, 9] + G[3, 8] + G[8, 3] + G[9, 2],0.666666666666667*Q[0, 6] + 0.333333333333333*Q[2, 2] + 1.0*Q[2, 7] + 1.0*Q[3, 5] + 1.0*Q[8, 1] - 2.0*Q[8, 8] - 4.0*Q[9, 6]==G[8, 9] + G[9, 8],0.333333333333333*Q[2, 8] + 0.666666666666667*Q[3, 6] + 0.333333333333333*Q[8, 2] + 1.0*Q[8, 7] + 1.0*Q[9, 5]==0.,0.333333333333333*Q[8, 8] + 0.666666666666667*Q[9, 6]==0.,-2.0*Q[2, 5] + 5.71428571428571*Q[2, 8] - 2.0*Q[6, 1] + 5.71428571428571*Q[6, 3]==G[0, 6] + G[2, 2] + G[6, 0],-4.0*Q[2, 6] - 2.0*Q[6, 2] - 2.0*Q[6, 7] + 11.4285714285714*Q[6, 9] - 2.0*Q[8, 5] + 5.71428571428571*Q[8, 8]==G[2, 8] + G[3, 6] + G[6, 3] + G[8, 2],1.0*Q[2, 5] + 1.0*Q[6, 1] - 2.0*Q[6, 8] - 4.0*Q[8, 6]==G[6, 9] + G[8, 8] + G[9, 6],0.666666666666667*Q[2, 6] + 0.333333333333333*Q[6, 2] + 1.0*Q[6, 7] + 1.0*Q[8, 5]==0.,0.333333333333333*Q[6, 8] + 0.666666666666667*Q[8, 6]==0.,-2.0*Q[6, 5] + 5.71428571428571*Q[6, 8]==G[2, 6] + G[6, 2],-4.0*Q[6, 6]==G[6, 8] + G[8, 6],1.0*Q[6, 5]==0.,0.666666666666667*Q[6, 6]==0.,G[6, 6]==0.,-4.0*Q[0, 4] + 5.71428571428571*Q[0, 7] - 2.0*Q[1, 1] + 5.71428571428571*Q[1, 3]==G[0, 1] + G[1, 0],-2.0*Q[0, 5] - 2.0*Q[1, 2] - 2.0*Q[1, 7] + 11.4285714285714*Q[1, 9] - 4.0*Q[3, 4] + 5.71428571428571*Q[3, 7] - 2.0*Q[7, 1] + 5.71428571428571*Q[7, 3]==G[0, 7] + G[1, 3] + G[3, 1] + G[7, 0],2.0*Q[0, 4] + 1.0*Q[1, 1] - 2.0*Q[1, 8] - 2.0*Q[3, 5] - 2.0*Q[7, 2] - 2.0*Q[7, 7] + 11.4285714285714*Q[7, 9] - 4.0*Q[9, 4] + 5.71428571428571*Q[9, 7]==G[1, 9] + G[3, 7] + G[7, 3] + G[9, 1],0.333333333333333*Q[0, 5] + 0.333333333333333*Q[1, 2] + 1.0*Q[1, 7] + 2.0*Q[3, 4] + 1.0*Q[7, 1] - 2.0*Q[7, 8] - 2.0*Q[9, 5]==G[7, 9] + G[9, 7],0.333333333333333*Q[1, 8] + 0.333333333333333*Q[3, 5] + 0.333333333333333*Q[7, 2] + 1.0*Q[7, 7] + 2.0*Q[9, 4]==0.,0.333333333333333*Q[7, 8] + 0.333333333333333*Q[9, 5]==0.,-2.0*Q[1, 5] + 5.71428571428571*Q[1, 8] - 4.0*Q[2, 4] + 5.71428571428571*Q[2, 7] - 2.0*Q[5, 1] + 5.71428571428571*Q[5, 3]==G[0, 5] + G[1, 2] + G[2, 1] + G[5, 0],-4.0*Q[1, 6] - 2.0*Q[2, 5] - 2.0*Q[5, 2] - 2.0*Q[5, 7] + 11.4285714285714*Q[5, 9] - 2.0*Q[7, 5] + 5.71428571428571*Q[7, 8] - 4.0*Q[8, 4] + 5.71428571428571*Q[8, 7]==G[1, 8] + G[2, 7] + G[3, 5] + G[5, 3] + G[7, 2] + G[8, 1],1.0*Q[1, 5] + 2.0*Q[2, 4] + 1.0*Q[5, 1] - 2.0*Q[5, 8] - 4.0*Q[7, 6] - 2.0*Q[8, 5]==G[5, 9] + G[7, 8] + G[8, 7] + G[9, 5],0.666666666666667*Q[1, 6] + 0.333333333333333*Q[2, 5] + 0.333333333333333*Q[5, 2] + 1.0*Q[5, 7] + 1.0*Q[7, 5] + 2.0*Q[8, 4]==0.,0.333333333333333*Q[5, 8] + 0.666666666666667*Q[7, 6] + 0.333333333333333*Q[8, 5]==0.,-2.0*Q[5, 5] + 5.71428571428571*Q[5, 8] - 4.0*Q[6, 4] + 5.71428571428571*Q[6, 7]==G[1, 6] + G[2, 5] + G[5, 2] + G[6, 1],-4.0*Q[5, 6] - 2.0*Q[6, 5]==G[5, 8] + G[6, 7] + G[7, 6] + G[8, 5],1.0*Q[5, 5] + 2.0*Q[6, 4]==0.,0.666666666666667*Q[5, 6] + 0.333333333333333*Q[6, 5]==0.,G[5, 6] + G[6, 5]==0.,-4.0*Q[1, 4] + 5.71428571428571*Q[1, 7] - 2.0*Q[4, 1] + 5.71428571428571*Q[4, 3]==G[0, 4] + G[1, 1] + G[4, 0],-2.0*Q[1, 5] - 2.0*Q[4, 2] - 2.0*Q[4, 7] + 11.4285714285714*Q[4, 9] - 4.0*Q[7, 4] + 5.71428571428571*Q[7, 7]==G[1, 7] + G[3, 4] + G[4, 3] + G[7, 1],2.0*Q[1, 4] + 1.0*Q[4, 1] - 2.0*Q[4, 8] - 2.0*Q[7, 5]==G[4, 9] + G[7, 7] + G[9, 4],0.333333333333333*Q[1, 5] + 0.333333333333333*Q[4, 2] + 1.0*Q[4, 7] + 2.0*Q[7, 4]==0.,0.333333333333333*Q[4, 8] + 0.333333333333333*Q[7, 5]==0.,-2.0*Q[4, 5] + 5.71428571428571*Q[4, 8] - 4.0*Q[5, 4] + 5.71428571428571*Q[5, 7]==G[1, 5] + G[2, 4] + G[4, 2] + G[5, 1],-4.0*Q[4, 6] - 2.0*Q[5, 5]==G[4, 8] + G[5, 7] + G[7, 5] + G[8, 4],1.0*Q[4, 5] + 2.0*Q[5, 4]==0.,0.666666666666667*Q[4, 6] + 0.333333333333333*Q[5, 5]==0.,G[4, 6] + G[5, 5] + G[6, 4]==0.,-4.0*Q[4, 4] + 5.71428571428571*Q[4, 7]==G[1, 4] + G[4, 1],-2.0*Q[4, 5]==G[4, 7] + G[7, 4],2.0*Q[4, 4]==0.,0.333333333333333*Q[4, 5]==0.,G[4, 5] + G[5, 4]==0.,G[4, 4]==0]
        constraints += [-2.4469506396*J0[0, 0] + 0.300563370000002*J1[0, 0] + 0.848238775*J2[0, 0] + 1.0653514616*J3[0, 0] + 1.0*Q[0, 0] - 1.000001==M[0, 0],-2.4469506396*J0[0, 3] - 2.4469506396*J0[3, 0] + 0.300563370000002*J1[0, 3] + 0.300563370000002*J1[3, 0] + 0.848238775*J2[0, 3] + 0.848238775*J2[3, 0] + 1.0653514616*J3[0, 3] + 1.0653514616*J3[3, 0] + 1.0*Q[0, 3] + 1.0*Q[3, 0]==M[0, 3] + M[3, 0],-2.4469506396*J0[3, 3] + 0.300563370000002*J1[3, 3] + 0.848238775*J2[3, 3] + 1.0653514616*J3[3, 3] + 1.0*Q[0, 9] + 1.0*Q[3, 3] + 1.0*Q[9, 0]==M[0, 9] + M[3, 3] + M[9, 0],Q[3, 9] + Q[9, 3]==M[3, 9] + M[9, 3],Q[9, 9]==M[9, 9],-1.02207*J0[0, 0] - 2.4469506396*J0[0, 2] - 2.4469506396*J0[2, 0] + 0.41097*J1[0, 0] + 0.300563370000002*J1[0, 2] + 0.300563370000002*J1[2, 0] + 0.39013*J2[0, 0] + 0.848238775*J2[0, 2] + 0.848238775*J2[2, 0] + 0.22097*J3[0, 0] + 1.0653514616*J3[0, 2] + 1.0653514616*J3[2, 0] + 1.0*Q[0, 2] + 1.0*Q[2, 0]==M[0, 2] + M[2, 0],-1.02207*J0[0, 3] - 2.4469506396*J0[2, 3] - 1.02207*J0[3, 0] - 2.4469506396*J0[3, 2] + 0.41097*J1[0, 3] + 0.300563370000002*J1[2, 3] + 0.41097*J1[3, 0] + 0.300563370000002*J1[3, 2] + 0.39013*J2[0, 3] + 0.848238775*J2[2, 3] + 0.39013*J2[3, 0] + 0.848238775*J2[3, 2] + 0.22097*J3[0, 3] + 1.0653514616*J3[2, 3] + 0.22097*J3[3, 0] + 1.0653514616*J3[3, 2] + 1.0*Q[0, 8] + 1.0*Q[2, 3] + 1.0*Q[3, 2] + 1.0*Q[8, 0]==M[0, 8] + M[2, 3] + M[3, 2] + M[8, 0],-1.02207*J0[3, 3] + 0.41097*J1[3, 3] + 0.39013*J2[3, 3] + 0.22097*J3[3, 3] + 1.0*Q[2, 9] + 1.0*Q[3, 8] + 1.0*Q[8, 3] + 1.0*Q[9, 2]==M[2, 9] + M[3, 8] + M[8, 3] + M[9, 2],Q[8, 9] + Q[9, 8]==M[8, 9] + M[9, 8],-1.02207*J0[0, 2] - 1.02207*J0[2, 0] - 2.4469506396*J0[2, 2] + 0.41097*J1[0, 2] + 0.41097*J1[2, 0] + 0.300563370000002*J1[2, 2] + 0.39013*J2[0, 2] + 0.39013*J2[2, 0] + 0.848238775*J2[2, 2] + 0.22097*J3[0, 2] + 0.22097*J3[2, 0] + 1.0653514616*J3[2, 2] + 1.0*Q[0, 6] + 1.0*Q[2, 2] + 1.0*Q[6, 0]==M[0, 6] + M[2, 2] + M[6, 0],-1.02207*J0[2, 3] - 1.02207*J0[3, 2] + 0.41097*J1[2, 3] + 0.41097*J1[3, 2] + 0.39013*J2[2, 3] + 0.39013*J2[3, 2] + 0.22097*J3[2, 3] + 0.22097*J3[3, 2] + 1.0*Q[2, 8] + 1.0*Q[3, 6] + 1.0*Q[6, 3] + 1.0*Q[8, 2]==M[2, 8] + M[3, 6] + M[6, 3] + M[8, 2],Q[6, 9] + Q[8, 8] + Q[9, 6]==M[6, 9] + M[8, 8] + M[9, 6],-1.02207*J0[2, 2] + 0.41097*J1[2, 2] + 0.39013*J2[2, 2] + 0.22097*J3[2, 2] + 1.0*Q[2, 6] + 1.0*Q[6, 2]==M[2, 6] + M[6, 2],Q[6, 8] + Q[8, 6]==M[6, 8] + M[8, 6],Q[6, 6]==M[6, 6],-0.45477*J0[0, 0] - 2.4469506396*J0[0, 1] - 2.4469506396*J0[1, 0] + 0.34515*J1[0, 0] + 0.300563370000002*J1[0, 1] + 0.300563370000002*J1[1, 0] + 0.1795*J2[0, 0] + 0.848238775*J2[0, 1] + 0.848238775*J2[1, 0] - 0.0698799999999999*J3[0, 0] + 1.0653514616*J3[0, 1] + 1.0653514616*J3[1, 0] + 1.0*Q[0, 1] + 1.0*Q[1, 0]==M[0, 1] + M[1, 0],-0.45477*J0[0, 3] - 2.4469506396*J0[1, 3] - 0.45477*J0[3, 0] - 2.4469506396*J0[3, 1] + 0.34515*J1[0, 3] + 0.300563370000002*J1[1, 3] + 0.34515*J1[3, 0] + 0.300563370000002*J1[3, 1] + 0.1795*J2[0, 3] + 0.848238775*J2[1, 3] + 0.1795*J2[3, 0] + 0.848238775*J2[3, 1] - 0.0698799999999999*J3[0, 3] + 1.0653514616*J3[1, 3] - 0.0698799999999999*J3[3, 0] + 1.0653514616*J3[3, 1] + 1.0*Q[0, 7] + 1.0*Q[1, 3] + 1.0*Q[3, 1] + 1.0*Q[7, 0]==M[0, 7] + M[1, 3] + M[3, 1] + M[7, 0],-0.45477*J0[3, 3] + 0.34515*J1[3, 3] + 0.1795*J2[3, 3] - 0.0698799999999999*J3[3, 3] + 1.0*Q[1, 9] + 1.0*Q[3, 7] + 1.0*Q[7, 3] + 1.0*Q[9, 1]==M[1, 9] + M[3, 7] + M[7, 3] + M[9, 1],Q[7, 9] + Q[9, 7]==M[7, 9] + M[9, 7],-1.02207*J0[0, 1] - 0.45477*J0[0, 2] - 1.02207*J0[1, 0] - 2.4469506396*J0[1, 2] - 0.45477*J0[2, 0] - 2.4469506396*J0[2, 1] + 0.41097*J1[0, 1] + 0.34515*J1[0, 2] + 0.41097*J1[1, 0] + 0.300563370000002*J1[1, 2] + 0.34515*J1[2, 0] + 0.300563370000002*J1[2, 1] + 0.39013*J2[0, 1] + 0.1795*J2[0, 2] + 0.39013*J2[1, 0] + 0.848238775*J2[1, 2] + 0.1795*J2[2, 0] + 0.848238775*J2[2, 1] + 0.22097*J3[0, 1] - 0.0698799999999999*J3[0, 2] + 0.22097*J3[1, 0] + 1.0653514616*J3[1, 2] - 0.0698799999999999*J3[2, 0] + 1.0653514616*J3[2, 1] + 1.0*Q[0, 5] + 1.0*Q[1, 2] + 1.0*Q[2, 1] + 1.0*Q[5, 0]==M[0, 5] + M[1, 2] + M[2, 1] + M[5, 0],-1.02207*J0[1, 3] - 0.45477*J0[2, 3] - 1.02207*J0[3, 1] - 0.45477*J0[3, 2] + 0.41097*J1[1, 3] + 0.34515*J1[2, 3] + 0.41097*J1[3, 1] + 0.34515*J1[3, 2] + 0.39013*J2[1, 3] + 0.1795*J2[2, 3] + 0.39013*J2[3, 1] + 0.1795*J2[3, 2] + 0.22097*J3[1, 3] - 0.0698799999999999*J3[2, 3] + 0.22097*J3[3, 1] - 0.0698799999999999*J3[3, 2] + 1.0*Q[1, 8] + 1.0*Q[2, 7] + 1.0*Q[3, 5] + 1.0*Q[5, 3] + 1.0*Q[7, 2] + 1.0*Q[8, 1]==M[1, 8] + M[2, 7] + M[3, 5] + M[5, 3] + M[7, 2] + M[8, 1],Q[5, 9] + Q[7, 8] + Q[8, 7] + Q[9, 5]==M[5, 9] + M[7, 8] + M[8, 7] + M[9, 5],-1.02207*J0[1, 2] - 1.02207*J0[2, 1] - 0.45477*J0[2, 2] + 0.41097*J1[1, 2] + 0.41097*J1[2, 1] + 0.34515*J1[2, 2] + 0.39013*J2[1, 2] + 0.39013*J2[2, 1] + 0.1795*J2[2, 2] + 0.22097*J3[1, 2] + 0.22097*J3[2, 1] - 0.0698799999999999*J3[2, 2] + 1.0*Q[1, 6] + 1.0*Q[2, 5] + 1.0*Q[5, 2] + 1.0*Q[6, 1]==M[1, 6] + M[2, 5] + M[5, 2] + M[6, 1],Q[5, 8] + Q[6, 7] + Q[7, 6] + Q[8, 5]==M[5, 8] + M[6, 7] + M[7, 6] + M[8, 5],Q[5, 6] + Q[6, 5]==M[5, 6] + M[6, 5],-0.45477*J0[0, 1] - 0.45477*J0[1, 0] - 2.4469506396*J0[1, 1] + 0.34515*J1[0, 1] + 0.34515*J1[1, 0] + 0.300563370000002*J1[1, 1] + 0.1795*J2[0, 1] + 0.1795*J2[1, 0] + 0.848238775*J2[1, 1] - 0.0698799999999999*J3[0, 1] - 0.0698799999999999*J3[1, 0] + 1.0653514616*J3[1, 1] + 1.0*Q[0, 4] + 1.0*Q[1, 1] + 1.0*Q[4, 0]==M[0, 4] + M[1, 1] + M[4, 0],-0.45477*J0[1, 3] - 0.45477*J0[3, 1] + 0.34515*J1[1, 3] + 0.34515*J1[3, 1] + 0.1795*J2[1, 3] + 0.1795*J2[3, 1] - 0.0698799999999999*J3[1, 3] - 0.0698799999999999*J3[3, 1] + 1.0*Q[1, 7] + 1.0*Q[3, 4] + 1.0*Q[4, 3] + 1.0*Q[7, 1]==M[1, 7] + M[3, 4] + M[4, 3] + M[7, 1],Q[4, 9] + Q[7, 7] + Q[9, 4]==M[4, 9] + M[7, 7] + M[9, 4],-1.02207*J0[1, 1] - 0.45477*J0[1, 2] - 0.45477*J0[2, 1] + 0.41097*J1[1, 1] + 0.34515*J1[1, 2] + 0.34515*J1[2, 1] + 0.39013*J2[1, 1] + 0.1795*J2[1, 2] + 0.1795*J2[2, 1] + 0.22097*J3[1, 1] - 0.0698799999999999*J3[1, 2] - 0.0698799999999999*J3[2, 1] + 1.0*Q[1, 5] + 1.0*Q[2, 4] + 1.0*Q[4, 2] + 1.0*Q[5, 1]==M[1, 5] + M[2, 4] + M[4, 2] + M[5, 1],Q[4, 8] + Q[5, 7] + Q[7, 5] + Q[8, 4]==M[4, 8] + M[5, 7] + M[7, 5] + M[8, 4],Q[4, 6] + Q[5, 5] + Q[6, 4]==M[4, 6] + M[5, 5] + M[6, 4],-0.45477*J0[1, 1] + 0.34515*J1[1, 1] + 0.1795*J2[1, 1] - 0.0698799999999999*J3[1, 1] + 1.0*Q[1, 4] + 1.0*Q[4, 1]==M[1, 4] + M[4, 1],Q[4, 7] + Q[7, 4]==M[4, 7] + M[7, 4],Q[4, 5] + Q[5, 4]==M[4, 5] + M[5, 4],Q[4, 4]==M[4, 4]]

        # p_d2
        #constraints += [-2.0*Q[0, 1] + 2.07982991009258*Q[0, 3]==G[0, 0],-2.0*Q[0, 2] - 2.0*Q[0, 7] + 4.15965982018517*Q[0, 9] - 2.0*Q[3, 1] + 2.07982991009258*Q[3, 3]==G[0, 3] + G[3, 0],1.0*Q[0, 1] - 2.0*Q[0, 8] - 2.0*Q[3, 2] - 2.0*Q[3, 7] + 4.15965982018517*Q[3, 9] - 2.0*Q[9, 1] + 2.07982991009258*Q[9, 3]==G[0, 9] + G[3, 3] + G[9, 0],0.333333333333333*Q[0, 2] + 1.0*Q[0, 7] + 1.0*Q[3, 1] - 2.0*Q[3, 8] - 2.0*Q[9, 2] - 2.0*Q[9, 7] + 4.15965982018517*Q[9, 9]==G[3, 9] + G[9, 3],0.333333333333333*Q[0, 8] + 0.333333333333333*Q[3, 2] + 1.0*Q[3, 7] + 1.0*Q[9, 1] - 2.0*Q[9, 8]==G[9, 9],0.333333333333333*Q[3, 8] + 0.333333333333333*Q[9, 2] + 1.0*Q[9, 7]==0.,0.333333333333333*Q[9, 8]==0.,-2.0*Q[0, 5] + 2.07982991009258*Q[0, 8] - 2.0*Q[2, 1] + 2.07982991009258*Q[2, 3]==G[0, 2] + G[2, 0],-4.0*Q[0, 6] - 2.0*Q[2, 2] - 2.0*Q[2, 7] + 4.15965982018517*Q[2, 9] - 2.0*Q[3, 5] + 2.07982991009258*Q[3, 8] - 2.0*Q[8, 1] + 2.07982991009258*Q[8, 3]==G[0, 8] + G[2, 3] + G[3, 2] + G[8, 0],1.0*Q[0, 5] + 1.0*Q[2, 1] - 2.0*Q[2, 8] - 4.0*Q[3, 6] - 2.0*Q[8, 2] - 2.0*Q[8, 7] + 4.15965982018517*Q[8, 9] - 2.0*Q[9, 5] + 2.07982991009258*Q[9, 8]==G[2, 9] + G[3, 8] + G[8, 3] + G[9, 2],0.666666666666667*Q[0, 6] + 0.333333333333333*Q[2, 2] + 1.0*Q[2, 7] + 1.0*Q[3, 5] + 1.0*Q[8, 1] - 2.0*Q[8, 8] - 4.0*Q[9, 6]==G[8, 9] + G[9, 8],0.333333333333333*Q[2, 8] + 0.666666666666667*Q[3, 6] + 0.333333333333333*Q[8, 2] + 1.0*Q[8, 7] + 1.0*Q[9, 5]==0.,0.333333333333333*Q[8, 8] + 0.666666666666667*Q[9, 6]==0.,-2.0*Q[2, 5] + 2.07982991009258*Q[2, 8] - 2.0*Q[6, 1] + 2.07982991009258*Q[6, 3]==G[0, 6] + G[2, 2] + G[6, 0],-4.0*Q[2, 6] - 2.0*Q[6, 2] - 2.0*Q[6, 7] + 4.15965982018517*Q[6, 9] - 2.0*Q[8, 5] + 2.07982991009258*Q[8, 8]==G[2, 8] + G[3, 6] + G[6, 3] + G[8, 2],1.0*Q[2, 5] + 1.0*Q[6, 1] - 2.0*Q[6, 8] - 4.0*Q[8, 6]==G[6, 9] + G[8, 8] + G[9, 6],0.666666666666667*Q[2, 6] + 0.333333333333333*Q[6, 2] + 1.0*Q[6, 7] + 1.0*Q[8, 5]==0.,0.333333333333333*Q[6, 8] + 0.666666666666667*Q[8, 6]==0.,-2.0*Q[6, 5] + 2.07982991009258*Q[6, 8]==G[2, 6] + G[6, 2],-4.0*Q[6, 6]==G[6, 8] + G[8, 6],1.0*Q[6, 5]==0.,0.666666666666667*Q[6, 6]==0.,G[6, 6]==0.,-4.0*Q[0, 4] + 2.07982991009258*Q[0, 7] - 2.0*Q[1, 1] + 2.07982991009258*Q[1, 3]==G[0, 1] + G[1, 0],-2.0*Q[0, 5] - 2.0*Q[1, 2] - 2.0*Q[1, 7] + 4.15965982018517*Q[1, 9] - 4.0*Q[3, 4] + 2.07982991009258*Q[3, 7] - 2.0*Q[7, 1] + 2.07982991009258*Q[7, 3]==G[0, 7] + G[1, 3] + G[3, 1] + G[7, 0],2.0*Q[0, 4] + 1.0*Q[1, 1] - 2.0*Q[1, 8] - 2.0*Q[3, 5] - 2.0*Q[7, 2] - 2.0*Q[7, 7] + 4.15965982018517*Q[7, 9] - 4.0*Q[9, 4] + 2.07982991009258*Q[9, 7]==G[1, 9] + G[3, 7] + G[7, 3] + G[9, 1],0.333333333333333*Q[0, 5] + 0.333333333333333*Q[1, 2] + 1.0*Q[1, 7] + 2.0*Q[3, 4] + 1.0*Q[7, 1] - 2.0*Q[7, 8] - 2.0*Q[9, 5]==G[7, 9] + G[9, 7],0.333333333333333*Q[1, 8] + 0.333333333333333*Q[3, 5] + 0.333333333333333*Q[7, 2] + 1.0*Q[7, 7] + 2.0*Q[9, 4]==0.,0.333333333333333*Q[7, 8] + 0.333333333333333*Q[9, 5]==0.,-2.0*Q[1, 5] + 2.07982991009258*Q[1, 8] - 4.0*Q[2, 4] + 2.07982991009258*Q[2, 7] - 2.0*Q[5, 1] + 2.07982991009258*Q[5, 3]==G[0, 5] + G[1, 2] + G[2, 1] + G[5, 0],-4.0*Q[1, 6] - 2.0*Q[2, 5] - 2.0*Q[5, 2] - 2.0*Q[5, 7] + 4.15965982018517*Q[5, 9] - 2.0*Q[7, 5] + 2.07982991009258*Q[7, 8] - 4.0*Q[8, 4] + 2.07982991009258*Q[8, 7]==G[1, 8] + G[2, 7] + G[3, 5] + G[5, 3] + G[7, 2] + G[8, 1],1.0*Q[1, 5] + 2.0*Q[2, 4] + 1.0*Q[5, 1] - 2.0*Q[5, 8] - 4.0*Q[7, 6] - 2.0*Q[8, 5]==G[5, 9] + G[7, 8] + G[8, 7] + G[9, 5],0.666666666666667*Q[1, 6] + 0.333333333333333*Q[2, 5] + 0.333333333333333*Q[5, 2] + 1.0*Q[5, 7] + 1.0*Q[7, 5] + 2.0*Q[8, 4]==0.,0.333333333333333*Q[5, 8] + 0.666666666666667*Q[7, 6] + 0.333333333333333*Q[8, 5]==0.,-2.0*Q[5, 5] + 2.07982991009258*Q[5, 8] - 4.0*Q[6, 4] + 2.07982991009258*Q[6, 7]==G[1, 6] + G[2, 5] + G[5, 2] + G[6, 1],-4.0*Q[5, 6] - 2.0*Q[6, 5]==G[5, 8] + G[6, 7] + G[7, 6] + G[8, 5],1.0*Q[5, 5] + 2.0*Q[6, 4]==0.,0.666666666666667*Q[5, 6] + 0.333333333333333*Q[6, 5]==0.,G[5, 6] + G[6, 5]==0.,-4.0*Q[1, 4] + 2.07982991009258*Q[1, 7] - 2.0*Q[4, 1] + 2.07982991009258*Q[4, 3]==G[0, 4] + G[1, 1] + G[4, 0],-2.0*Q[1, 5] - 2.0*Q[4, 2] - 2.0*Q[4, 7] + 4.15965982018517*Q[4, 9] - 4.0*Q[7, 4] + 2.07982991009258*Q[7, 7]==G[1, 7] + G[3, 4] + G[4, 3] + G[7, 1],2.0*Q[1, 4] + 1.0*Q[4, 1] - 2.0*Q[4, 8] - 2.0*Q[7, 5]==G[4, 9] + G[7, 7] + G[9, 4],0.333333333333333*Q[1, 5] + 0.333333333333333*Q[4, 2] + 1.0*Q[4, 7] + 2.0*Q[7, 4]==0.,0.333333333333333*Q[4, 8] + 0.333333333333333*Q[7, 5]==0.,-2.0*Q[4, 5] + 2.07982991009258*Q[4, 8] - 4.0*Q[5, 4] + 2.07982991009258*Q[5, 7]==G[1, 5] + G[2, 4] + G[4, 2] + G[5, 1],-4.0*Q[4, 6] - 2.0*Q[5, 5]==G[4, 8] + G[5, 7] + G[7, 5] + G[8, 4],1.0*Q[4, 5] + 2.0*Q[5, 4]==0.,0.666666666666667*Q[4, 6] + 0.333333333333333*Q[5, 5]==0.,G[4, 6] + G[5, 5] + G[6, 4]==0.,-4.0*Q[4, 4] + 2.07982991009258*Q[4, 7]==G[1, 4] + G[4, 1],-2.0*Q[4, 5]==G[4, 7] + G[7, 4],2.0*Q[4, 4]==0.,0.333333333333333*Q[4, 5]==0.,G[4, 5] + G[5, 4]==0.,G[4, 4]==0]
        #constraints += [-2.4469506396*J0[0, 0] + 0.300563370000002*J1[0, 0] + 0.848238775*J2[0, 0] + 1.0653514616*J3[0, 0] + 1.0*Q[0, 0] - 1.000001==M[0, 0],-2.4469506396*J0[0, 3] - 2.4469506396*J0[3, 0] + 0.300563370000002*J1[0, 3] + 0.300563370000002*J1[3, 0] + 0.848238775*J2[0, 3] + 0.848238775*J2[3, 0] + 1.0653514616*J3[0, 3] + 1.0653514616*J3[3, 0] + 1.0*Q[0, 3] + 1.0*Q[3, 0]==M[0, 3] + M[3, 0],-2.4469506396*J0[3, 3] + 0.300563370000002*J1[3, 3] + 0.848238775*J2[3, 3] + 1.0653514616*J3[3, 3] + 1.0*Q[0, 9] + 1.0*Q[3, 3] + 1.0*Q[9, 0]==M[0, 9] + M[3, 3] + M[9, 0],Q[3, 9] + Q[9, 3]==M[3, 9] + M[9, 3],Q[9, 9]==M[9, 9],-1.02207*J0[0, 0] - 2.4469506396*J0[0, 2] - 2.4469506396*J0[2, 0] + 0.41097*J1[0, 0] + 0.300563370000002*J1[0, 2] + 0.300563370000002*J1[2, 0] + 0.39013*J2[0, 0] + 0.848238775*J2[0, 2] + 0.848238775*J2[2, 0] + 0.22097*J3[0, 0] + 1.0653514616*J3[0, 2] + 1.0653514616*J3[2, 0] + 1.0*Q[0, 2] + 1.0*Q[2, 0]==M[0, 2] + M[2, 0],-1.02207*J0[0, 3] - 2.4469506396*J0[2, 3] - 1.02207*J0[3, 0] - 2.4469506396*J0[3, 2] + 0.41097*J1[0, 3] + 0.300563370000002*J1[2, 3] + 0.41097*J1[3, 0] + 0.300563370000002*J1[3, 2] + 0.39013*J2[0, 3] + 0.848238775*J2[2, 3] + 0.39013*J2[3, 0] + 0.848238775*J2[3, 2] + 0.22097*J3[0, 3] + 1.0653514616*J3[2, 3] + 0.22097*J3[3, 0] + 1.0653514616*J3[3, 2] + 1.0*Q[0, 8] + 1.0*Q[2, 3] + 1.0*Q[3, 2] + 1.0*Q[8, 0]==M[0, 8] + M[2, 3] + M[3, 2] + M[8, 0],-1.02207*J0[3, 3] + 0.41097*J1[3, 3] + 0.39013*J2[3, 3] + 0.22097*J3[3, 3] + 1.0*Q[2, 9] + 1.0*Q[3, 8] + 1.0*Q[8, 3] + 1.0*Q[9, 2]==M[2, 9] + M[3, 8] + M[8, 3] + M[9, 2],Q[8, 9] + Q[9, 8]==M[8, 9] + M[9, 8],-1.02207*J0[0, 2] - 1.02207*J0[2, 0] - 2.4469506396*J0[2, 2] + 0.41097*J1[0, 2] + 0.41097*J1[2, 0] + 0.300563370000002*J1[2, 2] + 0.39013*J2[0, 2] + 0.39013*J2[2, 0] + 0.848238775*J2[2, 2] + 0.22097*J3[0, 2] + 0.22097*J3[2, 0] + 1.0653514616*J3[2, 2] + 1.0*Q[0, 6] + 1.0*Q[2, 2] + 1.0*Q[6, 0]==M[0, 6] + M[2, 2] + M[6, 0],-1.02207*J0[2, 3] - 1.02207*J0[3, 2] + 0.41097*J1[2, 3] + 0.41097*J1[3, 2] + 0.39013*J2[2, 3] + 0.39013*J2[3, 2] + 0.22097*J3[2, 3] + 0.22097*J3[3, 2] + 1.0*Q[2, 8] + 1.0*Q[3, 6] + 1.0*Q[6, 3] + 1.0*Q[8, 2]==M[2, 8] + M[3, 6] + M[6, 3] + M[8, 2],Q[6, 9] + Q[8, 8] + Q[9, 6]==M[6, 9] + M[8, 8] + M[9, 6],-1.02207*J0[2, 2] + 0.41097*J1[2, 2] + 0.39013*J2[2, 2] + 0.22097*J3[2, 2] + 1.0*Q[2, 6] + 1.0*Q[6, 2]==M[2, 6] + M[6, 2],Q[6, 8] + Q[8, 6]==M[6, 8] + M[8, 6],Q[6, 6]==M[6, 6],-0.45477*J0[0, 0] - 2.4469506396*J0[0, 1] - 2.4469506396*J0[1, 0] + 0.34515*J1[0, 0] + 0.300563370000002*J1[0, 1] + 0.300563370000002*J1[1, 0] + 0.1795*J2[0, 0] + 0.848238775*J2[0, 1] + 0.848238775*J2[1, 0] - 0.0698799999999999*J3[0, 0] + 1.0653514616*J3[0, 1] + 1.0653514616*J3[1, 0] + 1.0*Q[0, 1] + 1.0*Q[1, 0]==M[0, 1] + M[1, 0],-0.45477*J0[0, 3] - 2.4469506396*J0[1, 3] - 0.45477*J0[3, 0] - 2.4469506396*J0[3, 1] + 0.34515*J1[0, 3] + 0.300563370000002*J1[1, 3] + 0.34515*J1[3, 0] + 0.300563370000002*J1[3, 1] + 0.1795*J2[0, 3] + 0.848238775*J2[1, 3] + 0.1795*J2[3, 0] + 0.848238775*J2[3, 1] - 0.0698799999999999*J3[0, 3] + 1.0653514616*J3[1, 3] - 0.0698799999999999*J3[3, 0] + 1.0653514616*J3[3, 1] + 1.0*Q[0, 7] + 1.0*Q[1, 3] + 1.0*Q[3, 1] + 1.0*Q[7, 0]==M[0, 7] + M[1, 3] + M[3, 1] + M[7, 0],-0.45477*J0[3, 3] + 0.34515*J1[3, 3] + 0.1795*J2[3, 3] - 0.0698799999999999*J3[3, 3] + 1.0*Q[1, 9] + 1.0*Q[3, 7] + 1.0*Q[7, 3] + 1.0*Q[9, 1]==M[1, 9] + M[3, 7] + M[7, 3] + M[9, 1],Q[7, 9] + Q[9, 7]==M[7, 9] + M[9, 7],-1.02207*J0[0, 1] - 0.45477*J0[0, 2] - 1.02207*J0[1, 0] - 2.4469506396*J0[1, 2] - 0.45477*J0[2, 0] - 2.4469506396*J0[2, 1] + 0.41097*J1[0, 1] + 0.34515*J1[0, 2] + 0.41097*J1[1, 0] + 0.300563370000002*J1[1, 2] + 0.34515*J1[2, 0] + 0.300563370000002*J1[2, 1] + 0.39013*J2[0, 1] + 0.1795*J2[0, 2] + 0.39013*J2[1, 0] + 0.848238775*J2[1, 2] + 0.1795*J2[2, 0] + 0.848238775*J2[2, 1] + 0.22097*J3[0, 1] - 0.0698799999999999*J3[0, 2] + 0.22097*J3[1, 0] + 1.0653514616*J3[1, 2] - 0.0698799999999999*J3[2, 0] + 1.0653514616*J3[2, 1] + 1.0*Q[0, 5] + 1.0*Q[1, 2] + 1.0*Q[2, 1] + 1.0*Q[5, 0]==M[0, 5] + M[1, 2] + M[2, 1] + M[5, 0],-1.02207*J0[1, 3] - 0.45477*J0[2, 3] - 1.02207*J0[3, 1] - 0.45477*J0[3, 2] + 0.41097*J1[1, 3] + 0.34515*J1[2, 3] + 0.41097*J1[3, 1] + 0.34515*J1[3, 2] + 0.39013*J2[1, 3] + 0.1795*J2[2, 3] + 0.39013*J2[3, 1] + 0.1795*J2[3, 2] + 0.22097*J3[1, 3] - 0.0698799999999999*J3[2, 3] + 0.22097*J3[3, 1] - 0.0698799999999999*J3[3, 2] + 1.0*Q[1, 8] + 1.0*Q[2, 7] + 1.0*Q[3, 5] + 1.0*Q[5, 3] + 1.0*Q[7, 2] + 1.0*Q[8, 1]==M[1, 8] + M[2, 7] + M[3, 5] + M[5, 3] + M[7, 2] + M[8, 1],Q[5, 9] + Q[7, 8] + Q[8, 7] + Q[9, 5]==M[5, 9] + M[7, 8] + M[8, 7] + M[9, 5],-1.02207*J0[1, 2] - 1.02207*J0[2, 1] - 0.45477*J0[2, 2] + 0.41097*J1[1, 2] + 0.41097*J1[2, 1] + 0.34515*J1[2, 2] + 0.39013*J2[1, 2] + 0.39013*J2[2, 1] + 0.1795*J2[2, 2] + 0.22097*J3[1, 2] + 0.22097*J3[2, 1] - 0.0698799999999999*J3[2, 2] + 1.0*Q[1, 6] + 1.0*Q[2, 5] + 1.0*Q[5, 2] + 1.0*Q[6, 1]==M[1, 6] + M[2, 5] + M[5, 2] + M[6, 1],Q[5, 8] + Q[6, 7] + Q[7, 6] + Q[8, 5]==M[5, 8] + M[6, 7] + M[7, 6] + M[8, 5],Q[5, 6] + Q[6, 5]==M[5, 6] + M[6, 5],-0.45477*J0[0, 1] - 0.45477*J0[1, 0] - 2.4469506396*J0[1, 1] + 0.34515*J1[0, 1] + 0.34515*J1[1, 0] + 0.300563370000002*J1[1, 1] + 0.1795*J2[0, 1] + 0.1795*J2[1, 0] + 0.848238775*J2[1, 1] - 0.0698799999999999*J3[0, 1] - 0.0698799999999999*J3[1, 0] + 1.0653514616*J3[1, 1] + 1.0*Q[0, 4] + 1.0*Q[1, 1] + 1.0*Q[4, 0]==M[0, 4] + M[1, 1] + M[4, 0],-0.45477*J0[1, 3] - 0.45477*J0[3, 1] + 0.34515*J1[1, 3] + 0.34515*J1[3, 1] + 0.1795*J2[1, 3] + 0.1795*J2[3, 1] - 0.0698799999999999*J3[1, 3] - 0.0698799999999999*J3[3, 1] + 1.0*Q[1, 7] + 1.0*Q[3, 4] + 1.0*Q[4, 3] + 1.0*Q[7, 1]==M[1, 7] + M[3, 4] + M[4, 3] + M[7, 1],Q[4, 9] + Q[7, 7] + Q[9, 4]==M[4, 9] + M[7, 7] + M[9, 4],-1.02207*J0[1, 1] - 0.45477*J0[1, 2] - 0.45477*J0[2, 1] + 0.41097*J1[1, 1] + 0.34515*J1[1, 2] + 0.34515*J1[2, 1] + 0.39013*J2[1, 1] + 0.1795*J2[1, 2] + 0.1795*J2[2, 1] + 0.22097*J3[1, 1] - 0.0698799999999999*J3[1, 2] - 0.0698799999999999*J3[2, 1] + 1.0*Q[1, 5] + 1.0*Q[2, 4] + 1.0*Q[4, 2] + 1.0*Q[5, 1]==M[1, 5] + M[2, 4] + M[4, 2] + M[5, 1],Q[4, 8] + Q[5, 7] + Q[7, 5] + Q[8, 4]==M[4, 8] + M[5, 7] + M[7, 5] + M[8, 4],Q[4, 6] + Q[5, 5] + Q[6, 4]==M[4, 6] + M[5, 5] + M[6, 4],-0.45477*J0[1, 1] + 0.34515*J1[1, 1] + 0.1795*J2[1, 1] - 0.0698799999999999*J3[1, 1] + 1.0*Q[1, 4] + 1.0*Q[4, 1]==M[1, 4] + M[4, 1],Q[4, 7] + Q[7, 4]==M[4, 7] + M[7, 4],Q[4, 5] + Q[5, 4]==M[4, 5] + M[5, 4],Q[4, 4]==M[4, 4]]

        # p_d3
        #constraints += [-2.0*Q[0, 1]==G[0, 0],-2.0*Q[0, 2] - 2.0*Q[0, 7] - 2.0*Q[3, 1]==G[0, 3] + G[3, 0],1.0*Q[0, 1] - 2.0*Q[0, 8] - 2.0*Q[3, 2] - 2.0*Q[3, 7] - 2.0*Q[9, 1]==G[0, 9] + G[3, 3] + G[9, 0],0.333333333333333*Q[0, 2] + 1.0*Q[0, 7] + 1.0*Q[3, 1] - 2.0*Q[3, 8] - 2.0*Q[9, 2] - 2.0*Q[9, 7]==G[3, 9] + G[9, 3],0.333333333333333*Q[0, 8] + 0.333333333333333*Q[3, 2] + 1.0*Q[3, 7] + 1.0*Q[9, 1] - 2.0*Q[9, 8]==G[9, 9],0.333333333333333*Q[3, 8] + 0.333333333333333*Q[9, 2] + 1.0*Q[9, 7]==0.,0.333333333333333*Q[9, 8]==0.,-2.0*Q[0, 5] - 2.0*Q[2, 1]==G[0, 2] + G[2, 0],-4.0*Q[0, 6] - 2.0*Q[2, 2] - 2.0*Q[2, 7] - 2.0*Q[3, 5] - 2.0*Q[8, 1]==G[0, 8] + G[2, 3] + G[3, 2] + G[8, 0],1.0*Q[0, 5] + 1.0*Q[2, 1] - 2.0*Q[2, 8] - 4.0*Q[3, 6] - 2.0*Q[8, 2] - 2.0*Q[8, 7] - 2.0*Q[9, 5]==G[2, 9] + G[3, 8] + G[8, 3] + G[9, 2],0.666666666666667*Q[0, 6] + 0.333333333333333*Q[2, 2] + 1.0*Q[2, 7] + 1.0*Q[3, 5] + 1.0*Q[8, 1] - 2.0*Q[8, 8] - 4.0*Q[9, 6]==G[8, 9] + G[9, 8],0.333333333333333*Q[2, 8] + 0.666666666666667*Q[3, 6] + 0.333333333333333*Q[8, 2] + 1.0*Q[8, 7] + 1.0*Q[9, 5]==0.,0.333333333333333*Q[8, 8] + 0.666666666666667*Q[9, 6]==0.,-2.0*Q[2, 5] - 2.0*Q[6, 1]==G[0, 6] + G[2, 2] + G[6, 0],-4.0*Q[2, 6] - 2.0*Q[6, 2] - 2.0*Q[6, 7] - 2.0*Q[8, 5]==G[2, 8] + G[3, 6] + G[6, 3] + G[8, 2],1.0*Q[2, 5] + 1.0*Q[6, 1] - 2.0*Q[6, 8] - 4.0*Q[8, 6]==G[6, 9] + G[8, 8] + G[9, 6],0.666666666666667*Q[2, 6] + 0.333333333333333*Q[6, 2] + 1.0*Q[6, 7] + 1.0*Q[8, 5]==0.,0.333333333333333*Q[6, 8] + 0.666666666666667*Q[8, 6]==0.,-2.0*Q[6, 5]==G[2, 6] + G[6, 2],-4.0*Q[6, 6]==G[6, 8] + G[8, 6],1.0*Q[6, 5]==0.,0.666666666666667*Q[6, 6]==0.,G[6, 6]==0.,-4.0*Q[0, 4] - 2.0*Q[1, 1]==G[0, 1] + G[1, 0],-2.0*Q[0, 5] - 2.0*Q[1, 2] - 2.0*Q[1, 7] - 4.0*Q[3, 4] - 2.0*Q[7, 1]==G[0, 7] + G[1, 3] + G[3, 1] + G[7, 0],2.0*Q[0, 4] + 1.0*Q[1, 1] - 2.0*Q[1, 8] - 2.0*Q[3, 5] - 2.0*Q[7, 2] - 2.0*Q[7, 7] - 4.0*Q[9, 4]==G[1, 9] + G[3, 7] + G[7, 3] + G[9, 1],0.333333333333333*Q[0, 5] + 0.333333333333333*Q[1, 2] + 1.0*Q[1, 7] + 2.0*Q[3, 4] + 1.0*Q[7, 1] - 2.0*Q[7, 8] - 2.0*Q[9, 5]==G[7, 9] + G[9, 7],0.333333333333333*Q[1, 8] + 0.333333333333333*Q[3, 5] + 0.333333333333333*Q[7, 2] + 1.0*Q[7, 7] + 2.0*Q[9, 4]==0.,0.333333333333333*Q[7, 8] + 0.333333333333333*Q[9, 5]==0.,-2.0*Q[1, 5] - 4.0*Q[2, 4] - 2.0*Q[5, 1]==G[0, 5] + G[1, 2] + G[2, 1] + G[5, 0],-4.0*Q[1, 6] - 2.0*Q[2, 5] - 2.0*Q[5, 2] - 2.0*Q[5, 7] - 2.0*Q[7, 5] - 4.0*Q[8, 4]==G[1, 8] + G[2, 7] + G[3, 5] + G[5, 3] + G[7, 2] + G[8, 1],1.0*Q[1, 5] + 2.0*Q[2, 4] + 1.0*Q[5, 1] - 2.0*Q[5, 8] - 4.0*Q[7, 6] - 2.0*Q[8, 5]==G[5, 9] + G[7, 8] + G[8, 7] + G[9, 5],0.666666666666667*Q[1, 6] + 0.333333333333333*Q[2, 5] + 0.333333333333333*Q[5, 2] + 1.0*Q[5, 7] + 1.0*Q[7, 5] + 2.0*Q[8, 4]==0.,0.333333333333333*Q[5, 8] + 0.666666666666667*Q[7, 6] + 0.333333333333333*Q[8, 5]==0.,-2.0*Q[5, 5] - 4.0*Q[6, 4]==G[1, 6] + G[2, 5] + G[5, 2] + G[6, 1],-4.0*Q[5, 6] - 2.0*Q[6, 5]==G[5, 8] + G[6, 7] + G[7, 6] + G[8, 5],1.0*Q[5, 5] + 2.0*Q[6, 4]==0.,0.666666666666667*Q[5, 6] + 0.333333333333333*Q[6, 5]==0.,G[5, 6] + G[6, 5]==0.,-4.0*Q[1, 4] - 2.0*Q[4, 1]==G[0, 4] + G[1, 1] + G[4, 0],-2.0*Q[1, 5] - 2.0*Q[4, 2] - 2.0*Q[4, 7] - 4.0*Q[7, 4]==G[1, 7] + G[3, 4] + G[4, 3] + G[7, 1],2.0*Q[1, 4] + 1.0*Q[4, 1] - 2.0*Q[4, 8] - 2.0*Q[7, 5]==G[4, 9] + G[7, 7] + G[9, 4],0.333333333333333*Q[1, 5] + 0.333333333333333*Q[4, 2] + 1.0*Q[4, 7] + 2.0*Q[7, 4]==0.,0.333333333333333*Q[4, 8] + 0.333333333333333*Q[7, 5]==0.,-2.0*Q[4, 5] - 4.0*Q[5, 4]==G[1, 5] + G[2, 4] + G[4, 2] + G[5, 1],-4.0*Q[4, 6] - 2.0*Q[5, 5]==G[4, 8] + G[5, 7] + G[7, 5] + G[8, 4],1.0*Q[4, 5] + 2.0*Q[5, 4]==0.,0.666666666666667*Q[4, 6] + 0.333333333333333*Q[5, 5]==0.,G[4, 6] + G[5, 5] + G[6, 4]==0.,-4.0*Q[4, 4]==G[1, 4] + G[4, 1],-2.0*Q[4, 5]==G[4, 7] + G[7, 4],2.0*Q[4, 4]==0.,0.333333333333333*Q[4, 5]==0.,G[4, 5] + G[5, 4]==0.,G[4, 4]==0]
        #constraints += [-2.4469506396*J0[0, 0] + 0.300563370000002*J1[0, 0] + 0.848238775*J2[0, 0] + 1.0653514616*J3[0, 0] + 1.0*Q[0, 0] - 1.000001==M[0, 0],-2.4469506396*J0[0, 3] - 2.4469506396*J0[3, 0] + 0.300563370000002*J1[0, 3] + 0.300563370000002*J1[3, 0] + 0.848238775*J2[0, 3] + 0.848238775*J2[3, 0] + 1.0653514616*J3[0, 3] + 1.0653514616*J3[3, 0] + 1.0*Q[0, 3] + 1.0*Q[3, 0]==M[0, 3] + M[3, 0],-2.4469506396*J0[3, 3] + 0.300563370000002*J1[3, 3] + 0.848238775*J2[3, 3] + 1.0653514616*J3[3, 3] + 1.0*Q[0, 9] + 1.0*Q[3, 3] + 1.0*Q[9, 0]==M[0, 9] + M[3, 3] + M[9, 0],Q[3, 9] + Q[9, 3]==M[3, 9] + M[9, 3],Q[9, 9]==M[9, 9],-1.02207*J0[0, 0] - 2.4469506396*J0[0, 2] - 2.4469506396*J0[2, 0] + 0.41097*J1[0, 0] + 0.300563370000002*J1[0, 2] + 0.300563370000002*J1[2, 0] + 0.39013*J2[0, 0] + 0.848238775*J2[0, 2] + 0.848238775*J2[2, 0] + 0.22097*J3[0, 0] + 1.0653514616*J3[0, 2] + 1.0653514616*J3[2, 0] + 1.0*Q[0, 2] + 1.0*Q[2, 0]==M[0, 2] + M[2, 0],-1.02207*J0[0, 3] - 2.4469506396*J0[2, 3] - 1.02207*J0[3, 0] - 2.4469506396*J0[3, 2] + 0.41097*J1[0, 3] + 0.300563370000002*J1[2, 3] + 0.41097*J1[3, 0] + 0.300563370000002*J1[3, 2] + 0.39013*J2[0, 3] + 0.848238775*J2[2, 3] + 0.39013*J2[3, 0] + 0.848238775*J2[3, 2] + 0.22097*J3[0, 3] + 1.0653514616*J3[2, 3] + 0.22097*J3[3, 0] + 1.0653514616*J3[3, 2] + 1.0*Q[0, 8] + 1.0*Q[2, 3] + 1.0*Q[3, 2] + 1.0*Q[8, 0]==M[0, 8] + M[2, 3] + M[3, 2] + M[8, 0],-1.02207*J0[3, 3] + 0.41097*J1[3, 3] + 0.39013*J2[3, 3] + 0.22097*J3[3, 3] + 1.0*Q[2, 9] + 1.0*Q[3, 8] + 1.0*Q[8, 3] + 1.0*Q[9, 2]==M[2, 9] + M[3, 8] + M[8, 3] + M[9, 2],Q[8, 9] + Q[9, 8]==M[8, 9] + M[9, 8],-1.02207*J0[0, 2] - 1.02207*J0[2, 0] - 2.4469506396*J0[2, 2] + 0.41097*J1[0, 2] + 0.41097*J1[2, 0] + 0.300563370000002*J1[2, 2] + 0.39013*J2[0, 2] + 0.39013*J2[2, 0] + 0.848238775*J2[2, 2] + 0.22097*J3[0, 2] + 0.22097*J3[2, 0] + 1.0653514616*J3[2, 2] + 1.0*Q[0, 6] + 1.0*Q[2, 2] + 1.0*Q[6, 0]==M[0, 6] + M[2, 2] + M[6, 0],-1.02207*J0[2, 3] - 1.02207*J0[3, 2] + 0.41097*J1[2, 3] + 0.41097*J1[3, 2] + 0.39013*J2[2, 3] + 0.39013*J2[3, 2] + 0.22097*J3[2, 3] + 0.22097*J3[3, 2] + 1.0*Q[2, 8] + 1.0*Q[3, 6] + 1.0*Q[6, 3] + 1.0*Q[8, 2]==M[2, 8] + M[3, 6] + M[6, 3] + M[8, 2],Q[6, 9] + Q[8, 8] + Q[9, 6]==M[6, 9] + M[8, 8] + M[9, 6],-1.02207*J0[2, 2] + 0.41097*J1[2, 2] + 0.39013*J2[2, 2] + 0.22097*J3[2, 2] + 1.0*Q[2, 6] + 1.0*Q[6, 2]==M[2, 6] + M[6, 2],Q[6, 8] + Q[8, 6]==M[6, 8] + M[8, 6],Q[6, 6]==M[6, 6],-0.45477*J0[0, 0] - 2.4469506396*J0[0, 1] - 2.4469506396*J0[1, 0] + 0.34515*J1[0, 0] + 0.300563370000002*J1[0, 1] + 0.300563370000002*J1[1, 0] + 0.1795*J2[0, 0] + 0.848238775*J2[0, 1] + 0.848238775*J2[1, 0] - 0.0698799999999999*J3[0, 0] + 1.0653514616*J3[0, 1] + 1.0653514616*J3[1, 0] + 1.0*Q[0, 1] + 1.0*Q[1, 0]==M[0, 1] + M[1, 0],-0.45477*J0[0, 3] - 2.4469506396*J0[1, 3] - 0.45477*J0[3, 0] - 2.4469506396*J0[3, 1] + 0.34515*J1[0, 3] + 0.300563370000002*J1[1, 3] + 0.34515*J1[3, 0] + 0.300563370000002*J1[3, 1] + 0.1795*J2[0, 3] + 0.848238775*J2[1, 3] + 0.1795*J2[3, 0] + 0.848238775*J2[3, 1] - 0.0698799999999999*J3[0, 3] + 1.0653514616*J3[1, 3] - 0.0698799999999999*J3[3, 0] + 1.0653514616*J3[3, 1] + 1.0*Q[0, 7] + 1.0*Q[1, 3] + 1.0*Q[3, 1] + 1.0*Q[7, 0]==M[0, 7] + M[1, 3] + M[3, 1] + M[7, 0],-0.45477*J0[3, 3] + 0.34515*J1[3, 3] + 0.1795*J2[3, 3] - 0.0698799999999999*J3[3, 3] + 1.0*Q[1, 9] + 1.0*Q[3, 7] + 1.0*Q[7, 3] + 1.0*Q[9, 1]==M[1, 9] + M[3, 7] + M[7, 3] + M[9, 1],Q[7, 9] + Q[9, 7]==M[7, 9] + M[9, 7],-1.02207*J0[0, 1] - 0.45477*J0[0, 2] - 1.02207*J0[1, 0] - 2.4469506396*J0[1, 2] - 0.45477*J0[2, 0] - 2.4469506396*J0[2, 1] + 0.41097*J1[0, 1] + 0.34515*J1[0, 2] + 0.41097*J1[1, 0] + 0.300563370000002*J1[1, 2] + 0.34515*J1[2, 0] + 0.300563370000002*J1[2, 1] + 0.39013*J2[0, 1] + 0.1795*J2[0, 2] + 0.39013*J2[1, 0] + 0.848238775*J2[1, 2] + 0.1795*J2[2, 0] + 0.848238775*J2[2, 1] + 0.22097*J3[0, 1] - 0.0698799999999999*J3[0, 2] + 0.22097*J3[1, 0] + 1.0653514616*J3[1, 2] - 0.0698799999999999*J3[2, 0] + 1.0653514616*J3[2, 1] + 1.0*Q[0, 5] + 1.0*Q[1, 2] + 1.0*Q[2, 1] + 1.0*Q[5, 0]==M[0, 5] + M[1, 2] + M[2, 1] + M[5, 0],-1.02207*J0[1, 3] - 0.45477*J0[2, 3] - 1.02207*J0[3, 1] - 0.45477*J0[3, 2] + 0.41097*J1[1, 3] + 0.34515*J1[2, 3] + 0.41097*J1[3, 1] + 0.34515*J1[3, 2] + 0.39013*J2[1, 3] + 0.1795*J2[2, 3] + 0.39013*J2[3, 1] + 0.1795*J2[3, 2] + 0.22097*J3[1, 3] - 0.0698799999999999*J3[2, 3] + 0.22097*J3[3, 1] - 0.0698799999999999*J3[3, 2] + 1.0*Q[1, 8] + 1.0*Q[2, 7] + 1.0*Q[3, 5] + 1.0*Q[5, 3] + 1.0*Q[7, 2] + 1.0*Q[8, 1]==M[1, 8] + M[2, 7] + M[3, 5] + M[5, 3] + M[7, 2] + M[8, 1],Q[5, 9] + Q[7, 8] + Q[8, 7] + Q[9, 5]==M[5, 9] + M[7, 8] + M[8, 7] + M[9, 5],-1.02207*J0[1, 2] - 1.02207*J0[2, 1] - 0.45477*J0[2, 2] + 0.41097*J1[1, 2] + 0.41097*J1[2, 1] + 0.34515*J1[2, 2] + 0.39013*J2[1, 2] + 0.39013*J2[2, 1] + 0.1795*J2[2, 2] + 0.22097*J3[1, 2] + 0.22097*J3[2, 1] - 0.0698799999999999*J3[2, 2] + 1.0*Q[1, 6] + 1.0*Q[2, 5] + 1.0*Q[5, 2] + 1.0*Q[6, 1]==M[1, 6] + M[2, 5] + M[5, 2] + M[6, 1],Q[5, 8] + Q[6, 7] + Q[7, 6] + Q[8, 5]==M[5, 8] + M[6, 7] + M[7, 6] + M[8, 5],Q[5, 6] + Q[6, 5]==M[5, 6] + M[6, 5],-0.45477*J0[0, 1] - 0.45477*J0[1, 0] - 2.4469506396*J0[1, 1] + 0.34515*J1[0, 1] + 0.34515*J1[1, 0] + 0.300563370000002*J1[1, 1] + 0.1795*J2[0, 1] + 0.1795*J2[1, 0] + 0.848238775*J2[1, 1] - 0.0698799999999999*J3[0, 1] - 0.0698799999999999*J3[1, 0] + 1.0653514616*J3[1, 1] + 1.0*Q[0, 4] + 1.0*Q[1, 1] + 1.0*Q[4, 0]==M[0, 4] + M[1, 1] + M[4, 0],-0.45477*J0[1, 3] - 0.45477*J0[3, 1] + 0.34515*J1[1, 3] + 0.34515*J1[3, 1] + 0.1795*J2[1, 3] + 0.1795*J2[3, 1] - 0.0698799999999999*J3[1, 3] - 0.0698799999999999*J3[3, 1] + 1.0*Q[1, 7] + 1.0*Q[3, 4] + 1.0*Q[4, 3] + 1.0*Q[7, 1]==M[1, 7] + M[3, 4] + M[4, 3] + M[7, 1],Q[4, 9] + Q[7, 7] + Q[9, 4]==M[4, 9] + M[7, 7] + M[9, 4],-1.02207*J0[1, 1] - 0.45477*J0[1, 2] - 0.45477*J0[2, 1] + 0.41097*J1[1, 1] + 0.34515*J1[1, 2] + 0.34515*J1[2, 1] + 0.39013*J2[1, 1] + 0.1795*J2[1, 2] + 0.1795*J2[2, 1] + 0.22097*J3[1, 1] - 0.0698799999999999*J3[1, 2] - 0.0698799999999999*J3[2, 1] + 1.0*Q[1, 5] + 1.0*Q[2, 4] + 1.0*Q[4, 2] + 1.0*Q[5, 1]==M[1, 5] + M[2, 4] + M[4, 2] + M[5, 1],Q[4, 8] + Q[5, 7] + Q[7, 5] + Q[8, 4]==M[4, 8] + M[5, 7] + M[7, 5] + M[8, 4],Q[4, 6] + Q[5, 5] + Q[6, 4]==M[4, 6] + M[5, 5] + M[6, 4],-0.45477*J0[1, 1] + 0.34515*J1[1, 1] + 0.1795*J2[1, 1] - 0.0698799999999999*J3[1, 1] + 1.0*Q[1, 4] + 1.0*Q[4, 1]==M[1, 4] + M[4, 1],Q[4, 7] + Q[7, 4]==M[4, 7] + M[7, 4],Q[4, 5] + Q[5, 4]==M[4, 5] + M[5, 4],Q[4, 4]==M[4, 4]]

        # p_d4
        #constraints += [-2.0*Q[0, 1] - 2.07982991009258*Q[0, 3]==G[0, 0],-2.0*Q[0, 2] - 2.0*Q[0, 7] - 4.15965982018517*Q[0, 9] - 2.0*Q[3, 1] - 2.07982991009258*Q[3, 3]==G[0, 3] + G[3, 0],1.0*Q[0, 1] - 2.0*Q[0, 8] - 2.0*Q[3, 2] - 2.0*Q[3, 7] - 4.15965982018517*Q[3, 9] - 2.0*Q[9, 1] - 2.07982991009258*Q[9, 3]==G[0, 9] + G[3, 3] + G[9, 0],0.333333333333333*Q[0, 2] + 1.0*Q[0, 7] + 1.0*Q[3, 1] - 2.0*Q[3, 8] - 2.0*Q[9, 2] - 2.0*Q[9, 7] - 4.15965982018517*Q[9, 9]==G[3, 9] + G[9, 3],0.333333333333333*Q[0, 8] + 0.333333333333333*Q[3, 2] + 1.0*Q[3, 7] + 1.0*Q[9, 1] - 2.0*Q[9, 8]==G[9, 9],0.333333333333333*Q[3, 8] + 0.333333333333333*Q[9, 2] + 1.0*Q[9, 7]==0.,0.333333333333333*Q[9, 8]==0.,-2.0*Q[0, 5] - 2.07982991009258*Q[0, 8] - 2.0*Q[2, 1] - 2.07982991009258*Q[2, 3]==G[0, 2] + G[2, 0],-4.0*Q[0, 6] - 2.0*Q[2, 2] - 2.0*Q[2, 7] - 4.15965982018517*Q[2, 9] - 2.0*Q[3, 5] - 2.07982991009258*Q[3, 8] - 2.0*Q[8, 1] - 2.07982991009258*Q[8, 3]==G[0, 8] + G[2, 3] + G[3, 2] + G[8, 0],1.0*Q[0, 5] + 1.0*Q[2, 1] - 2.0*Q[2, 8] - 4.0*Q[3, 6] - 2.0*Q[8, 2] - 2.0*Q[8, 7] - 4.15965982018517*Q[8, 9] - 2.0*Q[9, 5] - 2.07982991009258*Q[9, 8]==G[2, 9] + G[3, 8] + G[8, 3] + G[9, 2],0.666666666666667*Q[0, 6] + 0.333333333333333*Q[2, 2] + 1.0*Q[2, 7] + 1.0*Q[3, 5] + 1.0*Q[8, 1] - 2.0*Q[8, 8] - 4.0*Q[9, 6]==G[8, 9] + G[9, 8],0.333333333333333*Q[2, 8] + 0.666666666666667*Q[3, 6] + 0.333333333333333*Q[8, 2] + 1.0*Q[8, 7] + 1.0*Q[9, 5]==0.,0.333333333333333*Q[8, 8] + 0.666666666666667*Q[9, 6]==0.,-2.0*Q[2, 5] - 2.07982991009258*Q[2, 8] - 2.0*Q[6, 1] - 2.07982991009258*Q[6, 3]==G[0, 6] + G[2, 2] + G[6, 0],-4.0*Q[2, 6] - 2.0*Q[6, 2] - 2.0*Q[6, 7] - 4.15965982018517*Q[6, 9] - 2.0*Q[8, 5] - 2.07982991009258*Q[8, 8]==G[2, 8] + G[3, 6] + G[6, 3] + G[8, 2],1.0*Q[2, 5] + 1.0*Q[6, 1] - 2.0*Q[6, 8] - 4.0*Q[8, 6]==G[6, 9] + G[8, 8] + G[9, 6],0.666666666666667*Q[2, 6] + 0.333333333333333*Q[6, 2] + 1.0*Q[6, 7] + 1.0*Q[8, 5]==0.,0.333333333333333*Q[6, 8] + 0.666666666666667*Q[8, 6]==0.,-2.0*Q[6, 5] - 2.07982991009258*Q[6, 8]==G[2, 6] + G[6, 2],-4.0*Q[6, 6]==G[6, 8] + G[8, 6],1.0*Q[6, 5]==0.,0.666666666666667*Q[6, 6]==0.,G[6, 6]==0.,-4.0*Q[0, 4] - 2.07982991009258*Q[0, 7] - 2.0*Q[1, 1] - 2.07982991009258*Q[1, 3]==G[0, 1] + G[1, 0],-2.0*Q[0, 5] - 2.0*Q[1, 2] - 2.0*Q[1, 7] - 4.15965982018517*Q[1, 9] - 4.0*Q[3, 4] - 2.07982991009258*Q[3, 7] - 2.0*Q[7, 1] - 2.07982991009258*Q[7, 3]==G[0, 7] + G[1, 3] + G[3, 1] + G[7, 0],2.0*Q[0, 4] + 1.0*Q[1, 1] - 2.0*Q[1, 8] - 2.0*Q[3, 5] - 2.0*Q[7, 2] - 2.0*Q[7, 7] - 4.15965982018517*Q[7, 9] - 4.0*Q[9, 4] - 2.07982991009258*Q[9, 7]==G[1, 9] + G[3, 7] + G[7, 3] + G[9, 1],0.333333333333333*Q[0, 5] + 0.333333333333333*Q[1, 2] + 1.0*Q[1, 7] + 2.0*Q[3, 4] + 1.0*Q[7, 1] - 2.0*Q[7, 8] - 2.0*Q[9, 5]==G[7, 9] + G[9, 7],0.333333333333333*Q[1, 8] + 0.333333333333333*Q[3, 5] + 0.333333333333333*Q[7, 2] + 1.0*Q[7, 7] + 2.0*Q[9, 4]==0.,0.333333333333333*Q[7, 8] + 0.333333333333333*Q[9, 5]==0.,-2.0*Q[1, 5] - 2.07982991009258*Q[1, 8] - 4.0*Q[2, 4] - 2.07982991009258*Q[2, 7] - 2.0*Q[5, 1] - 2.07982991009258*Q[5, 3]==G[0, 5] + G[1, 2] + G[2, 1] + G[5, 0],-4.0*Q[1, 6] - 2.0*Q[2, 5] - 2.0*Q[5, 2] - 2.0*Q[5, 7] - 4.15965982018517*Q[5, 9] - 2.0*Q[7, 5] - 2.07982991009258*Q[7, 8] - 4.0*Q[8, 4] - 2.07982991009258*Q[8, 7]==G[1, 8] + G[2, 7] + G[3, 5] + G[5, 3] + G[7, 2] + G[8, 1],1.0*Q[1, 5] + 2.0*Q[2, 4] + 1.0*Q[5, 1] - 2.0*Q[5, 8] - 4.0*Q[7, 6] - 2.0*Q[8, 5]==G[5, 9] + G[7, 8] + G[8, 7] + G[9, 5],0.666666666666667*Q[1, 6] + 0.333333333333333*Q[2, 5] + 0.333333333333333*Q[5, 2] + 1.0*Q[5, 7] + 1.0*Q[7, 5] + 2.0*Q[8, 4]==0.,0.333333333333333*Q[5, 8] + 0.666666666666667*Q[7, 6] + 0.333333333333333*Q[8, 5]==0.,-2.0*Q[5, 5] - 2.07982991009258*Q[5, 8] - 4.0*Q[6, 4] - 2.07982991009258*Q[6, 7]==G[1, 6] + G[2, 5] + G[5, 2] + G[6, 1],-4.0*Q[5, 6] - 2.0*Q[6, 5]==G[5, 8] + G[6, 7] + G[7, 6] + G[8, 5],1.0*Q[5, 5] + 2.0*Q[6, 4]==0.,0.666666666666667*Q[5, 6] + 0.333333333333333*Q[6, 5]==0.,G[5, 6] + G[6, 5]==0.,-4.0*Q[1, 4] - 2.07982991009258*Q[1, 7] - 2.0*Q[4, 1] - 2.07982991009258*Q[4, 3]==G[0, 4] + G[1, 1] + G[4, 0],-2.0*Q[1, 5] - 2.0*Q[4, 2] - 2.0*Q[4, 7] - 4.15965982018517*Q[4, 9] - 4.0*Q[7, 4] - 2.07982991009258*Q[7, 7]==G[1, 7] + G[3, 4] + G[4, 3] + G[7, 1],2.0*Q[1, 4] + 1.0*Q[4, 1] - 2.0*Q[4, 8] - 2.0*Q[7, 5]==G[4, 9] + G[7, 7] + G[9, 4],0.333333333333333*Q[1, 5] + 0.333333333333333*Q[4, 2] + 1.0*Q[4, 7] + 2.0*Q[7, 4]==0.,0.333333333333333*Q[4, 8] + 0.333333333333333*Q[7, 5]==0.,-2.0*Q[4, 5] - 2.07982991009258*Q[4, 8] - 4.0*Q[5, 4] - 2.07982991009258*Q[5, 7]==G[1, 5] + G[2, 4] + G[4, 2] + G[5, 1],-4.0*Q[4, 6] - 2.0*Q[5, 5]==G[4, 8] + G[5, 7] + G[7, 5] + G[8, 4],1.0*Q[4, 5] + 2.0*Q[5, 4]==0.,0.666666666666667*Q[4, 6] + 0.333333333333333*Q[5, 5]==0.,G[4, 6] + G[5, 5] + G[6, 4]==0.,-4.0*Q[4, 4] - 2.07982991009258*Q[4, 7]==G[1, 4] + G[4, 1],-2.0*Q[4, 5]==G[4, 7] + G[7, 4],2.0*Q[4, 4]==0.,0.333333333333333*Q[4, 5]==0.,G[4, 5] + G[5, 4]==0.,G[4, 4]==0]
        #constraints += [-2.4469506396*J0[0, 0] + 0.300563370000002*J1[0, 0] + 0.848238775*J2[0, 0] + 1.0653514616*J3[0, 0] + 1.0*Q[0, 0] - 1.000001==M[0, 0],-2.4469506396*J0[0, 3] - 2.4469506396*J0[3, 0] + 0.300563370000002*J1[0, 3] + 0.300563370000002*J1[3, 0] + 0.848238775*J2[0, 3] + 0.848238775*J2[3, 0] + 1.0653514616*J3[0, 3] + 1.0653514616*J3[3, 0] + 1.0*Q[0, 3] + 1.0*Q[3, 0]==M[0, 3] + M[3, 0],-2.4469506396*J0[3, 3] + 0.300563370000002*J1[3, 3] + 0.848238775*J2[3, 3] + 1.0653514616*J3[3, 3] + 1.0*Q[0, 9] + 1.0*Q[3, 3] + 1.0*Q[9, 0]==M[0, 9] + M[3, 3] + M[9, 0],Q[3, 9] + Q[9, 3]==M[3, 9] + M[9, 3],Q[9, 9]==M[9, 9],-1.02207*J0[0, 0] - 2.4469506396*J0[0, 2] - 2.4469506396*J0[2, 0] + 0.41097*J1[0, 0] + 0.300563370000002*J1[0, 2] + 0.300563370000002*J1[2, 0] + 0.39013*J2[0, 0] + 0.848238775*J2[0, 2] + 0.848238775*J2[2, 0] + 0.22097*J3[0, 0] + 1.0653514616*J3[0, 2] + 1.0653514616*J3[2, 0] + 1.0*Q[0, 2] + 1.0*Q[2, 0]==M[0, 2] + M[2, 0],-1.02207*J0[0, 3] - 2.4469506396*J0[2, 3] - 1.02207*J0[3, 0] - 2.4469506396*J0[3, 2] + 0.41097*J1[0, 3] + 0.300563370000002*J1[2, 3] + 0.41097*J1[3, 0] + 0.300563370000002*J1[3, 2] + 0.39013*J2[0, 3] + 0.848238775*J2[2, 3] + 0.39013*J2[3, 0] + 0.848238775*J2[3, 2] + 0.22097*J3[0, 3] + 1.0653514616*J3[2, 3] + 0.22097*J3[3, 0] + 1.0653514616*J3[3, 2] + 1.0*Q[0, 8] + 1.0*Q[2, 3] + 1.0*Q[3, 2] + 1.0*Q[8, 0]==M[0, 8] + M[2, 3] + M[3, 2] + M[8, 0],-1.02207*J0[3, 3] + 0.41097*J1[3, 3] + 0.39013*J2[3, 3] + 0.22097*J3[3, 3] + 1.0*Q[2, 9] + 1.0*Q[3, 8] + 1.0*Q[8, 3] + 1.0*Q[9, 2]==M[2, 9] + M[3, 8] + M[8, 3] + M[9, 2],Q[8, 9] + Q[9, 8]==M[8, 9] + M[9, 8],-1.02207*J0[0, 2] - 1.02207*J0[2, 0] - 2.4469506396*J0[2, 2] + 0.41097*J1[0, 2] + 0.41097*J1[2, 0] + 0.300563370000002*J1[2, 2] + 0.39013*J2[0, 2] + 0.39013*J2[2, 0] + 0.848238775*J2[2, 2] + 0.22097*J3[0, 2] + 0.22097*J3[2, 0] + 1.0653514616*J3[2, 2] + 1.0*Q[0, 6] + 1.0*Q[2, 2] + 1.0*Q[6, 0]==M[0, 6] + M[2, 2] + M[6, 0],-1.02207*J0[2, 3] - 1.02207*J0[3, 2] + 0.41097*J1[2, 3] + 0.41097*J1[3, 2] + 0.39013*J2[2, 3] + 0.39013*J2[3, 2] + 0.22097*J3[2, 3] + 0.22097*J3[3, 2] + 1.0*Q[2, 8] + 1.0*Q[3, 6] + 1.0*Q[6, 3] + 1.0*Q[8, 2]==M[2, 8] + M[3, 6] + M[6, 3] + M[8, 2],Q[6, 9] + Q[8, 8] + Q[9, 6]==M[6, 9] + M[8, 8] + M[9, 6],-1.02207*J0[2, 2] + 0.41097*J1[2, 2] + 0.39013*J2[2, 2] + 0.22097*J3[2, 2] + 1.0*Q[2, 6] + 1.0*Q[6, 2]==M[2, 6] + M[6, 2],Q[6, 8] + Q[8, 6]==M[6, 8] + M[8, 6],Q[6, 6]==M[6, 6],-0.45477*J0[0, 0] - 2.4469506396*J0[0, 1] - 2.4469506396*J0[1, 0] + 0.34515*J1[0, 0] + 0.300563370000002*J1[0, 1] + 0.300563370000002*J1[1, 0] + 0.1795*J2[0, 0] + 0.848238775*J2[0, 1] + 0.848238775*J2[1, 0] - 0.0698799999999999*J3[0, 0] + 1.0653514616*J3[0, 1] + 1.0653514616*J3[1, 0] + 1.0*Q[0, 1] + 1.0*Q[1, 0]==M[0, 1] + M[1, 0],-0.45477*J0[0, 3] - 2.4469506396*J0[1, 3] - 0.45477*J0[3, 0] - 2.4469506396*J0[3, 1] + 0.34515*J1[0, 3] + 0.300563370000002*J1[1, 3] + 0.34515*J1[3, 0] + 0.300563370000002*J1[3, 1] + 0.1795*J2[0, 3] + 0.848238775*J2[1, 3] + 0.1795*J2[3, 0] + 0.848238775*J2[3, 1] - 0.0698799999999999*J3[0, 3] + 1.0653514616*J3[1, 3] - 0.0698799999999999*J3[3, 0] + 1.0653514616*J3[3, 1] + 1.0*Q[0, 7] + 1.0*Q[1, 3] + 1.0*Q[3, 1] + 1.0*Q[7, 0]==M[0, 7] + M[1, 3] + M[3, 1] + M[7, 0],-0.45477*J0[3, 3] + 0.34515*J1[3, 3] + 0.1795*J2[3, 3] - 0.0698799999999999*J3[3, 3] + 1.0*Q[1, 9] + 1.0*Q[3, 7] + 1.0*Q[7, 3] + 1.0*Q[9, 1]==M[1, 9] + M[3, 7] + M[7, 3] + M[9, 1],Q[7, 9] + Q[9, 7]==M[7, 9] + M[9, 7],-1.02207*J0[0, 1] - 0.45477*J0[0, 2] - 1.02207*J0[1, 0] - 2.4469506396*J0[1, 2] - 0.45477*J0[2, 0] - 2.4469506396*J0[2, 1] + 0.41097*J1[0, 1] + 0.34515*J1[0, 2] + 0.41097*J1[1, 0] + 0.300563370000002*J1[1, 2] + 0.34515*J1[2, 0] + 0.300563370000002*J1[2, 1] + 0.39013*J2[0, 1] + 0.1795*J2[0, 2] + 0.39013*J2[1, 0] + 0.848238775*J2[1, 2] + 0.1795*J2[2, 0] + 0.848238775*J2[2, 1] + 0.22097*J3[0, 1] - 0.0698799999999999*J3[0, 2] + 0.22097*J3[1, 0] + 1.0653514616*J3[1, 2] - 0.0698799999999999*J3[2, 0] + 1.0653514616*J3[2, 1] + 1.0*Q[0, 5] + 1.0*Q[1, 2] + 1.0*Q[2, 1] + 1.0*Q[5, 0]==M[0, 5] + M[1, 2] + M[2, 1] + M[5, 0],-1.02207*J0[1, 3] - 0.45477*J0[2, 3] - 1.02207*J0[3, 1] - 0.45477*J0[3, 2] + 0.41097*J1[1, 3] + 0.34515*J1[2, 3] + 0.41097*J1[3, 1] + 0.34515*J1[3, 2] + 0.39013*J2[1, 3] + 0.1795*J2[2, 3] + 0.39013*J2[3, 1] + 0.1795*J2[3, 2] + 0.22097*J3[1, 3] - 0.0698799999999999*J3[2, 3] + 0.22097*J3[3, 1] - 0.0698799999999999*J3[3, 2] + 1.0*Q[1, 8] + 1.0*Q[2, 7] + 1.0*Q[3, 5] + 1.0*Q[5, 3] + 1.0*Q[7, 2] + 1.0*Q[8, 1]==M[1, 8] + M[2, 7] + M[3, 5] + M[5, 3] + M[7, 2] + M[8, 1],Q[5, 9] + Q[7, 8] + Q[8, 7] + Q[9, 5]==M[5, 9] + M[7, 8] + M[8, 7] + M[9, 5],-1.02207*J0[1, 2] - 1.02207*J0[2, 1] - 0.45477*J0[2, 2] + 0.41097*J1[1, 2] + 0.41097*J1[2, 1] + 0.34515*J1[2, 2] + 0.39013*J2[1, 2] + 0.39013*J2[2, 1] + 0.1795*J2[2, 2] + 0.22097*J3[1, 2] + 0.22097*J3[2, 1] - 0.0698799999999999*J3[2, 2] + 1.0*Q[1, 6] + 1.0*Q[2, 5] + 1.0*Q[5, 2] + 1.0*Q[6, 1]==M[1, 6] + M[2, 5] + M[5, 2] + M[6, 1],Q[5, 8] + Q[6, 7] + Q[7, 6] + Q[8, 5]==M[5, 8] + M[6, 7] + M[7, 6] + M[8, 5],Q[5, 6] + Q[6, 5]==M[5, 6] + M[6, 5],-0.45477*J0[0, 1] - 0.45477*J0[1, 0] - 2.4469506396*J0[1, 1] + 0.34515*J1[0, 1] + 0.34515*J1[1, 0] + 0.300563370000002*J1[1, 1] + 0.1795*J2[0, 1] + 0.1795*J2[1, 0] + 0.848238775*J2[1, 1] - 0.0698799999999999*J3[0, 1] - 0.0698799999999999*J3[1, 0] + 1.0653514616*J3[1, 1] + 1.0*Q[0, 4] + 1.0*Q[1, 1] + 1.0*Q[4, 0]==M[0, 4] + M[1, 1] + M[4, 0],-0.45477*J0[1, 3] - 0.45477*J0[3, 1] + 0.34515*J1[1, 3] + 0.34515*J1[3, 1] + 0.1795*J2[1, 3] + 0.1795*J2[3, 1] - 0.0698799999999999*J3[1, 3] - 0.0698799999999999*J3[3, 1] + 1.0*Q[1, 7] + 1.0*Q[3, 4] + 1.0*Q[4, 3] + 1.0*Q[7, 1]==M[1, 7] + M[3, 4] + M[4, 3] + M[7, 1],Q[4, 9] + Q[7, 7] + Q[9, 4]==M[4, 9] + M[7, 7] + M[9, 4],-1.02207*J0[1, 1] - 0.45477*J0[1, 2] - 0.45477*J0[2, 1] + 0.41097*J1[1, 1] + 0.34515*J1[1, 2] + 0.34515*J1[2, 1] + 0.39013*J2[1, 1] + 0.1795*J2[1, 2] + 0.1795*J2[2, 1] + 0.22097*J3[1, 1] - 0.0698799999999999*J3[1, 2] - 0.0698799999999999*J3[2, 1] + 1.0*Q[1, 5] + 1.0*Q[2, 4] + 1.0*Q[4, 2] + 1.0*Q[5, 1]==M[1, 5] + M[2, 4] + M[4, 2] + M[5, 1],Q[4, 8] + Q[5, 7] + Q[7, 5] + Q[8, 4]==M[4, 8] + M[5, 7] + M[7, 5] + M[8, 4],Q[4, 6] + Q[5, 5] + Q[6, 4]==M[4, 6] + M[5, 5] + M[6, 4],-0.45477*J0[1, 1] + 0.34515*J1[1, 1] + 0.1795*J2[1, 1] - 0.0698799999999999*J3[1, 1] + 1.0*Q[1, 4] + 1.0*Q[4, 1]==M[1, 4] + M[4, 1],Q[4, 7] + Q[7, 4]==M[4, 7] + M[7, 4],Q[4, 5] + Q[5, 4]==M[4, 5] + M[5, 4],Q[4, 4]==M[4, 4]]

        # p_d5
        #constraints += [-2.0*Q[0, 1] - 5.71428571428571*Q[0, 3]==G[0, 0],-2.0*Q[0, 2] - 2.0*Q[0, 7] - 11.4285714285714*Q[0, 9] - 2.0*Q[3, 1] - 5.71428571428571*Q[3, 3]==G[0, 3] + G[3, 0],1.0*Q[0, 1] - 2.0*Q[0, 8] - 2.0*Q[3, 2] - 2.0*Q[3, 7] - 11.4285714285714*Q[3, 9] - 2.0*Q[9, 1] - 5.71428571428571*Q[9, 3]==G[0, 9] + G[3, 3] + G[9, 0],0.333333333333333*Q[0, 2] + 1.0*Q[0, 7] + 1.0*Q[3, 1] - 2.0*Q[3, 8] - 2.0*Q[9, 2] - 2.0*Q[9, 7] - 11.4285714285714*Q[9, 9]==G[3, 9] + G[9, 3],0.333333333333333*Q[0, 8] + 0.333333333333333*Q[3, 2] + 1.0*Q[3, 7] + 1.0*Q[9, 1] - 2.0*Q[9, 8]==G[9, 9],0.333333333333333*Q[3, 8] + 0.333333333333333*Q[9, 2] + 1.0*Q[9, 7]==0.,0.333333333333333*Q[9, 8]==0.,-2.0*Q[0, 5] - 5.71428571428571*Q[0, 8] - 2.0*Q[2, 1] - 5.71428571428571*Q[2, 3]==G[0, 2] + G[2, 0],-4.0*Q[0, 6] - 2.0*Q[2, 2] - 2.0*Q[2, 7] - 11.4285714285714*Q[2, 9] - 2.0*Q[3, 5] - 5.71428571428571*Q[3, 8] - 2.0*Q[8, 1] - 5.71428571428571*Q[8, 3]==G[0, 8] + G[2, 3] + G[3, 2] + G[8, 0],1.0*Q[0, 5] + 1.0*Q[2, 1] - 2.0*Q[2, 8] - 4.0*Q[3, 6] - 2.0*Q[8, 2] - 2.0*Q[8, 7] - 11.4285714285714*Q[8, 9] - 2.0*Q[9, 5] - 5.71428571428571*Q[9, 8]==G[2, 9] + G[3, 8] + G[8, 3] + G[9, 2],0.666666666666667*Q[0, 6] + 0.333333333333333*Q[2, 2] + 1.0*Q[2, 7] + 1.0*Q[3, 5] + 1.0*Q[8, 1] - 2.0*Q[8, 8] - 4.0*Q[9, 6]==G[8, 9] + G[9, 8],0.333333333333333*Q[2, 8] + 0.666666666666667*Q[3, 6] + 0.333333333333333*Q[8, 2] + 1.0*Q[8, 7] + 1.0*Q[9, 5]==0.,0.333333333333333*Q[8, 8] + 0.666666666666667*Q[9, 6]==0.,-2.0*Q[2, 5] - 5.71428571428571*Q[2, 8] - 2.0*Q[6, 1] - 5.71428571428571*Q[6, 3]==G[0, 6] + G[2, 2] + G[6, 0],-4.0*Q[2, 6] - 2.0*Q[6, 2] - 2.0*Q[6, 7] - 11.4285714285714*Q[6, 9] - 2.0*Q[8, 5] - 5.71428571428571*Q[8, 8]==G[2, 8] + G[3, 6] + G[6, 3] + G[8, 2],1.0*Q[2, 5] + 1.0*Q[6, 1] - 2.0*Q[6, 8] - 4.0*Q[8, 6]==G[6, 9] + G[8, 8] + G[9, 6],0.666666666666667*Q[2, 6] + 0.333333333333333*Q[6, 2] + 1.0*Q[6, 7] + 1.0*Q[8, 5]==0.,0.333333333333333*Q[6, 8] + 0.666666666666667*Q[8, 6]==0.,-2.0*Q[6, 5] - 5.71428571428571*Q[6, 8]==G[2, 6] + G[6, 2],-4.0*Q[6, 6]==G[6, 8] + G[8, 6],1.0*Q[6, 5]==0.,0.666666666666667*Q[6, 6]==0.,G[6, 6]==0.,-4.0*Q[0, 4] - 5.71428571428571*Q[0, 7] - 2.0*Q[1, 1] - 5.71428571428571*Q[1, 3]==G[0, 1] + G[1, 0],-2.0*Q[0, 5] - 2.0*Q[1, 2] - 2.0*Q[1, 7] - 11.4285714285714*Q[1, 9] - 4.0*Q[3, 4] - 5.71428571428571*Q[3, 7] - 2.0*Q[7, 1] - 5.71428571428571*Q[7, 3]==G[0, 7] + G[1, 3] + G[3, 1] + G[7, 0],2.0*Q[0, 4] + 1.0*Q[1, 1] - 2.0*Q[1, 8] - 2.0*Q[3, 5] - 2.0*Q[7, 2] - 2.0*Q[7, 7] - 11.4285714285714*Q[7, 9] - 4.0*Q[9, 4] - 5.71428571428571*Q[9, 7]==G[1, 9] + G[3, 7] + G[7, 3] + G[9, 1],0.333333333333333*Q[0, 5] + 0.333333333333333*Q[1, 2] + 1.0*Q[1, 7] + 2.0*Q[3, 4] + 1.0*Q[7, 1] - 2.0*Q[7, 8] - 2.0*Q[9, 5]==G[7, 9] + G[9, 7],0.333333333333333*Q[1, 8] + 0.333333333333333*Q[3, 5] + 0.333333333333333*Q[7, 2] + 1.0*Q[7, 7] + 2.0*Q[9, 4]==0.,0.333333333333333*Q[7, 8] + 0.333333333333333*Q[9, 5]==0.,-2.0*Q[1, 5] - 5.71428571428571*Q[1, 8] - 4.0*Q[2, 4] - 5.71428571428571*Q[2, 7] - 2.0*Q[5, 1] - 5.71428571428571*Q[5, 3]==G[0, 5] + G[1, 2] + G[2, 1] + G[5, 0],-4.0*Q[1, 6] - 2.0*Q[2, 5] - 2.0*Q[5, 2] - 2.0*Q[5, 7] - 11.4285714285714*Q[5, 9] - 2.0*Q[7, 5] - 5.71428571428571*Q[7, 8] - 4.0*Q[8, 4] - 5.71428571428571*Q[8, 7]==G[1, 8] + G[2, 7] + G[3, 5] + G[5, 3] + G[7, 2] + G[8, 1],1.0*Q[1, 5] + 2.0*Q[2, 4] + 1.0*Q[5, 1] - 2.0*Q[5, 8] - 4.0*Q[7, 6] - 2.0*Q[8, 5]==G[5, 9] + G[7, 8] + G[8, 7] + G[9, 5],0.666666666666667*Q[1, 6] + 0.333333333333333*Q[2, 5] + 0.333333333333333*Q[5, 2] + 1.0*Q[5, 7] + 1.0*Q[7, 5] + 2.0*Q[8, 4]==0.,0.333333333333333*Q[5, 8] + 0.666666666666667*Q[7, 6] + 0.333333333333333*Q[8, 5]==0.,-2.0*Q[5, 5] - 5.71428571428571*Q[5, 8] - 4.0*Q[6, 4] - 5.71428571428571*Q[6, 7]==G[1, 6] + G[2, 5] + G[5, 2] + G[6, 1],-4.0*Q[5, 6] - 2.0*Q[6, 5]==G[5, 8] + G[6, 7] + G[7, 6] + G[8, 5],1.0*Q[5, 5] + 2.0*Q[6, 4]==0.,0.666666666666667*Q[5, 6] + 0.333333333333333*Q[6, 5]==0.,G[5, 6] + G[6, 5]==0.,-4.0*Q[1, 4] - 5.71428571428571*Q[1, 7] - 2.0*Q[4, 1] - 5.71428571428571*Q[4, 3]==G[0, 4] + G[1, 1] + G[4, 0],-2.0*Q[1, 5] - 2.0*Q[4, 2] - 2.0*Q[4, 7] - 11.4285714285714*Q[4, 9] - 4.0*Q[7, 4] - 5.71428571428571*Q[7, 7]==G[1, 7] + G[3, 4] + G[4, 3] + G[7, 1],2.0*Q[1, 4] + 1.0*Q[4, 1] - 2.0*Q[4, 8] - 2.0*Q[7, 5]==G[4, 9] + G[7, 7] + G[9, 4],0.333333333333333*Q[1, 5] + 0.333333333333333*Q[4, 2] + 1.0*Q[4, 7] + 2.0*Q[7, 4]==0.,0.333333333333333*Q[4, 8] + 0.333333333333333*Q[7, 5]==0.,-2.0*Q[4, 5] - 5.71428571428571*Q[4, 8] - 4.0*Q[5, 4] - 5.71428571428571*Q[5, 7]==G[1, 5] + G[2, 4] + G[4, 2] + G[5, 1],-4.0*Q[4, 6] - 2.0*Q[5, 5]==G[4, 8] + G[5, 7] + G[7, 5] + G[8, 4],1.0*Q[4, 5] + 2.0*Q[5, 4]==0.,0.666666666666667*Q[4, 6] + 0.333333333333333*Q[5, 5]==0.,G[4, 6] + G[5, 5] + G[6, 4]==0.,-4.0*Q[4, 4] - 5.71428571428571*Q[4, 7]==G[1, 4] + G[4, 1],-2.0*Q[4, 5]==G[4, 7] + G[7, 4],2.0*Q[4, 4]==0.,0.333333333333333*Q[4, 5]==0.,G[4, 5] + G[5, 4]==0.,G[4, 4]==0]
        #constraints += [-2.4469506396*J0[0, 0] + 0.300563370000002*J1[0, 0] + 0.848238775*J2[0, 0] + 1.0653514616*J3[0, 0] + 1.0*Q[0, 0] - 1.000001==M[0, 0],-2.4469506396*J0[0, 3] - 2.4469506396*J0[3, 0] + 0.300563370000002*J1[0, 3] + 0.300563370000002*J1[3, 0] + 0.848238775*J2[0, 3] + 0.848238775*J2[3, 0] + 1.0653514616*J3[0, 3] + 1.0653514616*J3[3, 0] + 1.0*Q[0, 3] + 1.0*Q[3, 0]==M[0, 3] + M[3, 0],-2.4469506396*J0[3, 3] + 0.300563370000002*J1[3, 3] + 0.848238775*J2[3, 3] + 1.0653514616*J3[3, 3] + 1.0*Q[0, 9] + 1.0*Q[3, 3] + 1.0*Q[9, 0]==M[0, 9] + M[3, 3] + M[9, 0],Q[3, 9] + Q[9, 3]==M[3, 9] + M[9, 3],Q[9, 9]==M[9, 9],-1.02207*J0[0, 0] - 2.4469506396*J0[0, 2] - 2.4469506396*J0[2, 0] + 0.41097*J1[0, 0] + 0.300563370000002*J1[0, 2] + 0.300563370000002*J1[2, 0] + 0.39013*J2[0, 0] + 0.848238775*J2[0, 2] + 0.848238775*J2[2, 0] + 0.22097*J3[0, 0] + 1.0653514616*J3[0, 2] + 1.0653514616*J3[2, 0] + 1.0*Q[0, 2] + 1.0*Q[2, 0]==M[0, 2] + M[2, 0],-1.02207*J0[0, 3] - 2.4469506396*J0[2, 3] - 1.02207*J0[3, 0] - 2.4469506396*J0[3, 2] + 0.41097*J1[0, 3] + 0.300563370000002*J1[2, 3] + 0.41097*J1[3, 0] + 0.300563370000002*J1[3, 2] + 0.39013*J2[0, 3] + 0.848238775*J2[2, 3] + 0.39013*J2[3, 0] + 0.848238775*J2[3, 2] + 0.22097*J3[0, 3] + 1.0653514616*J3[2, 3] + 0.22097*J3[3, 0] + 1.0653514616*J3[3, 2] + 1.0*Q[0, 8] + 1.0*Q[2, 3] + 1.0*Q[3, 2] + 1.0*Q[8, 0]==M[0, 8] + M[2, 3] + M[3, 2] + M[8, 0],-1.02207*J0[3, 3] + 0.41097*J1[3, 3] + 0.39013*J2[3, 3] + 0.22097*J3[3, 3] + 1.0*Q[2, 9] + 1.0*Q[3, 8] + 1.0*Q[8, 3] + 1.0*Q[9, 2]==M[2, 9] + M[3, 8] + M[8, 3] + M[9, 2],Q[8, 9] + Q[9, 8]==M[8, 9] + M[9, 8],-1.02207*J0[0, 2] - 1.02207*J0[2, 0] - 2.4469506396*J0[2, 2] + 0.41097*J1[0, 2] + 0.41097*J1[2, 0] + 0.300563370000002*J1[2, 2] + 0.39013*J2[0, 2] + 0.39013*J2[2, 0] + 0.848238775*J2[2, 2] + 0.22097*J3[0, 2] + 0.22097*J3[2, 0] + 1.0653514616*J3[2, 2] + 1.0*Q[0, 6] + 1.0*Q[2, 2] + 1.0*Q[6, 0]==M[0, 6] + M[2, 2] + M[6, 0],-1.02207*J0[2, 3] - 1.02207*J0[3, 2] + 0.41097*J1[2, 3] + 0.41097*J1[3, 2] + 0.39013*J2[2, 3] + 0.39013*J2[3, 2] + 0.22097*J3[2, 3] + 0.22097*J3[3, 2] + 1.0*Q[2, 8] + 1.0*Q[3, 6] + 1.0*Q[6, 3] + 1.0*Q[8, 2]==M[2, 8] + M[3, 6] + M[6, 3] + M[8, 2],Q[6, 9] + Q[8, 8] + Q[9, 6]==M[6, 9] + M[8, 8] + M[9, 6],-1.02207*J0[2, 2] + 0.41097*J1[2, 2] + 0.39013*J2[2, 2] + 0.22097*J3[2, 2] + 1.0*Q[2, 6] + 1.0*Q[6, 2]==M[2, 6] + M[6, 2],Q[6, 8] + Q[8, 6]==M[6, 8] + M[8, 6],Q[6, 6]==M[6, 6],-0.45477*J0[0, 0] - 2.4469506396*J0[0, 1] - 2.4469506396*J0[1, 0] + 0.34515*J1[0, 0] + 0.300563370000002*J1[0, 1] + 0.300563370000002*J1[1, 0] + 0.1795*J2[0, 0] + 0.848238775*J2[0, 1] + 0.848238775*J2[1, 0] - 0.0698799999999999*J3[0, 0] + 1.0653514616*J3[0, 1] + 1.0653514616*J3[1, 0] + 1.0*Q[0, 1] + 1.0*Q[1, 0]==M[0, 1] + M[1, 0],-0.45477*J0[0, 3] - 2.4469506396*J0[1, 3] - 0.45477*J0[3, 0] - 2.4469506396*J0[3, 1] + 0.34515*J1[0, 3] + 0.300563370000002*J1[1, 3] + 0.34515*J1[3, 0] + 0.300563370000002*J1[3, 1] + 0.1795*J2[0, 3] + 0.848238775*J2[1, 3] + 0.1795*J2[3, 0] + 0.848238775*J2[3, 1] - 0.0698799999999999*J3[0, 3] + 1.0653514616*J3[1, 3] - 0.0698799999999999*J3[3, 0] + 1.0653514616*J3[3, 1] + 1.0*Q[0, 7] + 1.0*Q[1, 3] + 1.0*Q[3, 1] + 1.0*Q[7, 0]==M[0, 7] + M[1, 3] + M[3, 1] + M[7, 0],-0.45477*J0[3, 3] + 0.34515*J1[3, 3] + 0.1795*J2[3, 3] - 0.0698799999999999*J3[3, 3] + 1.0*Q[1, 9] + 1.0*Q[3, 7] + 1.0*Q[7, 3] + 1.0*Q[9, 1]==M[1, 9] + M[3, 7] + M[7, 3] + M[9, 1],Q[7, 9] + Q[9, 7]==M[7, 9] + M[9, 7],-1.02207*J0[0, 1] - 0.45477*J0[0, 2] - 1.02207*J0[1, 0] - 2.4469506396*J0[1, 2] - 0.45477*J0[2, 0] - 2.4469506396*J0[2, 1] + 0.41097*J1[0, 1] + 0.34515*J1[0, 2] + 0.41097*J1[1, 0] + 0.300563370000002*J1[1, 2] + 0.34515*J1[2, 0] + 0.300563370000002*J1[2, 1] + 0.39013*J2[0, 1] + 0.1795*J2[0, 2] + 0.39013*J2[1, 0] + 0.848238775*J2[1, 2] + 0.1795*J2[2, 0] + 0.848238775*J2[2, 1] + 0.22097*J3[0, 1] - 0.0698799999999999*J3[0, 2] + 0.22097*J3[1, 0] + 1.0653514616*J3[1, 2] - 0.0698799999999999*J3[2, 0] + 1.0653514616*J3[2, 1] + 1.0*Q[0, 5] + 1.0*Q[1, 2] + 1.0*Q[2, 1] + 1.0*Q[5, 0]==M[0, 5] + M[1, 2] + M[2, 1] + M[5, 0],-1.02207*J0[1, 3] - 0.45477*J0[2, 3] - 1.02207*J0[3, 1] - 0.45477*J0[3, 2] + 0.41097*J1[1, 3] + 0.34515*J1[2, 3] + 0.41097*J1[3, 1] + 0.34515*J1[3, 2] + 0.39013*J2[1, 3] + 0.1795*J2[2, 3] + 0.39013*J2[3, 1] + 0.1795*J2[3, 2] + 0.22097*J3[1, 3] - 0.0698799999999999*J3[2, 3] + 0.22097*J3[3, 1] - 0.0698799999999999*J3[3, 2] + 1.0*Q[1, 8] + 1.0*Q[2, 7] + 1.0*Q[3, 5] + 1.0*Q[5, 3] + 1.0*Q[7, 2] + 1.0*Q[8, 1]==M[1, 8] + M[2, 7] + M[3, 5] + M[5, 3] + M[7, 2] + M[8, 1],Q[5, 9] + Q[7, 8] + Q[8, 7] + Q[9, 5]==M[5, 9] + M[7, 8] + M[8, 7] + M[9, 5],-1.02207*J0[1, 2] - 1.02207*J0[2, 1] - 0.45477*J0[2, 2] + 0.41097*J1[1, 2] + 0.41097*J1[2, 1] + 0.34515*J1[2, 2] + 0.39013*J2[1, 2] + 0.39013*J2[2, 1] + 0.1795*J2[2, 2] + 0.22097*J3[1, 2] + 0.22097*J3[2, 1] - 0.0698799999999999*J3[2, 2] + 1.0*Q[1, 6] + 1.0*Q[2, 5] + 1.0*Q[5, 2] + 1.0*Q[6, 1]==M[1, 6] + M[2, 5] + M[5, 2] + M[6, 1],Q[5, 8] + Q[6, 7] + Q[7, 6] + Q[8, 5]==M[5, 8] + M[6, 7] + M[7, 6] + M[8, 5],Q[5, 6] + Q[6, 5]==M[5, 6] + M[6, 5],-0.45477*J0[0, 1] - 0.45477*J0[1, 0] - 2.4469506396*J0[1, 1] + 0.34515*J1[0, 1] + 0.34515*J1[1, 0] + 0.300563370000002*J1[1, 1] + 0.1795*J2[0, 1] + 0.1795*J2[1, 0] + 0.848238775*J2[1, 1] - 0.0698799999999999*J3[0, 1] - 0.0698799999999999*J3[1, 0] + 1.0653514616*J3[1, 1] + 1.0*Q[0, 4] + 1.0*Q[1, 1] + 1.0*Q[4, 0]==M[0, 4] + M[1, 1] + M[4, 0],-0.45477*J0[1, 3] - 0.45477*J0[3, 1] + 0.34515*J1[1, 3] + 0.34515*J1[3, 1] + 0.1795*J2[1, 3] + 0.1795*J2[3, 1] - 0.0698799999999999*J3[1, 3] - 0.0698799999999999*J3[3, 1] + 1.0*Q[1, 7] + 1.0*Q[3, 4] + 1.0*Q[4, 3] + 1.0*Q[7, 1]==M[1, 7] + M[3, 4] + M[4, 3] + M[7, 1],Q[4, 9] + Q[7, 7] + Q[9, 4]==M[4, 9] + M[7, 7] + M[9, 4],-1.02207*J0[1, 1] - 0.45477*J0[1, 2] - 0.45477*J0[2, 1] + 0.41097*J1[1, 1] + 0.34515*J1[1, 2] + 0.34515*J1[2, 1] + 0.39013*J2[1, 1] + 0.1795*J2[1, 2] + 0.1795*J2[2, 1] + 0.22097*J3[1, 1] - 0.0698799999999999*J3[1, 2] - 0.0698799999999999*J3[2, 1] + 1.0*Q[1, 5] + 1.0*Q[2, 4] + 1.0*Q[4, 2] + 1.0*Q[5, 1]==M[1, 5] + M[2, 4] + M[4, 2] + M[5, 1],Q[4, 8] + Q[5, 7] + Q[7, 5] + Q[8, 4]==M[4, 8] + M[5, 7] + M[7, 5] + M[8, 4],Q[4, 6] + Q[5, 5] + Q[6, 4]==M[4, 6] + M[5, 5] + M[6, 4],-0.45477*J0[1, 1] + 0.34515*J1[1, 1] + 0.1795*J2[1, 1] - 0.0698799999999999*J3[1, 1] + 1.0*Q[1, 4] + 1.0*Q[4, 1]==M[1, 4] + M[4, 1],Q[4, 7] + Q[7, 4]==M[4, 7] + M[7, 4],Q[4, 5] + Q[5, 4]==M[4, 5] + M[5, 4],Q[4, 4]==M[4, 4]]

        prob = cp.Problem(cp.Minimize(1),constraints)
        prob.solve(verbose=False)

        # Print result
        #print("status:", prob.status)
        #print("The optimal value is", prob.value)
        #print("A solution Q is")
        #print(Q.value)
        #print(J.value)

        #V_sol = b1.T @ Q.value @ b1

        #print(V_sol)




## apply control based on barrier function
def Apply_control(self):
    x = 0




def main():
    rospy.init_node('sum_of_squares_node',anonymous=True)
    ss = SOS()
    rospy.spin()
if __name__ == '__main__':
    main()