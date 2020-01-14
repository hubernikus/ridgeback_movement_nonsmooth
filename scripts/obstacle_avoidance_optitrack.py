#!/usr/bin/env python

print("Start importing libarries ...")

import rospy

# ROS libraries -- add to include file
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
from nav_msgs.msg import OccupancyGrid, Path

from geometry_msgs.msg import PoseStamped, TransformStamped, PolygonStamped, Point32, Point, Twist

import sensor_msgs.point_cloud2 as pc2

import tf
import tf2_ros as tf2
import tf_conversions

from tf import transformations as trafo

import time

import numpy as np
import numpy.linalg as LA

import sys, signal, warnings, copy

from math import pi

if (sys.version_info < (3, 0)):
    from itertools import izip

# import quaternion

import json

# Protect data during and from callback
from threading import Lock
lock = Lock()

# DEBUGGING
import matplotlib.pyplot as plt
plt.ion()

# import pdb; pdb.set_trace() # BREAKPOINT

# OBSTACLE AVOIDANCE LIBRARY
path_obstacle_avoidance = "/home/lukas/catkin_ws/src/ridgeback_movement/scripts/dynamic_obstacle_avoidance/src"
if not path_obstacle_avoidance in sys.path:
    sys.path.append(path_obstacle_avoidance)

from dynamic_obstacle_avoidance.obstacle_avoidance.obstacle import *
from dynamic_obstacle_avoidance.obstacle_avoidance.obstacle_polygon import Cuboid, Polygon
from dynamic_obstacle_avoidance.obstacle_avoidance.linear_modulations import * 

print("... finished importing libraries")

def define_obstacles_lab_south(obstacle_number):
    # TODO create json file for obstacle description.
    if obstacle_number ==-2:
        edge_points = np.array((
            [[-1.4,-2.2,-2.2, -1.4],
             [ 0.2, 0.2,-1.0,-1.0]]))

        frame_id = "world_lab"
        obs = Polygon(edge_points=edge_points, is_boundary=True)
        
        # Displacement
        obs.center_position += np.array([0.0, 0.0])
        obs.orientation += 0./180*pi

        return obs

    if obstacle_number ==-1:
        edge_points = np.array((
            [4.0, 1.0, 1.0, 1.0, 0.0,0.0,-4.0,-4.0,-2.5,-2.5, 0.0, 0.0, 4.0],
            [0.0, 0.0, 1.0, 1.0, 1.0,1.5,1.5,-2.0,-2.0,-3.5,-3.5,-3.0,-3.0 ]))

        # edge_points = np.array((
            # [100.0,-100.0,-100.0, 100.0],
            # [100.0, 100.0, -100.0,-100.0]))

        frame_id = "world_lab"
        obs = Polygon(edge_points=edge_points, is_boundary=True)
        
        # Displacement
        obs.center_position += np.array([0.8, -1.3])
        obs.orientation += -56./180*pi

        return obs

    if obstacle_number== 0:
        # Tool-Trolley
        obs = Cuboid(axes_length=[0.8,0.3])
                     
        return obs

    if obstacle_number==1:
        # Human
        obs = Ellipse(axes_length=[0.6, 0.3], p=[2,2]) 

        return obs

    return 

def rosquat2array(q):
    return np.array([q.w, q.x, q.y, q.z])
    

class ObstacleAvoidance_optitrack(): 
    robot_radius = 0.3 # Simplify the robot as a circle
    room_dimensions = [20,10,3] # Robot laboratory dimensions [x, y, z]

    height_robot = 1.3
    robot_clearance_height = 1.5 # easily pass objects with a height inferor to this

    n_rows_lidar = 16 
    ang_resolution = 600

    def __init__(self, n_obstacles=None):
        print("")
        print("Initializiation starting.\n")

        rospy.init_node('Sensory_Treatement', anonymous=True)
        self.node_is_active = True

        if n_obstacles is None:
            self.obstacle_topic_names = ['obstacle', 'hat']
            self.n_obstacles = len(self.obstacle_topic_names)
        else:
            self.n_obstacles = n_obstacles
            self.obstacle_topic_names = ['obstacles'+str(ii) for ii in range(self.n_obstacles)]

        # Callbacks and Subscribers
        self.awaiting_msg_obstacles = [True]*self.n_obstacles
        self.awaiting_msg_agent = True

        self.data_agent = None        
        self.data_obstacle = [None]*(self.n_obstacles+1) # + boundary

        # Use ridgeback-time stamp for messages
        self.ridgeback_stamp = None
        self.awaiting_msg_imu = True
        rospy.Subscriber("/imu/data", Imu, self.callback_imu) # get timestamp

        self.topic_agent = "/vrpn_client_node/ridgeback/pose"
        rospy.Subscriber(self.topic_agent, PoseStamped, self.callback_agent_optitrack)
        self.topic_obstacles = [0]*self.n_obstacles
        for oo, obs_str in izip(range(self.n_obstacles), self.obstacle_topic_names):
            self.topic_obstacles[oo] = "vrpn_client_node/"+ obs_str +"/pose"
            print("Topic " + str(oo) + " :" + self.topic_obstacles[oo])
            rospy.Subscriber(self.topic_obstacles[oo], PoseStamped, self.callback_obstacle_optitrack, oo)


        # Publisher
        # pub_test = rospy.Publisher('test_point_cloud', PointCloud2 , queue_size=5)
        # self.pub_occGrid = rospy.Publisher('occupancy_grid', OccupancyGrid , queue_size=5)
        self.pub_modVel = rospy.Publisher('occupancy_grid', OccupancyGrid , queue_size=5)
        self.pub_eeVel_modulated = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Create obstacles and publisher
        self.obstacles = []
        self.pub_obstacles = []
        self.pub_obstacles_axis = []

        for oo in range(self.n_obstacles):
            self.pub_obstacles.append(rospy.Publisher("polygon_obstacle"+str(oo), PolygonStamped, queue_size=5))
            self.pub_obstacles_axis.append(rospy.Publisher("axis_obstacle"+str(oo), PoseStamped, queue_size=5))
            self.obstacles.append(define_obstacles_lab_south(oo))

        # self.pub_obstacles.append(rospy.Publisher("polygon_wall", PolygonStamped, queue_size=5))
        self.pub_obstacles.append(rospy.Publisher("polygon_wall", Path, queue_size=10))
        self.obstacles.append(define_obstacles_lab_south(-1))

        self.tf_listener = tf.TransformListener()
        self.tf_broadcast = tf2.TransformBroadcaster()

        self.rate = rospy.Rate(50) # 10 Hz

        self.pos_obstacles = [0]*self.n_obstacles        
        self.orient_obstacles = [0]*self.n_obstacles

        self.obstacles[-1].draw_obstacle() # Draw boundary

        self.attractor = np.array([0,0]) # Define default attractor

        while ((any(self.awaiting_msg_obstacles) or self.awaiting_msg_agent)
               and not rospy.is_shutdown()):
            print("Waiting for messages ...")
            for ii in range(self.n_obstacles):
                if self.awaiting_msg_obstacles[ii]:
                    print('.. awaiting obstacle', self.obstacle_topic_names[ii])
            if self.awaiting_msg_agent:
                print('.. awaiting agent')
            rospy.sleep(0.25)
            
    def run(self):
        print("Starting main loop....")
        # START MAIN LOOP
        while not rospy.is_shutdown():
            lock.acquire() ##### LOCK ACQUIRE ######
            try:
                (self.pos_agent, self.orient_agent) = self.tf_listener.lookupTransform('/world_optitrack', '/ridgeback', rospy.Time(0))
                
                for oo in range(self.n_obstacles):
                    (self.pos_obstacles[oo], self.orient_obstacles[oo]) = self.tf_listener.lookupTransform('/world_optitrack', '/'+ self.obstacle_topic_names[oo], rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                lock.release(); self.rate.sleep(); continue

            # Update obstacle position
            for oo in range(len(self.obstacles)-1):
                # orientation to
                euler = tf.transformations.euler_from_quaternion(self.orient_obstacles[oo])
                
                self.obstacles[oo].update_position_and_orientation(
                    position=[self.pos_obstacles[oo][0],self.pos_obstacles[oo][1]],
                    orientation=euler[0])
                self.obstacles[oo].draw_obstacle()
                # print('obs #{}'.format(oo), self.obstacles[oo].center_position)
            
            # Publish obstacle visualization
            for oo in range(len(self.obstacles)-1):
                obstacle = PoseStamped()
                obstacle.header.frame_id = "world_lab"
                obstacle.header.stamp = self.ridgeback_stamp
                obstacle.pose.position.x, obstacle.pose.position.y = self.obstacles[oo].center_position[0], self.obstacles[oo].center_position[1]
                # obstacle.pose.position.z = 0
                q = tf.transformations.quaternion_from_euler(0, 0, self.obstacles[oo].orientation)
                obstacle.pose.orientation.w = q[3]
                obstacle.pose.orientation.x = q[0]
                obstacle.pose.orientation.y = q[1]
                obstacle.pose.orientation.z = q[2]
                self.pub_obstacles_axis[oo].publish(obstacle)
                
                obstacle_polygon = PolygonStamped()
                obstacle_polygon.header.frame_id = "world_lab"
                obstacle_polygon.header.stamp = self.ridgeback_stamp
                for pp in range(self.obstacles[oo].x_obs.shape[0]):
                    point = Point32(self.obstacles[oo].x_obs[pp, 0], self.obstacles[oo].x_obs[pp, 1], 0)
                    obstacle_polygon.polygon.points.append(point)
                
                self.pub_obstacles[oo].publish(obstacle_polygon)
            
            # Only last obstacle in the list is a path
            for oo in [len(self.obstacles)-1]:
                obstacle_path = Path()
                obstacle_path.header.frame_id = "world_lab"
                # obstacle_path.header.stamp = rospy.Time.now()
                obstacle_path.header.stamp = self.ridgeback_stamp
                for pp in range(self.obstacles[oo].x_obs.shape[0]):
                    point_pose = PoseStamped()
                    point_pose.header.frame_id = obstacle_path.header.frame_id
                    point_pose.header.stamp = obstacle_path.header.stamp
                    point_pose.pose.position.x = float(self.obstacles[oo].x_obs[pp, 0])
                    point_pose.pose.position.y = float(self.obstacles[oo].x_obs[pp, 1])
                    obstacle_path.poses.append(point_pose)
                # print('path # ' +  str(oo) )
                self.pub_obstacles[oo].publish(obstacle_path)
                


            # Modulation
            # publish_linear_ds(self, position, attractor, max_vel=0.07, slow_down_dist=0.1, publish_ros_message=False)
            # publish_modulated_ds(self, position, velocity, obstacles, max_vel=0.07, slow_down_dist=0.1, publish_ros_message=True)

            ##### Obstacle Avoidance Algorithm #####
            linear_ds = self.publish_linear_ds(position=self.pos_agent, attractor=self.attractor)
            modulated_ds = self.publish_modulated_ds(self.pos_agent, linear_ds, self.obstacles)
            ##### Finihsed #####

            lock.release() ##### LOCK RELEASE ######

            print("Loop finihsed")
            self.rate.sleep()

        print("Exiting main loop")

    def publish_linear_ds(self, position, attractor, max_vel=0.07, slow_down_dist=0.1, publish_ros_message=False):
        position = np.array([position[0], position[1]])
        linear_ds = attractor-position
        linear_ds = self.limit_velocity(linear_ds, position, attractor)

        if publish_ros_message:
            raise NotImplementedError()

        return linear_ds

    def publish_modulated_ds(self, position, velocity, obstacles, max_vel=0.07, slow_down_dist=0.1, publish_ros_message=True):
        if len(obstacles): # nonzero
            position = np.array([position[0], position[1]])
            velocity = obs_avoidance_interpolation_moving(position, velocity, obstacles)
            # modulated_ds = linear_ds
            velocity = self.limit_velocity(velocity, position, self.attractor)

        if publish_ros_message:
            self.msg_vel = Twist()
            self.msg_vel.linear.x, self.msg_vel.linear.y, self.msg_vel.linear.z = velocity[0], velocity[1], 0
            self.pub_eeVel_modulated.publish(self.msg_vel)
        
        return velocity


    @staticmethod 
    def limit_velocity(velocity, position, final_position, max_vel=0.07, slow_down_dist=0.1):
        # move to dynamical system class
        dist = final_position-position
        dist_norm = np.linalg.norm(dist)
        vel_norm = np.linalg.norm(velocity)
        
        if not dist_norm or not vel_norm:
            vel = np.zeros(3)
        elif dist_norm < slow_down_dist:
            vel = velocity/vel_norm*max_vel*dist_norm/slow_down_dist
        else:
            vel = velocity/vel_norm*max_vel
        return vel


    def callback_obstacle_optitrack(self, data, obstacle_number=0):
        lock.acquire()

        self.data_obstacle[obstacle_number] = data

        if self.awaiting_msg_obstacles[obstacle_number]:
            self.awaiting_msg_obstacles[obstacle_number] = False
            print("Got first obstacle #{}".format(obstacle_number))

        lock.release()


    def callback_agent_optitrack(self, data):
        lock.acquire()

        self.data_agent = data
        if self.awaiting_msg_agent:
            self.awaiting_msg_agent = False
            print("Got 1st agent pose")

        lock.release()        

        
    def callback_imu(self, data):
        # There is a problem with the clocks ridgeback/computer. 
        # The IMU stamp is used for every time!
        lock.acquire()
        if self.awaiting_msg_imu:
            self.awaiting_msg_imu = False
            print("Got 1st IMU")

        # self.imu_stamp = data.header.stamp 
        self.ridgeback_stamp = data.header.stamp 
        lock.release()

    def control_c_handler(self, sig, frame):
        rospy.signal_shutdown('Caught ctrl-c by user. Shutdown is initiated ...')
        print('\nCaught ctrl-c by user. Shutdown is initiated ...')
        self.shutdown()

    def shutdown(self):
        # lock.acquire()
        if not self.node_is_active:
            # lock.release(); return
            return

        # self.recorder.stop_recording(self.file_name)
        msg_vel = Twist()
        self.msg_vel.linear.x, self.msg_vel.linear.y, self.msg_vel.linear.z = 0,0,0
        self.msg_vel.angular.x, self.msg_vel.angular.y, self.msg_vel.angular.z = 0,0,0
        for ii in range(10):
            # Zero velocity
            # self.pub_(Bool(True))
            self.pub_eeVel_modulated.publish(msg_vel)
            rospy.sleep(0.02)
        

        self.node_is_active = False
        # lock.release()
        print("\nShutdown succesful.")


if __name__==('__main__'):
    print("Starting node")

    
    
    if len(sys.argv)>=2:
        n_obstacles = int(sys.argv[1])
    else:
        # print("WARNING - no obstacle number was given. Default value n=0 is chosen")
        n_obstacles = None

    try:
        ObstacleAvoidance_optitrack_int = ObstacleAvoidance_optitrack(n_obstacles)
        signal.signal(signal.SIGINT, 
                      ObstacleAvoidance_optitrack_int.control_c_handler)
        ObstacleAvoidance_optitrack_int.run()
    except rospy.ROSInterruptException:
        pass        
    # Instance_SensoryTreatment.listener()

print("")
print("Finished cleanly...\n")
