#!/usr/bin/env python

print("Start importing libarries ...")

import rospy

# ROS libraries -- add to include file
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
from nav_msgs.msg import OccupancyGrid, Path

from geometry_msgs.msg import PoseStamped, TransformStamped, PolygonStamped, Point32, Point

import sensor_msgs.point_cloud2 as pc2

import tf
import tf2_ros as tf2
import tf_conversions

from tf import transformations as trafo

import time

import numpy as np
import numpy.linalg as LA

import sys, warnings, copy

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
        obs.center_position += np.array([0.0, 0.0])
        obs.orientation += -30./180*pi

        return obs

    if obstacle_number== 0:
        # Tool-Trolley
        obs = Cuboid(axes_length=[0.8,0.3], 
                     position_3d=[0, 0, 0], orientation_3d=[0, 0, 0, 0])
        return obs

    if obstacle_number==1:
        # Human
        obs = Ellipse(axes_length=[0.6, 0.3], p=[2,2], 
                      position_3d=[0, 0, 0], orientation_3d=[0, 0, 0, 0])
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

        if n_obstacles is None:
            obstacle_topic_names = ['obstacle', 'hat']
            n_obstacles = len(obstacle_topic_names)
        else:
            obstacle_topic_names = ['obstacles'+str(ii) for ii in range(n_obstacles)]

        self.awaiting_msg_obstacles = [True]*n_obstacles
        self.awaiting_msg_agent = True

        self.data_agent = None        
        self.data_obstacle = [None]*(n_obstacles+1) # + boundary

        # Use ridgeback-time stamp for messages
        self.ridgeback_stamp = None
        self.awaiting_msg_imu = True
        rospy.Subscriber("/imu/data", Imu, self.callback_imu) # get timestamp

        rospy.Subscriber("/vrpn_client_node/ridgeback/pose", PoseStamped, self.callback_agent_optitrack)
        
        # rospy.Subscriber("/vrpn_client_node/robot/pose", PoseStamped, self.callback_agent_optitrack)

        for oo, obs_str in izip(range(n_obstacles), obstacle_topic_names):
            topic_str = "vrpn_client_node/"+ obs_str +"/pose"
            print("Topic " + str(oo) + " :" + topic_str)
            rospy.Subscriber(topic_str, PoseStamped, self.callback_obstacle_optitrack, oo)

            # import pdb; pdb.set_trace() # BREAKPOINT        
        pub_test = rospy.Publisher('test_point_cloud', PointCloud2 , queue_size=5)
        self.pub_occGrid = rospy.Publisher('occupancy_grid', OccupancyGrid , queue_size=5)
        
        # Create obstacles and publisher
        self.obstacles = []
        self.pub_obstacles = []

        for oo in range(n_obstacles):
            self.pub_obstacles.append(rospy.Publisher("polygon_obstacle"+str(oo), PolygonStamped, queue_size=5))
            self.obstacles.append(define_obstacles_lab_south(oo))

        # self.pub_obstacles.append(rospy.Publisher("polygon_wall", PolygonStamped, queue_size=5))
        self.pub_obstacles.append(rospy.Publisher("polygon_wall", Path, queue_size=10))
        self.obstacles.append(define_obstacles_lab_south(-1))

        # START LOOP
        while ((any(self.awaiting_msg_obstacles) or self.awaiting_msg_agent)
               and not rospy.is_shutdown()):
            # import pdb; pdb.set_trace() # BREAKPOINT        
            print("Waiting for messages ...")
            for ii in range(n_obstacles):
                if self.awaiting_msg_obstacles[ii]:
                    print('.. awaiting obstacle', obstacle_topic_names[ii])
            if self.awaiting_msg_agent:
                print('.. awaiting agent')

            rospy.sleep(0.25)
        
        self.tf_listener = tf.TransformListener()
        self.tf_broadcast = tf2.TransformBroadcaster()

        rate = rospy.Rate(50) # 10 Hz

        print("Entering main loop....")
        # Enter main loop
        while not rospy.is_shutdown():
            if not self.update_transformations():
                continue

            # Update obstacle position
            for oo in range(len(self.obstacles)-1):

                # orientation to
                self.obstacles[oo].update_position_and_orientation(
                    position=[self.data_obstacle[oo].pose.position.x, 
                              self.data_obstacle[oo].pose.position.y, 
                              self.data_obstacle[oo].pose.position.z],
                    orientation=[self.data_obstacle[oo].pose.orientation.w,
                                 self.data_obstacle[oo].pose.orientation.x,
                                 self.data_obstacle[oo].pose.orientation.y,
                                 self.data_obstacle[oo].pose.orientation.z])

            for oo in range(len(self.obstacles)-1):
                self.obstacles[oo].draw_obstacle()

                obstacle_polygon = PolygonStamped()
                obstacle_polygon.header.frame_id = "world_lab"
                obstacle_polygon.header.stamp = self.ridgeback_stamp
                for pp in range(self.obstacles[oo].x_obs.shape[0]):
                    # point = Point32()
                    # point.x = float(self.obstacles[oo].x_obs[pp, 0])
                    # point.y = float(self.obstacles[oo].x_obs[pp, 1])
                    # obstacle_polygon.polygon.points.append(point)
                    point = Point32(self.obstacles[oo].x_obs[pp, 0], self.obstacles[oo].x_obs[pp, 1], 0)
                    
                    obstacle_polygon.polygon.points.append(point)
                # print('poly', obstacle_polygon)
                # obstacle_polygon.polygon.points.append(point)

                # point = Point32()
                # point.x = float(self.obstacles[oo].reference_point[0])
                # point.y = float(self.obstacles[oo].reference_point[1])
                # point.z = float(-0.5)

                # obstacle_polygon.polygon.points[-2] = point

                self.pub_obstacles[oo].publish(obstacle_polygon)

            
            # Only last obstacle in the list is a path
            for oo in [len(self.obstacles)-1]:
                self.obstacles[oo].draw_obstacle()

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
                
            lock.acquire() ##### LOCK ACQUIRE ######

            # import pdb; pdb.set_trace() # BREAKPOINT      
            # tf_broadcast.sendTransform( (0,0,-3), tf.transformations.quaternion_from_euler(0, 0, 0),
                                        # rospy.Time.now(), 
                                        # "odom", "world")

            lock.release() ##### LOCK RELEASE ######

            print("Ouuups. I did it again")
                
            rate.sleep()

        print("Exiting main loop")



    def update_transformations(self, use_ridgeback_stamp=True):
        if use_ridgeback_stamp:
            time_stamp = self.ridgeback_stamp
        else:
            print("Warning: A time synchronization problem might occur due to clock synchro")
            print("we advice using the recoreded ridgeback time")

        try:
            (self.trans_odom2base, self.rot_odom2base) = self.tf_listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False         # Return False if not executed correctly

        transform = trafo.concatenate_matrices(trafo.translation_matrix(self.trans_odom2base), trafo.quaternion_matrix(self.rot_odom2base))
        inversed_transform = trafo.inverse_matrix(transform)

        # TODO inverse in other sence to have optitrack as 'master'
        trafo_odom2world_child = TransformStamped()

        # trafo_odom2world_child.header.stamp = rospy.Time.now()
        trafo_odom2world_child.header.stamp = time_stamp
        trafo_odom2world_child.header.frame_id = "base_link_clone"
        trafo_odom2world_child.child_frame_id = "odom"

        trafo_odom2world_child.transform.translation.x = self.trans_odom2base[0]
        trafo_odom2world_child.transform.translation.y = self.trans_odom2base[1]
        trafo_odom2world_child.transform.translation.z = self.trans_odom2base[2]

        trafo_odom2world_child.transform.rotation.x = self.rot_odom2base[0]
        trafo_odom2world_child.transform.rotation.y = self.rot_odom2base[1]
        trafo_odom2world_child.transform.rotation.z = self.rot_odom2base[2]
        trafo_odom2world_child.transform.rotation.w = self.rot_odom2base[3]
        self.tf_broadcast.sendTransform(trafo_odom2world_child)

        trafo_optitrack2base_link_clone = TransformStamped()
        trafo_optitrack2base_link_clone.header.stamp = time_stamp
        # trafo_optitrack2base_link_clone.header.stamp = rospy.Time.now()
        trafo_optitrack2base_link_clone.header.frame_id = "world_optitrack"
        trafo_optitrack2base_link_clone.child_frame_id = "base_link_clone"
        # import pdb; pdb.set_trace() # BREAKPOINT      
        trafo_optitrack2base_link_clone.transform.translation.x = self.data_agent.pose.position.x
        trafo_optitrack2base_link_clone.transform.translation.y = self.data_agent.pose.position.y
        trafo_optitrack2base_link_clone.transform.translation.z = 0

        trafo_optitrack2base_link_clone.transform.rotation.x = self.data_agent.pose.orientation.x
        trafo_optitrack2base_link_clone.transform.rotation.y = self.data_agent.pose.orientation.y
        trafo_optitrack2base_link_clone.transform.rotation.z = self.data_agent.pose.orientation.z
        trafo_optitrack2base_link_clone.transform.rotation.w = self.data_agent.pose.orientation.w
        self.tf_broadcast.sendTransform(trafo_optitrack2base_link_clone)


        # LAB 2 optitrack
        trafo_lab2optitrack = TransformStamped()
        trafo_lab2optitrack.header.stamp = time_stamp
        # trafo_lab2optitrack.header.stamp = rospy.Time.now()
        trafo_lab2optitrack.header.frame_id = "world_lab"
        trafo_lab2optitrack.child_frame_id = "world_optitrack"
        trafo_lab2optitrack.transform.translation.x = 1.0
        trafo_lab2optitrack.transform.translation.y = -3
        trafo_lab2optitrack.transform.translation.z = 0

        q = tf.transformations.quaternion_from_euler(0, 0, -9./180*pi)
        trafo_lab2optitrack.transform.rotation.x = q[0]
        trafo_lab2optitrack.transform.rotation.y = q[1]
        trafo_lab2optitrack.transform.rotation.z = q[2]
        trafo_lab2optitrack.transform.rotation.w = q[3]
        self.tf_broadcast.sendTransform(trafo_lab2optitrack)

        trafo_0 = TransformStamped()
        trafo_0.header.stamp = time_stamp
        trafo_0.header.frame_id = "world_lab"
        trafo_0.child_frame_id = "world"
        trafo_0.transform.rotation.w = 1
        self.tf_broadcast.sendTransform(trafo_0)

        return True



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


if __name__==('__main__'):
    print("Starting node")
    
    if len(sys.argv)>=2:
        n_obstacles = int(sys.argv[1])
    else:
        # print("WARNING - no obstacle number was given. Default value n=0 is chosen")
        n_obstacles = None

    try:
        ObstacleAvoidance_optitrack_int = ObstacleAvoidance_optitrack(n_obstacles)
    except rospy.ROSInterruptException:
        pass        
    # Instance_SensoryTreatment.listener()

print("")
print("")
print("Finished cleanly...")
