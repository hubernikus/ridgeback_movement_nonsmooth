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


class TransformPublisher_odom(): 

    def __init__(self):
        print("")
        print("Initializiation starting.\n")

        rospy.init_node('transform_publisher', anonymous=True)

        # Use ridgeback-time stamp for messages
        self.ridgeback_stamp = None
        self.awaiting_msg_imu = True
        rospy.Subscriber("/imu/data", Imu, self.callback_imu) # get timestamp

        # START LOOP
        while (self.awaiting_msg_imu:
               and not rospy.is_shutdown()):
            # import pdb; pdb.set_trace() # BREAKPOINT        
            print("Waiting for stamp message ...")
            rospy.sleep(0.25)
        
        self.tf_listener = tf.TransformListener()
        self.tf_broadcast = tf2.TransformBroadcaster()

        rate = rospy.Rate(50) # 10 Hz
        print("Entering main loop....")
        while not rospy.is_shutdown():
            if use_ridgeback_stamp:
                time_stamp = self.ridgeback_stamp
            else:
                print("Warning: A time synchronization problem might occur due to clock synchro")
                print("we advice using the recoreded ridgeback time")

            try:
                (self.trans_odom2base, self.rot_odom2base) = self.tf_listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue         # Return False if not executed correctly

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

            # return True

            rate.sleep()

        
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
