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
from obstacle_avoidance_optitrack import quaternion_from_2vectors

# OBSTACLE AVOIDANCE LIBRARY
path_obstacle_avoidance = "/home/lukas/catkin_ws/src/ridgeback_movement/scripts/dynamic_obstacle_avoidance/src"
if not path_obstacle_avoidance in sys.path:
    sys.path.append(path_obstacle_avoidance)

from dynamic_obstacle_avoidance.obstacle_avoidance.obstacle import *
from dynamic_obstacle_avoidance.obstacle_avoidance.obstacle_polygon import Cuboid, Polygon
from dynamic_obstacle_avoidance.obstacle_avoidance.linear_modulations import * 

print("... finished importing libraries")


class TransformPublisher_odom(): 
    def __init__(self, agent_topic_names=['ridgeback2', 'ridgeback1']):
        print("")
        print("Initializiation starting.\n")

        rospy.init_node('transform_publisher', anonymous=True)

        self.n_agent_topics = len(agent_topic_names)
        self.awaiting_msg_agent = [True]*self.n_agent_topics
        self.data_agent = [None]*self.n_agent_topics

        for ii in range(self.n_agent_topics):
            rospy.Subscriber("/vrpn_client_node/"+ agent_topic_names[ii] +"/pose", PoseStamped, self.callback_agent_optitrack, ii)

        # Use ridgeback-time stamp for messages
        self.ridgeback_stamp = None
        self.awaiting_msg_imu = True
        rospy.Subscriber("/imu/data", Imu, self.callback_imu) # get timestamp

        self.tf_listener = tf.TransformListener()
        self.tf_broadcast = tf2.TransformBroadcaster()

        # START LOOP
        while ((self.awaiting_msg_imu or np.sum(self.awaiting_msg_agent))
               and not rospy.is_shutdown()):
            # import pdb; pdb.set_trace() # BREAKPOINT        
            print("Waiting for stamp message ...")
            rospy.sleep(0.25)

        rate = rospy.Rate(100) # Hz
        print("Entering main loop....")

        while not rospy.is_shutdown():
            # lock.acquire()
            use_ridgeback_stamp = True
            if use_ridgeback_stamp:
                time_stamp = self.ridgeback_stamp
            else:
                print("Warning: A time synchronization problem might occur due to clock asynchronization")
                print("we advice using the recorded ridgeback time.")

            try:
                (self.trans_odom2base, self.rot_odom2base) = self.tf_listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

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

            # Optitrack Data - Smoothened / quaternion is calculated throuh multiple markers
            marker_positions = np.zeros((3, self.n_agent_topics))
            for ii in range(self.n_agent_topics):
                marker_positions[:, ii] = [self.data_agent[ii].pose.position.x, self.data_agent[ii].pose.position.y, self.data_agent[ii].pose.position.z]

            center_position = np.sum(marker_positions, axis=1)/self.n_agent_topics

            if self.n_agent_topics==1:
                quat = [self.data_agent[0].pose.orientation.w, self.data_agent[0].pose.orientation.x, self.data_agent[0].pose.orientation.y, self.data_agent.pose.orientation.z]

            elif self.n_agent_topics==2:
                y_dir_ridgeback = marker_positions[:, 1] - marker_positions[:, 0]
                y_dir_ridgeback[2] = 0 # remove z deviation
                # x_direction = np.cross(y_direction, [0, 0, 1])

                y_dir_init = np.array([0, 1, 0])
                quat = quaternion_from_2vectors(y_dir_init, y_dir_ridgeback)
            else:
                raise NotImplementedError()
            
            trafo_optitrack2base_link_clone = TransformStamped() 
            trafo_optitrack2base_link_clone.header.stamp = time_stamp # Time delay of clocks - use ridgeback-message stamp to bridge this
            # trafo_optitrack2base_link_clone.header.stamp = rospy.Time.now()
            trafo_optitrack2base_link_clone.header.frame_id = "world_optitrack"
            trafo_optitrack2base_link_clone.child_frame_id = "base_link_clone"
            # import pdb; pdb.set_trace() # BREAKPOINT      
            trafo_optitrack2base_link_clone.transform.translation.x = center_position[0]
            trafo_optitrack2base_link_clone.transform.translation.y = center_position[1]
            trafo_optitrack2base_link_clone.transform.translation.z = 0

            trafo_optitrack2base_link_clone.transform.rotation.w = quat[0]
            trafo_optitrack2base_link_clone.transform.rotation.x = quat[1]
            trafo_optitrack2base_link_clone.transform.rotation.y = quat[2]
            trafo_optitrack2base_link_clone.transform.rotation.z = quat[2]

            self.tf_broadcast.sendTransform(trafo_optitrack2base_link_clone)

            # LAB to optitrack
            trafo_lab2optitrack = TransformStamped()
            trafo_lab2optitrack.header.stamp = time_stamp
            # trafo_lab2optitrack.header.stamp = rospy.Time.now()
            trafo_lab2optitrack.header.frame_id = "world_lab"
            trafo_lab2optitrack.child_frame_id = "world_optitrack"
            trafo_lab2optitrack.transform.translation.x = 0.0
            trafo_lab2optitrack.transform.translation.y = 0.0
            trafo_lab2optitrack.transform.translation.z = 0.0

            # q = tf.transformations.quaternion_from_euler(0, 0, -9./180*pi)
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            trafo_lab2optitrack.transform.rotation.x = q[0]
            trafo_lab2optitrack.transform.rotation.y = q[1]
            trafo_lab2optitrack.transform.rotation.z = q[2]
            trafo_lab2optitrack.transform.rotation.w = q[3]
            self.tf_broadcast.sendTransform(trafo_lab2optitrack)

            # Zero TF 
            # TODO: is this really needed?
            trafo_0 = TransformStamped()
            trafo_0.header.stamp = time_stamp
            trafo_0.header.frame_id = "world_lab"
            trafo_0.child_frame_id = "world"
            trafo_0.transform.rotation.w = 1
            self.tf_broadcast.sendTransform(trafo_0)

            # lock.release()
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


    def callback_agent_optitrack(self, data, ii):
        lock.acquire()

        self.data_agent[ii] = data
        if self.awaiting_msg_agent[ii]:
            self.awaiting_msg_agent[ii] = False
            print("Got 1st agent pose part#{}".format(ii))

        lock.release()        


if __name__==('__main__'):
    print("Starting node")
    try:
        TransformPublisher_odom()
    except rospy.ROSInterruptException:
        pass        
    # Instance_SensoryTreatment.listener()


print("")
print("")
print("Finished <<transoform_publisher.py>>  cleanly...")
