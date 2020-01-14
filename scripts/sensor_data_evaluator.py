#!/usr/bin/env python
import rospy

# ROS libraries -- add to include file
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid

import sensor_msgs.point_cloud2 as pc2

import tf

import time

import numpy as np

import sys, warnings

from math import pi

from threading import Lock

# DEBUGGING
import matplotlib.pyplot as plt
plt.ion()

lock = Lock()

class SensoryTreatment(): 
    robot_radius = 0.3 # Simplify the robot as a circle
    room_dimensions = [20,10,3] # Robot laboratory dimensions [x, y, z]

    height_robot = 1.3
    robot_clearance_height = 1.5 # easily pass objects with a height inferor to this

    n_rows_lidar = 16 
    ang_resolution = 600
    
    def __init__(self):
        print("")
        print("Initializiation starting.\n")

        rospy.init_node('Sensory_Treatement', anonymous=True)

        self.awaiting_lidar = True
        self.awaiting_rearScan = True
        self.awaiting_frontScan = True
        
        rospy.Subscriber("velodyne_points", PointCloud2, self.callback_lidar)
        rospy.Subscriber("rear/scan", LaserScan, self.callback_laserScan, True)
        rospy.Subscriber("front/scan", LaserScan, self.callback_laserScan, False)

        pub_test = rospy.Publisher('test_point_cloud', PointCloud2 , queue_size=2)
        self.pub_occGrid = rospy.Publisher('occupancy_grid', OccupancyGrid , queue_size=2)

        while ((self.awaiting_lidar or self.awaiting_rearScan or self.awaiting_frontScan) 
               and not rospy.is_shutdown()):
            print("Waiting for sensors")
            rospy.sleep(0.2)
        
        tf_listener = tf.TransformListener()
        # self.subscribers()

        rate = rospy.Rate(10) # 10 Hz

        print("Entering main loop....")
        # Enter main loop
        while not rospy.is_shutdown():
            # print("Loop once again!")
            try:
                (self.trans_rearLaser, self.rot_rearLaser) = tf_listener.lookupTransform('/base_link', '/rear_laser', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            try:
                (self.trans_frontLaser, self.rot_frontLaser) = tf_listener.lookupTransform('/base_link', '/front_laser', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            try:
                (self.trans_lidar, self.rot_lidar) = tf_listener.lookupTransform('/base_link', '/velodyne_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
                
            lock.acquire() ##### LOCK ACQUIRE ######
            self.merge_sensors()

            self.occupancy_grid()

            lock.release() ##### LOCK RELEASE ######

            rate.sleep()

        print("Exiting main loop")


    def merge_sensors(self):
        # print('type', type(data))
        # print("\n\n")

        # tic = time.clock()

        n_points = self.data_lidar.height*self.data_lidar.width
        print("Number of data points {}".format(n_points))

        dim = 2
        self.points_lidar_2d = np.zeros((dim, n_points))
        self.points_lidar_z = np.zeros((n_points))

        it_count = 0
        for pp in pc2.read_points(self.data_lidar, skip_nans=True, field_names=("x", "y", "z")):
            # Cut off points which are to high
            self.points_lidar_2d[:, it_count] = [pp[0], pp[1]]
            self.points_lidar_z[it_count] = pp[2]

            # print("points are x={}, y={}, z={}".format(pp[0], pp[1], pp[2]))
            it_count += 1
            
        # print('it count', it_count)
        # print('n_points', n_points)

        self.points_lidar_2d += self.points_lidar_2d + np.tile(self.trans_lidar[:2], (n_points, 1)).T
        self.points_lidar_z += self.points_lidar_z + np.tile(self.trans_lidar[2], (n_points)).T
        
        # TODO - only evaluate if no collision
        ind_good = np.logical_and(self.points_lidar_z > 0, self.points_lidar_z < self.robot_clearance_height)
        
        # Describe points in robot center-frame
        radius_lidar = np.sqrt(self.points_lidar_2d[0, ind_good]*self.points_lidar_2d[0, ind_good] + 
                            self.points_lidar_2d[1, ind_good]*self.points_lidar_2d[1, ind_good])
        
        ind_good[ind_good] = (radius_lidar > self.robot_radius)

        self.points_lidar_2d = self.points_lidar_2d[:, ind_good]
        angle_lidar = np.arctan2(self.points_lidar_2d[1, :], self.points_lidar_2d[0, :])
v        
        # Front scan data
        n_points_frontScan = len(self.data_frontScan.ranges)
        front_scan_data = np.array(self.data_frontScan.ranges)
        ind_good = np.logical_and(front_scan_data<self.data_frontScan.range_max, 
                                  front_scan_data>self.data_frontScan.range_min)
        
        angle_front_scan = self.data_frontScan.angle_min + np.arange(n_points_frontScan)[ind_good]*self.data_frontScan.angle_increment

        
        self.points_frontScan = np.vstack((
            self.data_frontScan.ranges[ind_good]*np.cos(angle_front_scan),
            self.data_frontScan.ranges[ind_good]*np.sin(angle_front_scan) )) 

        # Include the transform difference (assumption of only linear transform)
        points_frontScan = self.points_frontScan + np.tile(self.trans_frontLaser[:2], (np.sum(ind_good), 1)).T
        angle_frontScan = np.arctan2(points_frontScan[1,:], points_frontScan[0,:]) 
        radius_frontScan = np.sqrt(points_frontScan[0,:]*points_frontScan[0,:] + points_frontScan[1,:]*points_frontScan[1,:])

        # Rear scan data
        n_points_rearScan = len(self.data_rearScan.ranges)
        rear_scan_data = np.asarray(self.data_rearScan.ranges)
        ind_good = np.logical_and(rear_scan_data<self.data_rearScan.range_max, 
                                  rear_scan_data>self.data_rearScan.range_min)

        angle_rearScan2base = pi
        angle_rearScan = self.data_rearScan.angle_min + np.arange(n_points_rearScan)[ind_good]*self.data_rearScan.angle_increment + angle_rearScan2base

        self.points_rearScan = np.vstack((
            self.data_rearScan.ranges[ind_good]*np.cos(angle_rearScan),
            self.data_rearScan.ranges[ind_good]*np.sin(angle_rearScan) )) 

        # Include the transform difference (assumption of only linear transform)
        points_rearScan = self.points_rearScan + np.tile(self.trans_rearLaser[:2], (np.sum(ind_good), 1)).T
        angle_rearScan = np.arctan2(points_rearScan[1,:], points_rearScan[0,:]) 
        radius_rearScan = np.sqrt(points_rearScan[0,:]*points_rearScan[0,:] + points_rearScan[1,:]*points_rearScan[1,:])


        # Merge sensors based on angle
        # TODO tweak an change this algorithm
        mergeAngle_start = -pi

        angle_merged = (np.arange(self.ang_resolution)+0.5)*self.ang_resolution - mergeAngle_start 
        radius_merged = np.zeros(self.ang_resolution)

        # ind_biggerThanLimitAngle_rearScan = np.ones(angle_rearScan.shape, dtype=bool)
        # ind_biggerThanLimitAngle_frontScan = np.ones(angle_frontScan.shape, dtype=bool)
        # ind_biggerThanLimitAngle_lidar = np.ones(angle_lidar.shape, dtype=bool)

        angle_rearScan_remaining = np.copy(angle_rearScan)
        angle_frontScan_remaining = np.copy(angle_frontScan)
        angle_lidar_remaining = np.copy(angle_lidar)

        radius_rearScan_remaining = np.copy(radius_rearScan)
        radius_frontScan_remaining = np.copy(radius_frontScan)
        radius_lidar_remaining = np.copy(radius_lidar)

        for aa in range(self.ang_resolution):
            limit_angle = mergeAngle_start + aa*(2*pi/self.ang_resolution)

            ind_criticalRange_rearScan = (angle_rearScan_remaining <= limit_angle)
            ind_criticalRange_frontScan = angle_frontScan_remaining <= limit_angle
            ind_criticalRange_lidar = angle_lidar_remaining <= limit_angle

            points_in_direction = np.hstack((radius_rearScan_remaining[ind_criticalRange_rearScan], radius_frontScan_remaining[ind_criticalRange_frontScan], radius_lidar_remaining[ind_criticalRange_lidar] ))


            np.delete(angle_rearScan_remaining, ind_criticalRange_rearScan)
            np.delete(radius_rearScan_remaining, ind_criticalRange_rearScan)

            np.delete(angle_frontScan_remaining, ind_criticalRange_frontScan)
            np.delete(radius_frontScan_remaining, ind_criticalRange_frontScan)

            np.delete(angle_lidar_remaining, ind_criticalRange_lidar)
            np.delete(radius_lidar_remaining,  ind_criticalRange_lidar)
            
            # ind_criticalRange_rearScan = (angle_rearScan[ind_biggerThanLimitAngle_rearScan] <= limit_angle)
            # ind_criticalRange_frontScan = angle_frontScan[ind_biggerThanLimitAngle_frontScan] <= limit_angle
            # ind_criticalRange_lidar = angle_lidar[ind_biggerThanLimitAngle_lidar] <= limit_angle

            # points_in_direction = np.hstack((radius_rearScan[ind_biggerThanLimitAngle_rearScan][ind_criticalRange_rearScan], radius_frontScan[ind_biggerThanLimitAngle_frontScan][ind_criticalRange_frontScan], radius_lidar[ind_biggerThanLimitAngle_lidar][ind_criticalRange_lidar] ))

            # ind_biggerThanLimitAngle_rearScan[ind_biggerThanLimitAngle_rearScan][ind_criticalRange_rearScan] = 0
            # ind_biggerThanLimitAngle_frontScan[ind_biggerThanLimitAngle_frontScan][ind_criticalRange_frontScan] = 0
            # ind_biggerThanLimitAngle_lidar[ind_biggerThanLimitAngle_lidar][ind_criticalRange_lidar] = 0

            print("total left {}".format(angle_rearScan_remaining.shape))
            print("inital toal {}".format(angle_rearScan.shape))
            import pdb; pdb.set_trace() # Breakpoint            

            if points_in_direction.shape[0] > 0:
                radius_merged[aa] = np.min(points_in_direction)
            # else:
                # import pdb; pdb.set_trace() # Breakpoint

        import pdb; pdb.set_trace() # Breakpoint
        nonzero_radius = (radius_merged>0)
        self.points_merged_cartesian = np.vstack((radius_merged[nonzero_radius]*np.cos(angle_merged[nonzero_radius]),
                                                  radius_merged[nonzero_radius]*np.sin(angle_merged[nonzero_radius]) ))

    def occupancy_grid(self, resolution=0.1):
        ind_occupancy_grid_2d = np.round(self.points_merged_cartesian/resolution).astype(int)

        import pdb; pdb.set_trace() # Breakpoint
        min_vals_cartesian = np.min(self.points_merged_cartesian, axis=1)

        min_ind_occGrid = np.min(ind_occupancy_grid_2d, axis=1)
        max_ind_occGrid = np.max(ind_occupancy_grid_2d, axis=1)

        # print(min_ind_occGrid)
        ind_occupancy_grid_2d = (ind_occupancy_grid_2d - np.tile(min_ind_occGrid, (ind_occupancy_grid_2d.shape[1],1)).T)

        msg_occupancy_grid = OccupancyGrid()
        msg_occupancy_grid.header.stamp = rospy.Time.now()
        msg_occupancy_grid.header.frame_id = "base_link"
        
        msg_occupancy_grid.info.resolution = resolution
        msg_occupancy_grid.info.width = (max_ind_occGrid[0]-min_ind_occGrid[0])+1
        msg_occupancy_grid.info.height = (max_ind_occGrid[1]-min_ind_occGrid[1])+1

        # Position occupancy grid
        msg_occupancy_grid.info.origin.position.x = min_vals_cartesian[0]
        msg_occupancy_grid.info.origin.position.y = min_vals_cartesian[1]

        # msg_occupancy_grid.data = 
        ind_occupancy_grid_1d = (ind_occupancy_grid_2d[0] + ind_occupancy_grid_2d[1]*(msg_occupancy_grid.info.width)).astype(int)
        # import pdb; pdb.set_trace()

        # TODO change str to bytes(2) in PYTHON 3
        if sys.version_info[0] > 2:
            warnings.warn("Change implementation for python 3")

        data_grid = np.zeros(msg_occupancy_grid.info.width*msg_occupancy_grid.info.height, dtype=np.int8)

        data_grid[ind_occupancy_grid_1d] = 100

        for ii in range(10):
            data_grid[ii] = 100
        
        # msg_occupancy_grid.data  = str(0)*ind_occupancy_grid_1d.shape[0]
        msg_occupancy_grid.data = data_grid.tolist()

        # import pdb; pdb.set_trace()

        self.pub_occGrid.publish(msg_occupancy_grid)


    def point_reduction(self, t, rad_cluster=0.01, 
                        redundancy_distance=0.2):
        # remove points which are percieved at non-old points
        # 

        # Remove close points // occupancy grid

        pass
        # Choose conservative approach
        # Extract x & y data based on estimated position
        # TODO
        
        # toc = time.clock()
        # print("unpacking took {} s".format(toc-tic))
        
        
    def callback_laserScan(self, data, is_front):
        lock.acquire()
        if is_front: # Frontlaser
            self.data_frontScan = data
            self.awaiting_frontScan = False
        else: # Backlaser
            self.data_rearScan = data
            self.awaiting_rearScan = False
        lock.release()



    def callback_lidar(self, data):
        lock.acquire()

        self.data_lidar = data

        lock.release()

        self.awaiting_lidar = False

        



if __name__==('__main__'):
    print("Starting node")
    Instance_SensoryTreatment = SensoryTreatment()
    # Instance_SensoryTreatment.listener()

print("")
print("")
print("Finished cleanly...")
