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
# path_obstacle_avoidance = "/home/lukas/catkin_ws/src/ridgeback_movement/scripts/dynamic_obstacle_avoidance/src"
path_obstacle_avoidance = "/home/lukas/ridgeback_ws/src/ridgeback_movement/scripts/dynamic_obstacle_avoidance/src"
if not path_obstacle_avoidance in sys.path:
    sys.path.append(path_obstacle_avoidance)

from dynamic_obstacle_avoidance.obstacle_avoidance.obstacle_container import *

from dynamic_obstacle_avoidance.obstacle_avoidance.obstacle import *
from dynamic_obstacle_avoidance.obstacle_avoidance.ellipse_obstacles import Ellipse
from dynamic_obstacle_avoidance.obstacle_avoidance.obstacle_polygon import Cuboid, Polygon
from dynamic_obstacle_avoidance.obstacle_avoidance.linear_modulations import * 

from dynamic_obstacle_avoidance.obstacle_avoidance.obs_common_section import *
from dynamic_obstacle_avoidance.obstacle_avoidance.obs_dynamic_center_3d import get_dynamic_center_obstacles


print("... finished importing libraries")


# @staticmethod
def quaternion_from_2vectors(vec1, vec2=None):
    if vec2==None:
        vec2 = vec1
        vec1 = np.zeros(vec2.shape)
        vec1[0] = 1

    # Normalize?
    quat = np.zeros(4)
    quat[0] = np.linalg.norm(vec1)*np.linalg.norm(vec2) + np.dot(vec1, vec2) # w

    if vec1.shape[0]==2: # 2D projection
        quat[3] = np.cross(vec1, vec2) # vec
    else:
        quat[1:] = np.cross(vec1, vec2) # vec
    return quat/np.linalg.norm(quat)


def define_obstacles_lab_south(obstacle_number, robot_radius=0.53, exponential_weight=0.1):
    # TODO create json file for obstacle description.
    if obstacle_number ==-2:
        edge_points = np.array((
            [[-1.4,-2.2,-2.2, -1.4],
             [ 0.2, 0.2,-1.0,-1.0]]))

        frame_id = "world_lab"
        obs = Polygon(edge_points=edge_points, is_boundary=True, margin_absolut=robot_radius, sigma=exponential_weight)
        
        # Displacement
        obs.center_position += np.array([0.0, 0.0])
        obs.orientation += 0./180*pi
        return obs

    if obstacle_number ==-1:
        edge_points = np.array((
            # [4.0, 1.0, 1.0, 0.0, 0.0,-4.0,-4.0,-2.5,-2.5, 0.0, 0.0, 4.0],
            # [0.0, 0.0, 1.0, 1.0, 1.6, 1.6,-2.0,-2.0,-3.6,-3.6,-3.0,-3.0])
            [3.8, 3.8,-0.5,-0.5, 0.2, 0.2, 3.8],
            [0.0, 2.0, 2.0,-0.8,-0.8,-2.0,-2.0]
        ))

        # edge_points = np.array((
            # [100.0,-100.0,-100.0, 100.0],
            # [100.0, 100.0, -100.0,-100.0]))

        frame_id = "world_lab"
        obs = Polygon(edge_points=edge_points, is_boundary=True, margin_absolut=robot_radius, sigma=exponential_weight)
        
        # Displacement
        obs.center_position += np.array([-2.4, 0])
        obs.orientation += 0.5/180*pi
        return obs

    if obstacle_number==0:
        # Human
        obs = Ellipse(axes_length=[0.35, 0.15], margin_absolut=robot_radius, sigma=exponential_weight, name='coworker')
        obs.is_static = False
        return obs

    if obstacle_number==1:
        # Table
        obs = Cuboid(axes_length=[1.0, 1.0], center_position=[0.3, -1.70], margin_absolut=robot_radius, sigma=exponential_weight, name='kuka')
        obs.is_static = True
        return obs

    if obstacle_number==2:
        # Table
        obs = Cuboid(axes_length=[0.8, 1.8], center_position=[-0.65, -1.35], margin_absolut=robot_radius, sigma=exponential_weight, name='table')
        obs.is_static = True
        return obs

    if obstacle_number==3:
        # Table
        obs = Cuboid(axes_length=[0.8, 1.8], center_position=[1.18, 0.86], margin_absolut=robot_radius, sigma=exponential_weight, name='table_computer')
        obs.is_static = True
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

    def __init__(self, n_dynamic_obstacles=None, n_static_obstacles=3):
        print("")
        print("Initializiation starting.\n")

        rospy.init_node('Sensory_Treatement', anonymous=True)
        self.node_is_active = True

        # First obstacles are dynamic
        if n_dynamic_obstacles is None:
            # self.obstacle_topic_names = ['obstacle', 'hat']
            # self.obstacle_topic_names = ['obstacle0', 'obstacle1']
            self.obstacle_topic_names = ['obstacle0']
            self.n_dynamic_obstacles = len(self.obstacle_topic_names)
        else:
            self.n_dynamic_obstacles = n_dynamic_obstacles
            self.obstacle_topic_names = ['obstacle'+str(ii) for ii in range(self.n_obstacles)] # 

        self.awaiting_msg_obstacles = [True]*self.n_dynamic_obstacles

        # Static obstacles
        self.n_obstacles = self.n_dynamic_obstacles + n_static_obstacles
        self.obstacles = ObstacleContainer()
        for oo in range(self.n_obstacles):
            self.obstacles.append(define_obstacles_lab_south(oo))
            
        # Callbacks and Subscribers

        # self.data_agent = None        
        self.data_obstacle = [None]*(self.n_obstacles+1) # + boundary

        # Use ridgeback-time stamp for messages
        self.ridgeback_stamp = None
        self.awaiting_msg_imu = True
        rospy.Subscriber("/imu/data", Imu, self.callback_imu) # get timestamp

        # self.topic_agent = "/vrpn_client_node/ridgeback0/pose"
        # rospy.Subscriber(self.topic_agent, PoseStamped, self.callback_agent_optitrack)

        self.topic_obstacles = [0]*self.n_obstacles
        for oo, obs_str in izip(range(self.n_dynamic_obstacles), self.obstacle_topic_names):
            self.topic_obstacles[oo] = "vrpn_client_node/"+ obs_str +"/pose"
            print("Topic " + str(oo) + " :" + self.topic_obstacles[oo])
            rospy.Subscriber(self.topic_obstacles[oo], PoseStamped, self.callback_obstacle_optitrack, oo)

        # Publisher
        # pub_test = rospy.Publisher('test_point_cloud', PointCloud2 , queue_size=5)
        # self.pub_occGrid = rospy.Publisher('occupancy_grid', OccupancyGrid , queue_size=5)
        self.pub_eeVel_modulated = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Visualization
        self.pub_eeVel_initial_pose = rospy.Publisher("/ridgeback/inital_ds", PoseStamped, queue_size=5)
        self.pub_eeVel_modulated_pose = rospy.Publisher("/ridgeback/modulated_ds", PoseStamped, queue_size=5)
        
        # Create obstacles and publisher
        self.pub_obstacles = []
        self.pub_obstacles_hull = []
        self.pub_obstacles_axis = []

        for oo in range(self.n_obstacles):
            self.pub_obstacles.append(rospy.Publisher("polygon_obstacle"+str(oo), PolygonStamped, queue_size=5))
            self.pub_obstacles_hull.append(rospy.Publisher("polygon_hull_obstacle"+str(oo), PolygonStamped, queue_size=5))
            self.pub_obstacles_axis.append(rospy.Publisher("axis_obstacle"+str(oo), PoseStamped, queue_size=5))

        # self.pub_obstacles.append(rospy.Publisher("polygon_wall", PolygonStamped, queue_size=5))
        self.pub_obstacles.append(rospy.Publisher("polygon_wall", Path, queue_size=10))
        self.pub_obstacles_hull.append(rospy.Publisher("polygon_hull_wall", Path, queue_size=10))

        self.obstacles.index_wall = len(self.obstacles)
        self.obstacles.append(define_obstacles_lab_south(-1))

        self.tf_listener = tf.TransformListener()
        self.tf_broadcast = tf2.TransformBroadcaster()

        # self.rate = rospy.Rate(50) # Hz
        self.rate = rospy.Rate(5) # Hz

        self.pos_obstacles = [0]*self.n_obstacles        
        self.orient_obstacles = [0]*self.n_obstacles

        for oo in range(len(self.obstacles)):
            self.obstacles[oo].draw_obstacle() # Draw boundary
        
        self.it_attractor = 0
        self.attractor_list = np.array([[0, 1, 0],
                                        [0, 0, 1]]) # Define default attractor

        while (any(self.awaiting_msg_obstacles) 
               and not rospy.is_shutdown()):
            print("Waiting for messages ...")
            for ii in range(self.n_dynamic_obstacles):
                if self.awaiting_msg_obstacles[ii]:
                    print('.. awaiting obstacle', self.obstacle_topic_names[ii])
            # if self.awaiting_msg_agent:
                # print('.. awaiting agent')
            rospy.sleep(0.25)
            
    def run(self):
        print("Starting main loop....")
        # START MAIN LOOP
        while not rospy.is_shutdown():
            lock.acquire() ##### LOCK ACQUIRE ######
            try:
                (self.pos_agent, self.orient_agent) = self.tf_listener.lookupTransform('/world_optitrack', '/base_link', rospy.Time(0))
                
                for oo in range(self.n_dynamic_obstacles):
                    (self.pos_obstacles[oo], self.orient_obstacles[oo]) = self.tf_listener.lookupTransform('/world_optitrack', '/'+ self.obstacle_topic_names[oo], rospy.Time(0))
                    
            except:
                print('No luck today for the mighty transform scout...')
                lock.release(); self.rate.sleep(); continue
                import pdb; pdb.set_trace()

            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # print('No luck today for the mighty transform scout...')
                # import pdb; pdb.set_trace()
                
            # Update obstacle position
            for oo in range(self.n_dynamic_obstacles):
                try:
                    euler = tf.transformations.euler_from_quaternion(self.orient_obstacles[oo])
                except:
                    import pdb; pdb.set_trace()
                # print('obs #{} - orienation={}deg -- pos=[{}, {}]'.format(oo, np.round(euler[0]*180/pi,2), np.round(self.pos_obstacles[oo][0],2), np.round(self.pos_obstacles[oo][1],2)))
                self.obstacles[oo].update_position_and_orientation(
                    position=[self.pos_obstacles[oo][0],self.pos_obstacles[oo][1]],
                    orientation=euler[0])

            # Publish obstacle visualization
            for oo in range(len(self.obstacles)-1):
                obstacle = PoseStamped()
                obstacle.header.frame_id = "world_lab"
                obstacle.header.stamp = self.ridgeback_stamp # Time issues -> take ridgeback_stamp
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
                obstacle_polygon.header.stamp = self.ridgeback_stamp # Time issues -> take ridgeback_stamp

                surface_points = self.obstacles[oo].x_obs
                for pp in range(surface_points.shape[1]):
                    point = Point32(surface_points[0, pp], surface_points[1, pp], 0)
                    obstacle_polygon.polygon.points.append(point)
                self.pub_obstacles[oo].publish(obstacle_polygon)

                # Surface polygon with  with margin
                obstacle_polygon = PolygonStamped()
                obstacle_polygon.header.frame_id = "world_lab"
                obstacle_polygon.header.stamp = self.ridgeback_stamp # Time issues -> take ridgeback_stamp

                surface_points = self.obstacles[oo].x_obs_sf
                for pp in range(surface_points.shape[1]): # 
                    point = Point32(surface_points[0, pp], surface_points[1, pp], 0)
                    obstacle_polygon.polygon.points.append(point)
                self.pub_obstacles_hull[oo].publish(obstacle_polygon)

            
            # Only wall visualization (last obstacle of the list) is a <<path>> message
            for oo in [len(self.obstacles)-1]:
                obstacle_path = Path()
                obstacle_path.header.frame_id = "world_lab"
                # obstacle_path.header.stamp = rospy.Time.now() 
                obstacle_path.header.stamp = self.ridgeback_stamp # Time issues -> take ridgeback_stamp

                surface_points = self.obstacles[oo].x_obs
                for pp in range(surface_points.shape[1]): # 
                    point_pose = PoseStamped()
                    point_pose.header.frame_id = obstacle_path.header.frame_id
                    point_pose.header.stamp = obstacle_path.header.stamp
                    point_pose.pose.position.x, point_pose.pose.position.y = float(surface_points[0, pp]), float(surface_points[1, pp])
                    obstacle_path.poses.append(point_pose)

                self.pub_obstacles[oo].publish(obstacle_path)
                
                # Points With margin
                obstacle_path = Path()
                obstacle_path.header.frame_id = "world_lab"
                # obstacle_path.header.stamp = rospy.Time.now()
                obstacle_path.header.stamp = self.ridgeback_stamp # Time issues -> take ridgeback_stamp

                surface_points = self.obstacles[oo].x_obs_sf
                for pp in range(surface_points.shape[1]): # 
                    point_pose = PoseStamped()
                    point_pose.header.frame_id = obstacle_path.header.frame_id # Needed?
                    point_pose.header.stamp = obstacle_path.header.stamp
                    point_pose.pose.position.x, point_pose.pose.position.y = float(surface_points[0, pp]), float(surface_points[1, pp])
                    obstacle_path.poses.append(point_pose)

                self.pub_obstacles_hull[oo].publish(obstacle_path)

            # Modulation
            # publish_linear_ds(self, position, attractor, max_vel=0.07, slow_down_dist=0.1, publish_ros_message=False)
            # publish_modulated_ds(self, position, velocity, obstacles, max_vel=0.07, slow_down_dist=0.1, publish_ros_message=True)
            # import pdb; pdb.set_trace()

            ##### Obstacle Avoidance Algorithm #####
            # Worksapce udpate
                # Adjust dynamic center
            automatic_reference_point = True
            if automatic_reference_point:
                intersection_obs = get_intersections_obstacles(self.obstacles)
                get_dynamic_center_obstacles(self.obstacles, intersection_obs)
            
            attractor_is_reached = self.toggle_attractor(position=self.pos_agent)

            if attractor_is_reached:
                print("Switch attractor to {}".format(self.attractor))
                
            linear_ds = self.publish_linear_ds(position=self.pos_agent, attractor=self.attractor)
            modulated_ds = self.publish_modulated_ds(self.pos_agent, linear_ds, self.obstacles)
            ### Visualize
            self.visualize_ds(position=self.pos_agent, initial_ds=linear_ds, modulated_ds=modulated_ds, time_stamp=self.ridgeback_stamp)

            ##### Finihsed #####

            lock.release() ##### LOCK RELEASE ######

            print("Loop finihsed")
            self.rate.sleep()

        print("Exiting main loop")
    
    @property
    def attractor(self):
        return self.attractor_list[:, self.it_attractor]
        # if True:
            # attr = self.obstacles[-1].transform_relative2global(attr)

    def toggle_attractor(self, position, attraction_dist=0.1, random_attr=True):
        dist = np.linalg.norm(self.attractor-position[:2])
        
        if (dist<attraction_dist):
            if random_attr:
                min_vals = np.min(self.obstacles[-1].edge_points, axis=0)
                max_vals = np.min(self.obstacles[-1].edge_points, axis=0)

                in_free_space =  False
                while not in_free_space:
                    new_point = np.random.rand(2)*(max_vals-min_vals)+min_vals

                    in_free_space = obs_check_collision(new_point)
                    in_free_space = in_free_space[0] # Reshape
                self.attractor_list = new_point.reshape(self.dim, 1)
                self.it_attractor = 0 # Keep 1

            else:
                self.it_attractor += 1
            return True
        else:
            return False


    def visualize_ds(self, position, initial_ds, modulated_ds, time_stamp=None):
        msg = PoseStamped()
        if time_stamp is None:
            msg.header.stamp = rospy.Time.now()
        else:
            msg.header.stamp = time_stamp
        msg.header.frame_id = '/world_lab'
        msg.pose.position.x, msg.pose.position.y = position[0], position[1]
        
        # Initial DS
        orientation = quaternion_from_2vectors(np.array([1, 0]), initial_ds)
        msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = tuple(orientation)
        self.pub_eeVel_initial_pose.publish(msg)
        
        # Modulated DS
        orientation = quaternion_from_2vectors(np.array([1, 0]),  modulated_ds)
        msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = tuple(orientation)
        self.pub_eeVel_modulated_pose.publish(msg)

        
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


    def callback_obstacle_optitrack(self, data, obstacle_number):
        lock.acquire()

        self.data_obstacle[obstacle_number] = data

        if self.awaiting_msg_obstacles[obstacle_number]:
            self.awaiting_msg_obstacles[obstacle_number] = False
            print("Got first obstacle #{}".format(obstacle_number))

        # if True and obstacle_number==0:
            # quat = [data.pose.orientation.w,
                    # data.pose.orientation.x,
                    # data.pose.orientation.y,
                    # data.pose.orientation.z]
            # print('orienation of obs#{} = {}'.format(obstacle_number, np.round(quat, 2)))
        lock.release()

    # def callback_agent_optitrack(self, data):
        # lock.acquire()
        # self.data_agent = data
        # if self.awaiting_msg_agent:
            # self.awaiting_msg_agent = False
            # print("Got 1st agent pose")
        # lock.release()

    # def callback_agent_pose(self, data):
        # Agent pose given by the 'transform publisher'
        # lock.acquire()
        # self.data_agent = data
        # if self.awaiting_msg_agent:
            # self.awaiting_msg_agent = False
            # print("Got 1st agent pose")
        # lock.release()        
        
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
