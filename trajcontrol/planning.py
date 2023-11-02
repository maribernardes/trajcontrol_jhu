import os
import rclpy
import numpy as np
import quaternion
import datetime


from rclpy.node import Node
from numpy import loadtxt
from std_msgs.msg import Int8, Int16
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Quaternion, Point
from ros2_igtl_bridge.msg import PointArray, String
from scipy.ndimage import median_filter

#########################################################################
#
# Planning Node
#
# Description:
# This node works in two different modes.
# If use_slicer is True, it receives target and skin entry points in 
# zFrame coordinates sent from 3DSlicer through OpenIGTLink bridge and 
# convertes them to robot coordinate frame
# If use_slicer is False, it selects the target to be a straight line 
# from the initial guide position with insertion length defined by
# insertion_length + air_gap
#
# Subscribes:   
# 'IGTL_POINT_IN'           (ros2_igtl_bridge.msg.PointArray) -  zFrame     name:TARGET 
# '/stage/initial_point'    (geometry_msgs.msg.PointStamped)- robot frame
#
# Publishes:    
# '/subject/state/target'     (geometry_msgs.msg.PointStamped) - robot frame
# '/subject/state/skin_entry' (geometry_msgs.msg.PointStamped) - robot frame
#
#########################################################################

class Planning(Node):

    def __init__(self):
        super().__init__('planning')

        #Declare node parameters
        self.declare_parameter('use_slicer', False)         # Get target and skin entry from 3DSlicer
        self.declare_parameter('air_gap', 10)               # Air gap between needle guide and tissue - Set target and skin entry from initial point
        self.declare_parameter('insertion_length', 100.0)   # Desired insertion length                - Set target and skin entry from initial point

#### Subscribed topics ###################################################

        #Topics from 3D Slicer interface (OpenIGTLink Bridge)
        self.subscription_bridge_point = self.create_subscription(PointArray, 'IGTL_POINT_IN', self.bridge_point_callback, 10)
        self.subscription_bridge_point # prevent unused variable warning

        # Topics from keyboard interface node (robot initialization)
        self.subscription_initial_point = self.create_subscription(PoseStamped, '/stage/initial_point', self.initial_point_callback, 10)
        self.subscription_initial_point # prevent unused variable warning

#### Published topics ###################################################

        # Skin entry and target (robot frame)
        timer_period_planning = 1.0  # seconds
        self.timer_planning = self.create_timer(timer_period_planning, self.timer_planning_callback)        
        self.publisher_skin_entry = self.create_publisher(PointStamped, '/subject/state/skin_entry', 10)
        self.publisher_target = self.create_publisher(PointStamped, '/subject/state/target', 10)
        
#### Stored variables ###################################################
        # Frame transformations
        self.zFrameToRobot = np.empty(shape=[0,7])  # ZFrame to robot frame transform

        self.initial_point = np.empty(shape=[0,3])  # Robot position at begining of experiment
        self.target = np.empty(shape=[0,3])         # User-defined tip position at desired target
        self.skin_entry = np.empty(shape=[0,3])     # User-defined tip position at skin entry
        
        self.use_slicer = self.get_parameter('use_slicer').get_parameter_value().bool_value
        self.air_gap = self.get_parameter('air_gap').get_parameter_value().double_value
        self.insertion_length = self.get_parameter('insertion_length').get_parameter_value().double_value
        
#### Interface initialization ###################################################

        # Initialize zFrameToRobot transform
        q_tf = np.quaternion(np.cos(np.deg2rad(45)), np.sin(np.deg2rad(45)), 0, 0)
        zFrameCenter = np.array([0,0,0])
        self.zFrameToRobot = np.concatenate((zFrameCenter, np.array([q_tf.w, q_tf.x, q_tf.y, q_tf.z])))

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

#### Listening callbacks ###################################################

    # Get current skin entry and target points 
    def bridge_point_callback(self, msg_point):
        name = msg_point.name      
        npoints = len(msg_point.pointdata)
        if (name == 'TARGET') and (npoints == 2): # Name is adjusted in 3DSlicer module
            skin_entry_zFrame = np.array([msg_point.pointdata[0].x, msg_point.pointdata[0].y, msg_point.pointdata[0].z, 1,0,0,0])
            target_zFrame = np.array([msg_point.pointdata[1].x, msg_point.pointdata[1].y, msg_point.pointdata[1].z, 1,0,0,0])
            skin_entry_robot = pose_transform(skin_entry_zFrame, self.zFrameToRobot)
            target_robot = pose_transform(target_zFrame, self.zFrameToRobot)
            self.skin_entry = skin_entry_robot[0:3]
            self.target = target_robot[0:3]

    # Get initial robot pose
    def initial_point_callback(self, msg_robot):
        if (self.initial_point.size == 0): # Do it only once
            robot = msg_robot.pose
            # Set initial point
            self.initial_point = np.array([robot.position.x, robot.position.y, robot.position.z])
            q_tf1= np.quaternion(np.cos(np.deg2rad(45)), np.sin(np.deg2rad(45)), 0, 0)
            q_tf2= np.quaternion(np.cos(np.deg2rad(45)), 0, 0, np.sin(np.deg2rad(45)))
            q_tf = q_tf1*q_tf2   
            # Set needleToRobot transform         
            self.needleToRobot = np.concatenate((self.initial_point[0:3], np.array([q_tf.w, q_tf.x, q_tf.y, q_tf.z]))) # Registration now comes from entry point

#### Publishing callbacks ###################################################

    # Publishes skin entry and target points
    def timer_planning_callback(self):
        if self.use_slicer is True:
            # Publishes only after 3DSlicer pushed values
            if (self.skin_entry.size != 0):
                msg = PointStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'stage'
                msg.point = Point(x=self.skin_entry[0], y=self.skin_entry[1], z=self.skin_entry[2])
                self.publisher_skin_entry.publish(msg)
                self.get_logger().debug('Skin entry (stage) = %s' %(self.skin_entry))
            if (self.target.size != 0):
                msg = PointStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'stage'
                msg.point = Point(x=self.target[0], y=self.target[1], z=self.target[2])
                self.publisher_target.publish(msg)
                self.get_logger().debug('Target (stage) = %s' %(self.target))
        else:
            # Publishes only after experiment started (stored inital point is available)
            if (self.initial_point.size != 0):
                msg = PointStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'stage'
                msg.point = Point(x=self.initial_point[0], y=self.initial_point[1]+self.air_gap, z=self.initial_point[2])
                self.publisher_skin_entry.publish(msg)
                self.get_logger().debug('Skin entry (stage) = %s' %(self.skin_entry))
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.point = Point(x=self.initial_point[0], y=self.initial_point[1]+self.air_gap+self.insertion_length, z=self.initial_point[2])
            
########################################################################

# Function: pose_transform
# DO: Transform pose to new reference frame
# Inputs: 
#   x_origin: pose in original reference frame (numpy array [x, y, z, qw, qx, qy, qz])
#   x_tf: transformation from original to new frame (numpy array [x, y, z, qw, qx, qy, qz])
# Output:
#   x_new: pose in new reference frame (numpy array [x, y, z, qw, qx, qy, qz])
def pose_transform(x_orig, x_tf):

    #Define frame transformation
    p_tf = np.quaternion(0, x_tf[0], x_tf[1], x_tf[2])
    q_tf= np.quaternion(x_tf[3], x_tf[4], x_tf[5], x_tf[6])

    #Define original position and orientation
    p_orig = np.quaternion(0, x_orig[0], x_orig[1], x_orig[2])
    q_orig = np.quaternion(x_orig[3], x_orig[4], x_orig[5], x_orig[6])

    #Transform to new frame
    q_new = q_tf*q_orig
    p_new = q_tf*p_orig*q_tf.conj() + p_tf

    x_new = np.array([p_new.x, p_new.y, p_new.z, q_new.w, q_new.x, q_new.y, q_new.z])
    return x_new

########################################################################

# Function: pose_inv_transform
# DO: Transform pose to new reference frame with inverse transform 
# Inputs: 
#   x_origin: pose in original reference frame (numpy array [x, y, z, qw, qx, qy, qz])
#   x_tf: transformation from original to new frame (numpy array [x, y, z, qw, qx, qy, qz])
# Output:
#   x_new: pose in new reference frame (numpy array [x, y, z, qw, qx, qy, qz])
def pose_inv_transform(x_orig, x_tf):

    #Define frame transformation
    p_tf = np.quaternion(0, x_tf[0], x_tf[1], x_tf[2])
    q_tf= np.quaternion(x_tf[3], x_tf[4], x_tf[5], x_tf[6])

    #Define original position and orientation
    p_orig = np.quaternion(0, x_orig[0], x_orig[1], x_orig[2])
    q_orig = np.quaternion(x_orig[3], x_orig[4], x_orig[5], x_orig[6])

    #Transform to new frame
    q_new = q_tf.conj()*q_orig
    p_new = q_tf.conj()*(p_orig-p_tf)*q_tf
    x_new = np.array([p_new.x, p_new.y, p_new.z, q_new.w, q_new.x, q_new.y, q_new.z])
    return x_new

########################################################################

def main(args=None):
    rclpy.init(args=args)

    planning = Planning()
    rclpy.spin(planning)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    planning.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()