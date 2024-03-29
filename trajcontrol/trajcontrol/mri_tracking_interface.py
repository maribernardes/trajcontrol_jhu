import os
import rclpy
import numpy as np
import quaternion
import datetime

from rclpy.node import Node
from numpy import loadtxt
from std_msgs.msg import Int8, Int16
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion, Point, PoseArray
from ros2_igtl_bridge.msg import Transform, PointArray

from numpy import loadtxt
from ament_index_python.packages import get_package_share_directory
from .utils import *

#########################################################################
#
# MRI Tracking Interface Node
#
# This node gets current needle tip pose from 3DSlicer and converts
# it to robot frame. It also gets robot current position and extracts the
# needle base pose in needle frame.
#
# Subscribes:   
# 'IGTL_TRANSFORM_IN'           (ros2_igtl_bridge.msg.Transform) - zFrame           name: CurrentTrackedTipZ
# 'IGTL_TRANSFORM_IN'           (ros2_igtl_bridge.msg.Transform) - scanner frame    name: CurrentTrackedTipTransform
# '/stage/state/guide_pose'     (geometry_msgs.msg.PointStamped)  - robot frame
# '/stage/initial_point'        (geometry_msgs.msg.PointStamped) - robot frame
#
# Publishes:    
# '/needle/state/tip_mri_pose'  (geometry_msgs.msg.PoseStamped)   - scanner frame
# '/sensor/tip'                 (geometry_msgs.msg.PoseStamped)   - robot frame
# '/sensor/base'                (geometry_msgs.msg.PoseStamped)   - robot frame
# 
#########################################################################

class MRITrackingInterface(Node):

    def __init__(self):
        super().__init__('mri_tracking_interface')

        self.declare_parameter('needle_length', 20.0) # Needle total length parameter

#### Subscribed topics ###################################################

        #Topics from 3D Slicer interface (OpenIGTLink Bridge)
        self.subscription_bridge_transform = self.create_subscription(Transform, 'IGTL_TRANSFORM_IN', self.bridge_transform_callback, 10)
        self.subscription_bridge_transform # prevent unused variable warning

        #Topics from robot node (template)
        self.subscription_robot = self.create_subscription(PointStamped, '/stage/state/guide_pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Topic from keypress node (keyboard)
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning
        self.listen_keyboard = False

#### Published topics ###################################################

        # TODO: Adjust names '/sensor/tip' and '/sensor/base'

        # Tip (scanner frame)
        self.publisher_tip_scanner = self.create_publisher(PoseStamped, '/needle/state/tip_mri_pose', 10)  #(scanner frame)

        # Tip (robot frame)
        timer_period_tip = 0.3 # seconds
        self.timer_tip = self.create_timer(timer_period_tip, self.timer_tip_callback)
        self.publisher_tip = self.create_publisher(PoseStamped, '/sensor/tip', 10)  #(stage frame)

        # Base (robot frame)
        timer_period_base = 0.3 # seconds
        self.timer_base = self.create_timer(timer_period_base, self.timer_base_callback)
        self.publisher_base = self.create_publisher(PoseStamped,'/sensor/base', 10) #(stage frame)      

#### Stored variables ###################################################

        self.zFrameToRobot = np.empty(shape=[0,7])  # ZFrame to robot frame transform

        self.tip_scanner = np.empty(shape=[0,7])    # Tracked tip position (scanner frame)
        self.Z = np.empty(shape=[0,7])            # Tracked tip position (robot frame)
        self.initial_point = np.empty(shape=[0,3])  # Needle guide position at begining of experiment (robot frame)
        self.stage = np.empty(shape=[0,3])          # Robot current position (robot frame)
        self.X = np.empty(shape=[0,7])              # Base pose (robot frame)

        # Node parameters
        self.needle_length = self.get_parameter('needle_lenght').get_parameter_value().double_value

#### Interface initialization ###################################################

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        # Load zFrameToRobot transform
        try:
            smart_template_share_directory = get_package_share_directory('smart_template')
            self.zFrameToRobot  = np.array(loadtxt(os.path.join(smart_template_share_directory,'files','zframe.csv'), delimiter=','))
        except IOError:
            self.get_logger().info('Could not find zframe.csv file')

#### Listening callbacks ###################################################

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
            needle_base = np.array([self.initial_point[0], self.initial_point[1]-self.needle_length, self.initial_point[2]])
            self.needleToRobot = np.concatenate((needle_base, np.array([q_tf.w, q_tf.x, q_tf.y, q_tf.z]))) # Registration now comes from entry point

    # Get tracked tip pose and convert to robot frame
    def bridge_transform_callback(self, msg):
        name = msg.name      
        self.get_logger().info('Name = %s' %name)
        if (name == 'CurrentTrackedTipZ'): # Name is adjusted in 3DSlicer module
            tip_zFrame = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z, \
                msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z])
            self.tip = pose_transform(tip_zFrame, self.zFrameToRobot)
            self.get_logger().info('Tip = %s' %(self.tip)) 
        if (name == 'CurrentTrackedTipTransform'): # Name is adjusted in 3DSlicer module
            self.tip_scanner = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z, \
                msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z])
            self.get_logger().info('Tip Scanner = %s' %(self.tip_scanner)) 

    # Get current robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.point
        self.stage = np.array([robot.x, robot.y, robot.z])
        # Store current needle base pose (in robot and needle frames)
        if (self.needleToRobot.size != 0):
            needle_q = self.needleToRobot[3:7]
            self.X = np.array([self.stage[0], self.stage[1], self.stage[2], needle_q[0], needle_q[1], needle_q[2], needle_q[3]]) #base in robot frame       
            self.needle_pose = pose_inv_transform(self.X, self.needleToRobot)   # base in needle frame

#### Publishing callbacks ###################################################

    # Publishes needle tip in scanner frame and robot frame
    def timer_tip_callback (self):
        if (self.tip_scanner.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'scanner'
            msg.pose.position = Point(x=self.tip_scanner[0], y=self.tip_scanner[1], z=self.tip_scanner[2])
            msg.pose.orientation = Quaternion(w=self.tip_scanner[3], x=self.tip_scanner[4], y=self.tip_scanner[5], z=self.tip_scanner[6])
            self.publisher_tip_scanner.publish(msg)
        if (self.Z.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            msg.pose.position = Point(x=self.Z[0], y=self.Z[1], z=self.Z[2])
            msg.pose.orientation = Quaternion(w=self.Z[3], x=self.Z[4], y=self.Z[5], z=self.Z[6])
            self.publisher_tip.publish(msg)

    # Publishes needle base transformed to robot frame
    def timer_base_callback (self):
        # Publish last needle pose in robot frame
        if (self.X.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            msg.pose.position = Point(x=self.X[0], y=self.X[1], z=self.X[2])
            msg.pose.orientation = Quaternion(w=self.X[3], x=self.X[4], y=self.X[5], z=self.X[6])
            self.publisher_base.publish(msg)

########################################################################

def main(args=None):
    rclpy.init(args=args)

    mritracking_interface = MRITrackingInterface()
    rclpy.spin(mritracking_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mritracking_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()