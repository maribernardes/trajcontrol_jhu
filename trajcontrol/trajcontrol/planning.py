import os
import rclpy
import numpy as np
import quaternion
import datetime

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from std_msgs.msg import Int8
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Quaternion, Point
from ros2_igtl_bridge.msg import PointArray
from smart_control_interfaces.action import MoveStage
from smart_control_interfaces.srv import GetPoint

from numpy import loadtxt
from ament_index_python.packages import get_package_share_directory

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
# '/keyboard/key'           (std_msgs.msg.Int8)
# '/stage/state/guide_pose' (geometry_msgs.msg.PointStamped)  - robot frame
#
# Service server:    
# '/planning/get_skin_entry'         (trajcontrol_interfaces.srv.GetPoint) - robot frame
# '/planning/get_target'             (trajcontrol_interfaces.srv.GetPoint) - robot frame
# '/planning/get_initial_point'      (trajcontrol_interfaces.srv.GetPoint) - robot frame
#
# Action client:
# 'stage/move'              (smart_control_interfaces.action.MoveStage) - robot frame
#
#########################################################################

class Planning(Node):

    def __init__(self):
        super().__init__('planning')

        #Declare node parameters
        self.declare_parameter('use_slicer', True)          # Get target and skin entry from 3DSlicer
        self.declare_parameter('air_gap', 10.0)             # Air gap between needle guide and tissue - use_slicer is False (set target and skin entry from homing point)
        self.declare_parameter('insertion_length', 100.0)   # Desired insertion length                - use_slicer is False (set target and skin entry from homing point)

#### Stored variables ###################################################

        self.use_slicer = self.get_parameter('use_slicer').get_parameter_value().bool_value
        self.air_gap = self.get_parameter('air_gap').get_parameter_value().double_value
        self.insertion_length = self.get_parameter('insertion_length').get_parameter_value().double_value

        self.zFrameToRobot = np.empty(shape=[0,7])  # ZFrame to robot frame transform

        self.target = np.empty(shape=[0,3])         # User-defined tip position at desired target
        self.skin_entry = np.empty(shape=[0,3])     # User-defined tip position at skin entry
        
        self.initial_point = np.empty(shape=[0,3])  # Stage position at begining of experiment
        self.stage = np.empty(shape=[0,3])          # Stage positions: horizontal / depth / vertical 

        self.listen_keyboard = False                # Flag for waiting keyboard input (set robot initial position)

#### Subscribed topics ###################################################

        #Topics from 3D Slicer interface (OpenIGTLink Bridge)
        if self.use_slicer is True:
            self.subscription_bridge_point = self.create_subscription(PointArray, 'IGTL_POINT_IN', self.bridge_point_callback, 10)
            self.subscription_bridge_point # prevent unused variable warning

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PointStamped, '/stage/state/guide_pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

#### Action client ###################################################

        # Action client to move_stage
        self.action_client = ActionClient(self, MoveStage, '/stage/move')

#### Service server ##############################################

        # Service servers to return planning points
        self.skin_entry_server = None  # Activate only after robot is initialized (see function self.get_result_callback)
        self.target_server = None  # Activate only after robot is initialized (see function self.get_result_callback)
        self.initial_point_server = None  # Activate only after robot is initialized (see function self.get_result_callback)

#### Interface initialization ###################################################
        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        # Load SmartTemplate Transform
        self.get_logger().info('Loading SmartTemplate zFrame...')
        try:
            smart_template_share_directory = get_package_share_directory('smart_template')
            self.zFrameToRobot  = np.array(loadtxt(os.path.join(smart_template_share_directory,'files','zframe.csv'), delimiter=','))
            self.get_logger().info('Loaded zFrame: %s' %self.zFrameToRobot)
        except IOError:
            self.get_logger().info('Could not find zframe.csv file')

#### Listening callbacks ###################################################

    # Get current robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.point
        self.stage = np.array([robot.x, robot.y, robot.z])
        
    # Get current skin entry and target points 
    def bridge_point_callback(self, msg_point):
        if (self.use_slicer is True): # Only for Slicer
            name = msg_point.name      
            npoints = len(msg_point.pointdata)
            if (name == 'TARGET') and (npoints == 2): # Name is adjusted in 3DSlicer module
                q = np.quaternion(self.zFrameToRobot[3], self.zFrameToRobot[4], self.zFrameToRobot[5], self.zFrameToRobot[6]).conj() # Get identity orientation in zFrame coordinates
                if (self.initial_point.size == 0): # Experiment not initialized
                    self.listen_keyboard == True 
                    # Change skin_entry only if experiment not initialized yet
                    skin_entry_zFrame = np.array([msg_point.pointdata[0].x, msg_point.pointdata[0].y, msg_point.pointdata[0].z, q.w,q.x,q.y,q.z])
                    skin_entry_robot = pose_transform(skin_entry_zFrame, self.zFrameToRobot)
                    self.skin_entry = skin_entry_robot[0:3]
                    self.get_logger().info('Skin entry (stage) = %f, %f, %f' %(skin_entry_robot[0], skin_entry_robot[1], skin_entry_robot[2]))
                # Target can be changed in 3DSlicer after initialization
                target_zFrame = np.array([msg_point.pointdata[1].x, msg_point.pointdata[1].y, msg_point.pointdata[1].z, q.w,q.x,q.y,q.z])
                target_robot = pose_transform(target_zFrame, self.zFrameToRobot)
                self.target = target_robot[0:3]
                self.get_logger().info('Target (stage) = = %f, %f, %f' %(target_robot[0], target_robot[1], target_robot[2]))

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        if (self.listen_keyboard is True) and (self.initial_point.size == 0) and (msg.data == 32):  # SPACE: initialize stage initial point only ONCE
            self.get_logger().info('Initializing experiment:')
            if (self.use_slicer is False): # Define entry point with current position
                self.skin_entry = np.array([self.stage[0], self.stage[1]+self.air_gap, self.stage[2]]) 
                self.target = np.array([self.skin_entry[0], self.skin_entry[1]+self.insertion_length, self.skin_entry[2]]) 
            # Place robot at initial point
            self.get_logger().info('Place robot at initial point...')
            self.send_cmd(self.skin_entry[0], self.stage[1], self.skin_entry[2])

#### Service server functions ###################################################

    # Provide skin_entry point
    def get_skin_entry_callback(self, request, response):
        self.get_logger().debug('Received skin_entry request')
        if (self.skin_entry.size == 0):
            response.valid = False
        else:
            response.valid = True
            response.x = self.skin_entry[0]
            response.y = self.skin_entry[1]
            response.z = self.skin_entry[2]    
        return response    

    # Provide target point
    def get_target_callback(self, request, response):
        self.get_logger().debug('Received target request')
        if (self.target.size == 0):
            response.valid = False
        else:
            response.valid = True
            response.x = self.target[0]
            response.y = self.target[1]
            response.z = self.target[2]    
        return response  
    
    # Provide initial point
    def get_initial_point_callback(self, request, response):
        self.get_logger().debug('Received initial_point request')
        if (self.initial_point.size == 0):
            response.valid = False
        else:
            response.valid = True
            response.x = self.initial_point[0]
            response.y = self.initial_point[1]
            response.z = self.initial_point[2]    
        return response    

#### Action client functions ###################################################

    # Send MoveStage action to Stage
    def send_cmd(self, x, y, z):
        # Send command to stage (convert mm to m)
        self.robot_idle = False     # Set robot status to NOT IDLE
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.z = float(z)
        goal_msg.eps = 0.5
        self.get_logger().info('Send goal request... Control u: x=%.4f, y=%.4f, z=%.4f' % (x, y, z))

        # Wait for action server
        self.action_client.wait_for_server()        
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    # Check if MoveStage action was accepted 
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Get MoveStage action finish message (Result)
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached: %.4f, %.4f, %.4f' %(result.x, result.y, result.z))
            self.stage = np.array([result.x, result.y, result.z])
            # Set robot initial point
            if (self.initial_point.size == 0):
                self.initial_point = np.copy(self.stage)
                self.get_logger().info('Robot at initial position: %s' %(self.initial_point)) 
                self.listen_keyboard == False
                 # Place needle at skin_entry
                self.get_logger().info('Place needle at skin_entry...')
                self.send_cmd(self.skin_entry[0], self.skin_entry[1], self.skin_entry[2])
            else:
                # Activate planning service servers
                self.get_logger().info('Needle at skin_entry: %s' %(self.skin_entry)) 
                self.skin_entry_server = self.create_service(GetPoint, '/planning/get_skin_entry', self.get_skin_entry_callback)
                self.target_server = self.create_service(GetPoint, '/planning/get_target', self.get_target_callback)
                self.initial_point_server = self.create_service(GetPoint, '/planning/get_initial_point', self.get_initial_point_callback)
                self.get_logger().info('Planning services are now available')
                self.get_logger().info('***** Initialization Sucessfull *****')
        elif result.error_code == 1:
            self.get_logger().info('Goal failed: TIMETOUT')
        self.robot_idle = True       # Set robot status to IDLE
            
############################################################Pointrray [x, y, z, qw, qx, qy, qz])
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

    planning.get_logger().info('Waiting robot to connect...')
    # Wait for robot
    while rclpy.ok():
        rclpy.spin_once(planning)
        if(planning.stage.size == 0): # Keep loop while stage position not set
            pass
        else:
            if (planning.use_slicer is False):
                planning.get_logger().info('Robot current position = %s' %(planning.stage))
                planning.get_logger().warn('**** Hit SPACE to initialize experiment ****')
                planning.get_logger().warn('REMEMBER: Use another terminal to run keypress node')
                planning.listen_keyboard = True
            break

    if (planning.use_slicer is True):
        planning.get_logger().warn('**** Waiting planning points from 3DSlicer ****')
    # Wait for skin_entry to be defined
    while rclpy.ok():
        rclpy.spin_once(planning)
        if(planning.skin_entry.size == 0): # Keep loop while skin_entry not set
            pass
        else:
            if (planning.use_slicer is True):
                initial_point = np.array([planning.skin_entry[0], planning.stage[1], planning.skin_entry[2]])
                planning.get_logger().info('Planned robot initial point = %s' %(initial_point))
                planning.get_logger().warn('**** To position robot and initialize experiment, hit SPACE ****')
                planning.get_logger().warn('REMEMBER: Use another terminal to run keypress node')
                planning.listen_keyboard = True
            break

    rclpy.spin(planning)    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    planning.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()