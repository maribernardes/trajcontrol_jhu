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
from smart_template_interfaces.action import MoveStage
from smart_template_interfaces.srv import GetPoint

from numpy import loadtxt
from ament_index_python.packages import get_package_share_directory
from .utils import *
from functools import partial

#########################################################################
#
# Planning Node
#
# Description:
# This node receives entry point message and uses it to
# align the SmartTemplate and push the needle to the skin entry point
#  
# Subscribes:   
# '/planning/skin_entry'  (geometry_msgs.msg.Point)
# '/stage/state/guide_pose' (geometry_msgs.msg.PointStamped)  - robot frame
#
# Service server:    
# '/planning/get_skin_entry'         (trajcontrol_interfaces.srv.GetPoint) - robot frame
# '/planning/get_target'             (trajcontrol_interfaces.srv.GetPoint) - robot frame
# '/planning/get_initial_point'      (trajcontrol_interfaces.srv.GetPoint) - robot frame
#
# Action client:
# 'stage/move'              (smart_template_interfaces.action.MoveStage) - robot frame
#
#########################################################################

class Targeting(Node):

    def __init__(self):
        super().__init__('targeting')
 
#### Stored variables ###################################################

        self.target = np.empty(shape=[0,3])         # User-defined tip position at desired target
        self.skin_entry = np.empty(shape=[0,3])     # User-defined tip position at skin entry
        
        self.initial_point = np.empty(shape=[0,3])  # Stage position at begining of experiment (aligned with no insertion)
        self.stage = np.empty(shape=[0,3])          # Stage positions: horizontal / depth / vertical 

        self.ready = False

#### Action/service clients ###################################################

        # Action client to move_stage
        self.action_client = ActionClient(self, MoveStage, '/stage/move')

        # /stage/get_position Service client
        self.stage_position_service_client = self.create_client(GetPoint, '/stage/get_position')
        if not self.stage_position_service_client.service_is_ready():
            self.get_logger().warn('/stage/get_position service not available, waiting...')
        while not self.stage_position_service_client.wait_for_service(timeout_sec=5.0):
            pass  

#### Subscribed topics ###################################################

        self.subscription_skin_entry = self.create_subscription(Point, '/planning/skin_entry', self.entry_callback,  10)
        self.subscription_skin_entry # prevent unused variable warning

#### Service server ##############################################

        # Service servers to return planning points
        self.skin_entry_server = None       # Activate only after robot is initialized (see function self.get_result_callback)
        self.target_server = None           # Activate only after robot is initialized (see function self.get_result_callback)
        self.initial_point_server = None    # Activate only after robot is initialized (see function self.get_result_callback)

#### Interface initialization ###################################################
        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

#### Listening callbacks ###################################################

    # Callback function for the entry point message
    # Stores entry and prepare for aiming and approaching stages
    def entry_callback(self, msg: Point):
        if self.skin_entry.size == 0:   # Get entry only once
            # Convert Point message to numpy array and update self.entry
            self.skin_entry = np.array([msg.x, msg.y, msg.z]).reshape(3, 1)
            self.get_logger().info('Skin entry = %f, %f, %f' %(self.skin_entry[0], self.skin_entry[1], self.skin_entry[2]))
            self.update_stage()

#### Service client functions ###################################################

    # Update stage (non blocking service request)
    def update_stage(self):
        request = GetPoint.Request()
        future = self.stage_position_service_client.call_async(request)
        # When stage request done, do callback
        future.add_done_callback(partial(self.update_stage_callback))
        return future.result()

    # Update stage service response message (Response)
    def update_stage_callback(self, future):
        try:
            stage = future.result()
            if stage.valid is True:
                self.stage = np.array([stage.x, stage.y, stage.z])
                self.get_logger().info('Robot = %f, %f, %f' %(self.stage[0], self.stage[1], self.stage[2]))
                # Start aiming phase
                self.get_logger().info('AIMING...')
                self.send_cmd(self.skin_entry[0], self.stage[1], self.skin_entry[2])
            else:
                self.get_logger().error('Invalid stage position')
        except Exception as e:
            self.get_logger().error('Stage call failed: %r' %(e,))

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
            self.get_logger().debug('Goal reached: %.4f, %.4f, %.4f' %(result.x, result.y, result.z))
            self.stage = np.array([result.x, result.y, result.z])
            # End of aiming phase
            if (self.initial_point.size == 0):
                self.initial_point = np.copy(self.stage)
                self.get_logger().info('Robot at initial position: %s' %(self.initial_point)) 
                # Start approaching phase
                self.get_logger().info('APPROACHING...')
                self.send_cmd(self.skin_entry[0], self.skin_entry[1], self.skin_entry[2])
            # End of approaching phase
            else:
                # Activate planning service servers
                self.ready = True
                self.get_logger().info('Robot actual position at skin entry: %.4f, %.4f, %.4f' %(result.x, result.y, result.z)) 
                self.skin_entry_server = self.create_service(GetPoint, '/planning/get_skin_entry', self.get_skin_entry_callback)
                self.target_server = self.create_service(GetPoint, '/planning/get_target', self.get_target_callback)
                self.initial_point_server = self.create_service(GetPoint, '/planning/get_initial_point', self.get_initial_point_callback)
                self.get_logger().info('Planning services are now available')
        elif result.error_code == 1:
            self.get_logger().info('Goal failed: TIMETOUT')
            

########################################################################

def main(args=None):
    rclpy.init(args=args)

    targeting = Targeting()   
    targeting.get_logger().warn('**** Waiting planning points from 3DSlicer ****')
    # Wait for skin_entry to be defined
    while rclpy.ok():
        rclpy.spin_once(targeting)
        if(targeting.ready == False): # Keep loop while not finished
            pass
        else:
            targeting.get_logger().warn('***** Targeting Sucessfull *****')
            break

    rclpy.spin(targeting)    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    targeting.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()