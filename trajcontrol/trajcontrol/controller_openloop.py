import rclpy
import numpy as np
import time
import math
import quaternion

from std_msgs.msg import Int8
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from smart_control_interfaces.action import MoveStage
from smart_control_interfaces.srv import GetPoint, GetPose

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from functools import partial
from .utils import get_angles

#########################################################################
#
# Open-loop Controller Node
#
# Description:
# This node implements the open-loop control.
# It gets the desired target to calculate the total insertion_length and
# pushes the needle step_wise at each keyboard SPACE input (NO LATERAL MOTION - Only insertion)
#
# Subscribes:   
# '/keyboard/key'           (std_msgs.msg.Int8)
#
# Service client:    
# '/planning/get_skin_entry' (smart_control_interfaces.srv.GetPoint) - robot frame
# '/planning/get_target'     (smart_control_interfaces.srv.GetPoint) - robot frame
# '/stage/get_position'      (smart_control_interfaces.srv.GetPoint) - robot frame
# '/needle/get_tip'          (smart_control_interfaces.srv.GetPose) - robot frame
#
# Action client:
# 'stage/move'               (smart_control_interfaces.action.MoveStage) - robot frame
#
##########################################################################

class ControllerOpenLoop(Node):

    def __init__(self):
        super().__init__('controller_openloop')

        #Declare node parameters
        self.declare_parameter('insertion_step', 10.0)          # Insertion step parameter

    #### Stored variables ###################################################

        # Stored values
        self.skin_entry = np.empty(shape=[3,0])    # Stage position after experiment initialization
        self.target = np.empty(shape=[3,0])        # Planned target point

        self.tip = np.empty(shape=[0,5])        # Current tip input  [x, y, z, angle_h, angle_v]
        self.stage = np.empty(shape=[0,3])      # Current base input [x, y, z]
        self.cmd = np.empty(shape=[0,3])        # Control output to the robot stage
        
        self.step = 0                       # Current insertion step         
        self.depth = 0.0                    # Current insertion depth = stage[1] - skin_entry[1]

        self.wait_stage = False       # Flag to wait for tip value
        self.wait_tip = False         # Flag to wait for base value
        self.robot_idle = False       # Flag for move robot

    #### Action/service clients ###################################################
 
        # OBS: Node will not spin until servers are available
        jacobian_inputs_callgroup = MutuallyExclusiveCallbackGroup() 
        # /stage/move Action client
        self.move_action_client = ActionClient(self, MoveStage, '/stage/move', callback_group=MutuallyExclusiveCallbackGroup())
        if not self.move_action_client.server_is_ready():
            self.get_logger().info('/stage/move action not available, waiting...')
        while not self.move_action_client.wait_for_server(timeout_sec=5.0):
            pass
        # /planning/get_target Service client
        self.target_service_client = self.create_client(GetPoint, '/planning/get_target', callback_group=MutuallyExclusiveCallbackGroup())
        if not self.target_service_client.service_is_ready():
            self.get_logger().info('/planning/get_target service not available, waiting...')
        while not self.target_service_client.wait_for_service(timeout_sec=5.0):
            pass     
        # /planning/get_skin_entry Service client
        self.skin_entry_service_client = self.create_client(GetPoint, '/planning/get_skin_entry', callback_group=MutuallyExclusiveCallbackGroup())
        if not self.skin_entry_service_client.service_is_ready():
            self.get_logger().info('/planning/get_skin_entry service not available, waiting...')
        while not self.skin_entry_service_client.wait_for_service(timeout_sec=5.0):
            pass      
        # /stage/get_position Service client
        self.stage_position_service_client = self.create_client(GetPoint, '/stage/get_position', callback_group=jacobian_inputs_callgroup)
        if not self.stage_position_service_client.service_is_ready():
            self.get_logger().info('/stage/get_position service not available, waiting...')
        while not self.stage_position_service_client.wait_for_service(timeout_sec=5.0):
            pass          
        # /needle/get_tip Service client
        self.tip_service_client = self.create_client(GetPose, '/needle/get_tip', callback_group=jacobian_inputs_callgroup)
        if not self.tip_service_client.service_is_ready():
            self.get_logger().info('/needle/get_tip service not available, waiting...')
        while not self.tip_service_client.wait_for_service(timeout_sec=5.0):
            pass  

        self.get_logger().info('Action and services available')

    #### Initialize variables ###################################################

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        self.insertion_step = self.get_parameter('insertion_step').get_parameter_value().double_value

        # Initialize skin_entry
        self.get_logger().info('Initializing skin_entry and target')
        self.skin_entry = self.get_skin_entry()     # get_skin is a blocking service request
        self.target = self.get_target()     # get_skin is a blocking service request
        self.robot_idle = True

    #### Subscribed topics ###################################################

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning

#### Internal functions ###################################################

    def next_step(self):
        # Update system inputs (tip and base) - To compare with closed-loop control
        # Responses trigger move_step
        self.wait_stage = True
        self.wait_tip = True
        self.update_stage()
        self.update_tip()

    def move_step(self):
        self.robot_idle = False
        self.step += 1
        # Calculate step goal
        step_depth = self.skin_entry[1] + self.insertion_step*self.step
        x = self.target[0]
        y = min(self.target[1], step_depth)
        z = self.target[2]
        self.cmd = np.array([x,y,z])
        # Send to stage
        self.move_stage(x, y, z) 

#### Listening callbacks ###################################################

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        if (msg.data == 32):
            if (self.robot_idle is True):
                self.get_logger().warn('Start next step...')
                self.next_step()
            else:
                self.get_logger().info('Motion not available')

#### Service client functions ###################################################

    # Get target (blocking service request)
    def get_target(self):
        request = GetPoint.Request()
        future = self.target_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        target = future.result()
        if target.valid is True:
            return np.array([target.x, target.y, target.z])
        else:
            self.get_logger().error('Invalid target')
            return None

    # Get skin_entry (blocking service request)
    def get_skin_entry(self):
        request = GetPoint.Request()
        future = self.skin_entry_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        skin_entry = future.result()
        if skin_entry.valid is True:
            return np.array([skin_entry.x, skin_entry.y, skin_entry.z])
        else:
            self.get_logger().error('Invalid skin_entry')
            return None

    # Update stage (non blocking service request)
    def update_stage(self):
        request = GetPoint.Request()
        future = self.stage_position_service_client.call_async(request)
        # When stage request done, do callback
        future.add_done_callback(partial(self.update_stage_callback))
        return future.result()

    # Update tip (non blocking service request)
    def update_tip(self):
        request = GetPose.Request()
        future = self.tip_service_client.call_async(request)
        # When tip request done, do callback
        future.add_done_callback(partial(self.update_tip_callback))
        return future.result()

    # Update stage service response message (Response)
    def update_stage_callback(self, future):
        try:
            stage = future.result()
            if stage.valid is True:
                self.stage = np.array([stage.x, stage.y, stage.z])
                self.depth = self.stage[1] - self.skin_entry[1]
                self.wait_stage = False
            else:
                self.get_logger().error('Invalid stage position')
        except Exception as e:
            self.get_logger().error('Stage call failed: %r' %(e,))
        if (self.wait_stage is False) and (self.wait_tip is False) and (self.robot_idle is True):
            self.move_step()

    # Update tip service response message (Response)
    def update_tip_callback(self, future):
        try:
            tip = future.result()
            if tip.valid is True:
                p = np.array([tip.x, tip.y, tip.z])
                q = np.array([tip.qw, tip.qx, tip.qy, tip.qz])
                self.get_logger().info('Tip p (robot) = %s' %p)
                self.get_logger().info('Tip q (robot) = %s' %q)
                angles = get_angles(q)
                self.tip = np.array([tip.x, tip.y, tip.z, angles[0],angles[1]])
                self.wait_tip = False
            else:
                self.get_logger().error('Invalid tip pose')
        except Exception as e:
            self.get_logger().error('Tip call failed: %r' %(e,))
        if (self.wait_stage is False) and (self.wait_tip is False) and (self.robot_idle is True):
            self.move_step()    

#### Action client functions ###################################################

    # Send MoveStage action to robot
    def move_stage(self, x, y, z):
        # Send command to stage (convert mm to m)
        self.robot_idle = False     # Set robot status to NOT IDLE
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.z = float(z)
        goal_msg.eps = 0.5 # in mm
        self.get_logger().debug('Move request: %.4f, %.4f, %.4f' % (x, y, z))
        # Wait for action server
        self.move_action_client.wait_for_server()        
        self.send_goal_future = self.move_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.move_response_callback)

    # Check if MoveStage action was accepted 
    def move_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Move rejected :(')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_move_result_callback)

    # Get MoveStage action finish message (Result)
    def get_move_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            error = self.target - self.tip[0:3]
            # Print values
            self.get_logger().info('\n****** STEP #%i ******\nInsertion depth: %f\nTarget: (%f, %f, %f)\nTip: (%f, %f, %f)\
                \nError: (%f, %f, %f / %f (%f deg), %f (%f deg))\nStage: (%f, %f, %f)\nCmd: (%f, %f, %f)\
                \nReached: (%.4f, %.4f, %.4f)\n*********************' \
                %(self.step, self.depth, \
                self.target[0], self.target[1], self.target[2],\
                self.tip[0],self.tip[1], self.tip[2],\
                error[0], error[1], error[2], self.tip[3], math.degrees(self.tip[3]), self.tip[4], math.degrees(self.tip[4]),\
                self.stage[0], self.stage[1], self.stage[2],\
                self.cmd[0], self.cmd[1], self.cmd[2], \
                result.x, result.y, result.z)
            )
            self.get_logger().warn('Hit SPACE for next step')
            self.stage = np.array([result.x, result.y, result.z])
        elif result.error_code == 1:
            self.get_logger().info('Move failed: TIMETOUT')
        self.robot_idle = True       # Set robot status to IDLE

########################################################################

def main(args=None):
    rclpy.init(args=args)
    controller_openloop = ControllerOpenLoop()

    executor = MultiThreadedExecutor()

    controller_openloop.get_logger().warn('***** Ready to INSERT - Open loop *****')
    controller_openloop.get_logger().warn('Use SPACE to signal each insertion step')

    try:
        rclpy.spin(controller_openloop, executor=executor)
    except KeyboardInterrupt:
        controller_openloop.get_logger().info('Keyboard interrupt, shutting down.\n')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_openloop.destroy_node()  

    rclpy.shutdown()

if __name__ == '__main__':
    main()