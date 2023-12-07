import rclpy
import numpy as np
import time

from std_msgs.msg import Int8
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from smart_control_interfaces.action import MoveStage
from smart_control_interfaces.srv import GetPoint

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from functools import partial

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
# '/planning/get_skin_entry'     (smart_control_interfaces.srv.GetPoint) - robot frame
# '/planning/get_target'         (smart_control_interfaces.srv.GetPoint) - robot frame
#
# Action client:
# 'stage/move'              (smart_control_interfaces.action.MoveStage) - robot frame
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
        
        self.robot_idle = False       # Flag for move robot
        self.step = 0                 # Current insertion step         

    #### Action/service clients ###################################################
 
        # OBS: Node will not spin until servers are available
        # /stage/move Action client
        self.move_action_client = ActionClient(self, MoveStage, '/stage/move', callback_group=MutuallyExclusiveCallbackGroup())
        if not self.move_action_client.server_is_ready():
            self.get_logger().warn('/stage/move action not available, waiting...')
        while not self.move_action_client.wait_for_server(timeout_sec=5.0):
            pass
        # /planning/get_target Service client
        self.target_service_client = self.create_client(GetPoint, '/planning/get_target', callback_group=MutuallyExclusiveCallbackGroup())
        if not self.target_service_client.service_is_ready():
            self.get_logger().warn('/planning/get_target service not available, waiting...')
        while not self.target_service_client.wait_for_service(timeout_sec=5.0):
            pass     
        # /planning/get_skin_entry Service client
        self.skin_entry_service_client = self.create_client(GetPoint, '/planning/get_skin_entry', callback_group=MutuallyExclusiveCallbackGroup())
        if not self.skin_entry_service_client.service_is_ready():
            self.get_logger().warn('/planning/get_skin_entry service not available, waiting...')
        while not self.skin_entry_service_client.wait_for_service(timeout_sec=5.0):
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

    def move_step(self):
        # Calculate step goal
        max_depth = self.target[1]
        step_depth = self.skin_entry[1] + self.insertion_step*self.step
        x = self.skin_entry[0]
        y = min(max_depth, step_depth)
        z = self.skin_entry[2]
        # Send to stage
        self.move_stage(x, y, z) 

#### Listening callbacks ###################################################

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        if (msg.data == 32):
            if (self.robot_idle is True):
                self.next_step()
            else:
                self.get_logger().info('Motion not available')

#### Service client functions ###################################################

    # Update target (non blocking service request)
    def next_step(self):
        # Get current target
        request = GetPoint.Request()
        future = self.target_service_client.call_async(request)
        # When target request done, do callback
        future.add_done_callback(partial(self.update_target_callback))
        return future.result()

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

    # Update target service response message (Response)
    def update_target_callback(self, future):
        try:
            target = future.result()
            if target.valid is True:
                self.target = np.array([target.x, target.y, target.z])
                if (self.robot_idle is True):
                    self.step += 1
                    self.get_logger().info('STEP #%i' %self.step)
                    self.get_logger().info('Target: %s' %self.target)
                    self.move_step()
            else:
                self.get_logger().error('Invalid target')
        except Exception as e:
            self.get_logger().error('Service call failed: %r' %(e,))

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
        self.get_logger().info('Move request: %.4f, %.4f, %.4f' % (x, y, z))
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
            self.get_logger().info('Goal reached: %.4f, %.4f, %.4f' %(result.x, result.y, result.z))
            self.stage = np.array([result.x, result.y, result.z])
            self.get_logger().warn('Ready for next step. Hit SPACE')
        elif result.error_code == 1:
            self.get_logger().info('Move failed: TIMETOUT')
        self.robot_idle = True       # Set robot status to IDLE

#########################################################################

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