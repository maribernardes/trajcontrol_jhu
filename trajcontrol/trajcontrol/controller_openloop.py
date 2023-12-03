import rclpy
import numpy as np
import time

from std_msgs.msg import Int8
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from smart_control_interfaces.action import MoveStage
from smart_control_interfaces.srv import GetPoint

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
# '/stage/get_position'          (smart_control_interfaces.srv.GetPoint) - robot frame
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
        self.stage = np.empty(shape=[3,0])         # Current stage position

        self.robot_idle = False         # Flag for move robot
        self.step = 0                   # Current insertion step         

    #### Subscribed topics ###################################################

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning

    #### Action/service clients ###################################################
 
        # OBS: Node will not spin until servers are available
        # /stage/move Action client
        self.move_action_client = ActionClient(self, MoveStage, '/stage/move')
        if not self.move_action_client.server_is_ready():
            self.get_logger().warn('/stage/move action not available, waiting...')
        while not self.move_action_client.wait_for_server(timeout_sec=5.0):
            pass
        # /stage/get_position Service client
        self.stage_position_service_client = self.create_client(GetPoint, '/stage/get_position')
        if not self.stage_position_service_client.service_is_ready():
            self.get_logger().warn('/stage/get_position service not available, waiting...')
        while not self.stage_position_service_client.wait_for_service(timeout_sec=5.0):
            pass   
        # /planning/get_target Service client
        self.target_service_client = self.create_client(GetPoint, '/planning/get_target')
        if not self.target_service_client.service_is_ready():
            self.get_logger().warn('/planning/get_target service not available, waiting...')
        while not self.target_service_client.wait_for_service(timeout_sec=5.0):
            pass     
        # /planning/get_skin_entry Service client
        self.skin_entry_service_client = self.create_client(GetPoint, '/planning/get_skin_entry')
        if not self.skin_entry_service_client.service_is_ready():
            self.get_logger().warn('/planning/get_skin_entry service not available, waiting...')
        while not self.skin_entry_service_client.wait_for_service(timeout_sec=5.0):
            pass      
        self.get_logger().info('Action and services available')

    #### Initialize variables ###################################################

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        self.insertion_step = self.get_parameter('insertion_step').get_parameter_value().double_value

        # Initialize target and initial_point
        self.skin_entry = self.get_skin_entry()
        self.target = self.get_target()
        self.stage = self.get_stage_position()
        self.get_logger().info('Loaded target')

        # Make robot available for motion request
        self.robot_idle = True

#### Internal functions ###################################################

    # Get skin_entry
    def get_target(self):
        target = self.request_target()
        if target.valid is True:
            return np.array([target.x, target.y, target.z])
        else:
            self.get_logger().error('Invalid target')
            return None

    # Get skin_entry
    def get_skin_entry(self):
        skin_entry = self.request_skin_entry()
        if skin_entry.valid is True:
            return np.array([skin_entry.x, skin_entry.y, skin_entry.z])
        else:
            self.get_logger().error('Invalid skin_entry')
            return None

    # Get stage
    def get_stage_position(self):
        stage = self.request_stage_position()
        if stage.valid is True:
            return np.array([stage.x, stage.y, stage.z])
        else:
            self.get_logger().error('Invalid stage position')
            return None

#### Listening callbacks ###################################################

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        # Only takes new control input after converged to previous
        if (self.robot_idle is True) and (msg.data == 32):
            self.step += 1
            self.get_logger().info('STEP #%i' %self.step)
            # Calculate step goal
            max_depth = self.target[1]
            step_depth = self.skin_entry[1] + self.insertion_step*self.step
            x = self.skin_entry[0]
            y = min(max_depth, step_depth)
            z = self.skin_entry[2]
            # Send to stage
            self.move_stage(x, y, z) 
            
        elif (self.robot_idle is False):
            self.get_logger().info('Motion not available')

#### Service client functions ###################################################

    # Request target
    def request_target(self):
        request = GetPoint.Request()
        future = self.target_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    # Update target
    def update_target(self):
        request = GetPoint.Request()
        future = self.target_service_client.call_async(request)
        # rclpy.spin_until_future_complete(self, future)
        future.add_done_callback(partial(self.get_target_callback))
        return future.result()

    # Request skin_entry
    def request_skin_entry(self):
        request = GetPoint.Request()
        future = self.skin_entry_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    # Get Command service response message (Response)
    def get_target_callback(self, future):
        try:
            target = future.result()
            target = self.request_target()
            if target.valid is True:
                self.target = np.array([target.x, target.y, target.z])
            else:
                self.get_logger().error('Invalid target')
                return None
        except Exception as e:
            self.get_logger().error('Service call failed: %r' %(e,))

    # Request stage
    def request_stage_position(self):
        request = GetPoint.Request()
        future = self.stage_position_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

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
        self.get_logger().info('Send goal request... Control u: x=%.4f, y=%.4f, z=%.4f' % (x, y, z))
        # Wait for action server
        self.move_action_client.wait_for_server()        
        self.send_goal_future = self.move_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.move_response_callback)

    # Check if MoveStage action was accepted 
    def move_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_move_result_callback)

    # Get MoveStage action finish message (Result)
    def get_move_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached: %.4f, %.4f, %.4f' %(result.x, result.y, result.z))
            self.stage = np.array([result.x, result.y, result.z])
        elif result.error_code == 1:
            self.get_logger().info('Goal failed: TIMETOUT')
        self.robot_idle = True       # Set robot status to IDLE

#########################################################################

def main(args=None):
    rclpy.init(args=args)
    controller_openloop = ControllerOpenLoop()

    # # Wait for experiment initialization
    # while rclpy.ok():
    #     rclpy.spin_once(controller_openloop)
    #     if not controller_openloop.planning_client.service_is_ready():
    #         pass
    #     else:
    #         break

    controller_openloop.get_logger().info('Target: %s' %controller_openloop.target)
    controller_openloop.get_logger().info('Stage: =%s' %controller_openloop.skin_entry)
    controller_openloop.get_logger().info('Skin Entry: %s' %controller_openloop.stage)
    controller_openloop.get_logger().warn('***** Ready to INSERT - Open loop *****')
    controller_openloop.get_logger().warn('Use SPACE to signal each insertion step')

    rclpy.spin(controller_openloop)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_openloop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()