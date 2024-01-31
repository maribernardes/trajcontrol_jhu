import rclpy
import numpy as np
import time
import math
import quaternion
import os

from std_msgs.msg import Int8
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from smart_control_interfaces.action import MoveStage
from smart_control_interfaces.srv import GetPoint, GetPose

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from scipy.io import savemat

from ament_index_python.packages import get_package_share_directory
from functools import partial
from .utils import get_angles

#########################################################################
#
# Cube Controller Node
#
# Description:
# This node implements the cube control.
# It moves the robot to the vertices of a cube centered at a given position
#
# Subscribes:   
# '/keyboard/key'           (std_msgs.msg.Int8)
#
# Service client:    
# '/planning/get_skin_entry' (smart_control_interfaces.srv.GetPoint) - robot frame
# '/planning/get_target'     (smart_control_interfaces.srv.GetPoint) - robot frame
# '/stage/get_position'      (smart_control_interfaces.srv.GetPoint) - robot frame
#
# Action client:
# 'stage/move'               (smart_control_interfaces.action.MoveStage) - robot frame
#
##########################################################################

GALIL_ERR = 0.5

class ControllerCube(Node):

    def __init__(self):
        super().__init__('controller_cube')

        #Declare node parameters
        self.declare_parameter('cube_size', 50.0)           # Cube size
        self.declare_parameter('fiducial_offset', 20.0)      # Fiducial offset from needle guide
        self.declare_parameter('filename', 'cube_data') # Name of file where data values are saved

    #### Stored variables ###################################################

        # Stored values
        self.center = np.empty(shape=[3,0])     # Center of cube (target from 3DSlicer)
        self.stage = np.empty(shape=[0,3])      # Current base input [x, y, z]
        self.cmd = np.empty(shape=[0,3])        # Control output to the robot stage

        self.step = 0                       # Current insertion step         

        self.wait_stage = False       # Flag to wait for tip value
        self.wait_tip = False         # Flag to wait for base value
        self.wait_finish = False      # Flag to wait for end of insertion
        self.robot_idle = False       # Flag for move robot

    #### Action/service clients ###################################################
 
        # OBS: Node will not spin until servers are available
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
        # /stage/get_position Service client
        self.stage_position_service_client = self.create_client(GetPoint, '/stage/get_position', callback_group=MutuallyExclusiveCallbackGroup() )
        if not self.stage_position_service_client.service_is_ready():
            self.get_logger().info('/stage/get_position service not available, waiting...')
        while not self.stage_position_service_client.wait_for_service(timeout_sec=5.0):
            pass                

        self.get_logger().info('Action and services available')

    #### Initialize variables ###################################################

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        # Get sequence parameters
        offset = self.get_parameter('fiducial_offset').get_parameter_value().double_value
        delta = 0.5*self.get_parameter('cube_size').get_parameter_value().double_value

        # Set .mat filename and path
        trajcontrol_share_directory = get_package_share_directory('trajcontrol') 
        self.filename = os.path.join(trajcontrol_share_directory,'data',self.get_parameter('filename').get_parameter_value().string_value + '.mat') #String with full path to file
        self.get_logger().info('Saving experiment data to %s' %self.filename)
        
        # Initialize center
        self.center = self.get_target()             # get_target is a blocking service request
        self.center[1] = self.center[1] - offset
        self.get_logger().info('Cube center: (%.4f, %.4f, %.4f)' %(self.center[0], self.center[1], self.center[2]))
        self.robot_idle = True

        # Define sequence steps
        self.sequence = np.array([[0,0,0],[-delta,-delta,-delta],[delta,-delta,-delta],[delta,-delta,delta],[-delta,-delta,delta],[-delta,delta,-delta],[delta,delta,-delta],[delta,delta,delta],[-delta,delta,delta]])                    
        self.ns = self.sequence.shape[0]
        self.get_logger().info('Cube size: %.2f mm' %(2*delta))
        self.get_logger().info('Fiducial offset: %.2f mm' %(offset))
        self.get_logger().info('Total sequence: %i steps' %(self.ns))

        # Experiment data (save to .mat file)
        self.stage_data = np.zeros((1, 3, 0))     # Stage input before control action
        self.goal_data = np.zeros((1, 3, 0))      # Expected tip pose
        self.cmd_data = np.zeros((1, 3, 0))       # Step control action

    #### Subscribed topics ###################################################

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning

#### Internal functions ###################################################

    def next_step(self):
        # Update system inputs (tip and base) - To compare with closed-loop control
        # Responses trigger move_step
        self.wait_stage = True
        self.update_stage()

    def move_step(self):
        self.robot_idle = False
        self.step += 1
        # Calculate step goal
        goal = self.center + self.sequence[self.step-1,:]
        x = goal[0]
        y = goal[1]
        z = goal[2]
        self.cmd = np.array([x,y,z])
        # Send to stage
        self.move_stage(x, y, z) 
        # Save .mat file
        self.stage_data = np.dstack((self.stage_data , self.stage))
        self.cmd_data = np.dstack((self.cmd_data , self.cmd))
        savemat(self.filename, {'center':self.center, 'stage': self.stage_data, 'cmd':self.cmd_data})

    def finish_insertion(self):
        # Save last values into .mat file
        self.stage_data = np.dstack((self.stage_data , self.stage))
        savemat(self.filename, {'center':self.center, 'stage': self.stage_data, 'cmd':self.cmd_data})
        # Print last values
        self.get_logger().info('\n****** FINAL ******\nStage: (%.4f, %.4f, %.4f)\nPrev Cmd: (%.4f, %.4f, %.4f)\n*********************' \
            %(self.stage[0], self.stage[1], self.stage[2], \
            self.cmd[0], self.cmd[1], self.cmd[2])
        )
        self.get_logger().warn('Press Ctrl+C to finish')

#### Listening callbacks ###################################################

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        if (msg.data == 32):
            if (self.robot_idle is True):
                self.get_logger().warn('Starting step #%i...' %(self.step+1))  
                self.next_step()
            else:
                self.get_logger().info('Motion not available')
        elif (msg.data == 10):
            if (self.wait_finish is True):
                self.update_stage()

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
                self.wait_stage = False
            else:
                self.get_logger().error('Invalid stage position')
        except Exception as e:
            self.get_logger().error('Stage call failed: %r' %(e,))
        if (self.wait_finish is True):
            self.finish_insertion()
        elif  (self.wait_stage is False) and (self.robot_idle is True):
            self.move_step() 

#### Action client functions ###################################################

    # Send MoveStage action to robot
    def move_stage(self, x, y, z):
        # Send command to stage (in mm)
        self.robot_idle = False     # Set robot status to NOT IDLE
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.z = float(z)
        goal_msg.eps = GALIL_ERR # in mm
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
            # Print values
            self.get_logger().info('\n****** STEP #%i ******\nStage: (%.4f, %.4f, %.4f)\nCmd: (%.4f, %.4f, %.4f)\
                \nReached: (%.4f, %.4f, %.4f)\n*********************' \
                %(self.step, \
                self.stage[0], self.stage[1], self.stage[2],\
                self.cmd[0], self.cmd[1], self.cmd[2], \
                result.x, result.y, result.z)
            )
            self.stage = np.array([result.x, result.y, result.z])
            if (self.step >= self.ns):
                self.robot_idle = False
                self.wait_finish = True
                self.get_logger().warn('End of sequence. Hit ENTER to finish')
            else:
                self.robot_idle = True 
                self.get_logger().warn('Hit SPACE for next step')
        elif result.error_code == 1:
            self.get_logger().info('Move failed: TIMETOUT')

########################################################################

def main(args=None):
    rclpy.init(args=args)
    controller_cube = ControllerCube()

    executor = MultiThreadedExecutor()

    controller_cube.get_logger().warn('***** Ready to INSERT - Cube *****')
    controller_cube.get_logger().warn('Use SPACE to signal each cube step')

    try:
        rclpy.spin(controller_cube, executor=executor)
    except KeyboardInterrupt:
        controller_cube.get_logger().info('Keyboard interrupt, shutting down.\n')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_cube.destroy_node()  

    rclpy.shutdown()

if __name__ == '__main__':
    main()