import rclpy
import numpy as np

from std_msgs.msg import Int8
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PointStamped
from smart_control_interfaces.action import MoveStage
from smart_control_interfaces.srv import ControllerCommand
from trajcontrol_interfaces.srv import GetPoint, GetPose

from functools import partial

#########################################################################
#
# Open-loop Controller Node
#
# Description:
# This node implements the step_wise mpc_control.
# It gets the delta in needle_base and in needle_tip to estimate 
# the model Jacobian.
# 
# Service server:    
# '/get_jacobian'             (trajcontrol_interfaces.srv.GetPoint) - robot frame
#  name: "initial_point", "skin_entry" OR "target"
#
# Service client:    
# '/get_needle_tip'             (trajcontrol_interfaces.srv.GetPose) - robot frame
#  OBS: request.name: "tip"
# '/get_needle_base'            (trajcontrol_interfaces.srv.GetPose) - robot frame
#  OBS: request.name: base"
#
# Service server:    
# '/get_jacobian'             (trajcontrol_interfaces.srv.GetMatrix) - robot frame
#  OBS: request.name: "jacobian"
#
#########################################################################

class ControllerOpenLoop(Node):

    def __init__(self):
        super().__init__('controller_openloop')

        #Declare node parameters
        self.declare_parameter('lateral_step', 1.0)             # Lateral motion step parameter
        self.declare_parameter('insertion_step', 10.0)          # Insertion step parameter
        self.declare_parameter('wait_init', False)              # Wait for experiment initialization before taking commands

#### Stored variables ###################################################

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        # Stored values
        self.initial_point = np.empty(shape=[3,0])  # Stage home position
        self.stage = np.empty(shape=[3,0])          # Current stage pose
        
        self.lateral_step = self.get_parameter('lateral_step').get_parameter_value().double_value
        self.insertion_step = self.get_parameter('insertion_step').get_parameter_value().double_value
        self.wait_initialization = self.get_parameter('wait_init').get_parameter_value().bool_value

        if self.wait_initialization is True:  # Stage status
            self.robot_idle = False                      
            self.planning_client = self.create_client(GetPoint, '/get_planning_point')
            self.get_logger().info('Waiting robot initialization...')
        else:
            self.robot_idle = True

#### Subscribed topics ###################################################

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PointStamped, '/stage/state/guide_pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

#### Action/service client ###################################################

        # Action client 
        self.action_client = ActionClient(self, MoveStage, '/move_stage')
        while not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('SmartTemplate action server not available, waiting again...')

        # Service client
        self.service_client = self.create_client(ControllerCommand, '/command')
        while not self.service_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('SmartTemplate service server not available, waiting again...')

#### Listening callbacks ###################################################

    # Get robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.point
        self.stage = np.array([robot.x, robot.y, robot.z])

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        # Only takes new control input after converged to previous
        if (self.robot_idle is True):
            x = self.stage[0]
            y = self.stage[1]
            z = self.stage[2]
            # Send move_stage action
            if (msg.data == 50): # move down
                z = z - self.lateral_step
                self.move_stage(x, y, z) 
            elif (msg.data == 52): # move left
                x = x - self.lateral_step
                self.move_stage(x, y, z) 
            elif (msg.data == 54): # move right
                x = x + self.lateral_step
                self.move_stage(x, y, z) 
            elif (msg.data == 56): # move up
                z = z + self.lateral_step
                self.move_stage(x, y, z) 
            elif (msg.data == 10): # insert one step
                y = y + self.insertion_step
                self.move_stage(x, y, z) 
            elif (msg.data == 32): # retract one step
                y = y - self.insertion_step
                self.move_stage(x, y, z) 
            # Send command service
            elif (msg.data == 65): # abort
                self.command('ABORT')
            elif (msg.data == 72): # home
                self.command('HOME')
            elif (msg.data == 82): # retract
                self.command('RETRACT')

#### Service client functions ###################################################

    # Send Command service to robot 
    def command(self, cmd_string):
        request = ControllerCommand.Request()
        request.command = cmd_string
        future = self.service_client.call_async(request)
        future.add_done_callback(partial(self.get_response_callback))

    # Get Command service response message (Response)
    def get_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Service call sucessful: %s' %(response.response)) 
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
        elif result.error_code == 1:
            self.get_logger().info('Goal failed: TIMETOUT')
        self.robot_idle = True       # Set robot status to IDLE

def main(args=None):
    rclpy.init(args=args)
    controller_openloop = ControllerOpenLoop()

    # Wait for experiment initialization
    while rclpy.ok():
        rclpy.spin_once(controller_openloop)
        if not controller_openloop.planning_client.service_is_ready():
            pass
        else:
            controller_openloop.planning_client.destroy()
            controller_openloop.robot_idle = True  
            break

    controller_openloop.get_logger().info('***** Ready to manually control *****')
    controller_openloop.get_logger().info('Use arrows from numerical keyboad to move template')
    controller_openloop.get_logger().info('Use enter/space to insert/retract the needle')
    controller_openloop.get_logger().info('H = HOME, R = RETRACT, A = ABORT')

    rclpy.spin(controller_openloop)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_openloop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()