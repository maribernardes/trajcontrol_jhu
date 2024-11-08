import rclpy
import numpy as np

from std_msgs.msg import Int8
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PointStamped
from smart_template_interfaces.action import MoveStage
from smart_template_interfaces.srv import ControllerCommand
from smart_template_interfaces.srv import GetPoint, GetPose

from functools import partial

#########################################################################
#
# Controller Manual Smart
#
# Description:
# This node manually controls the SmartTemplate robot
#
# Subscribes:   
# '/keyboard/key'               (std_msgs.msg.Int8)
# '/stage/state/guide_pose'     (geometry_msgs.msg.PointStamped)  - robot frame
#
# Action/service client:
# '/move_stage'             (smart_template_interfaces.action.MoveStage) - robot frame
# '/command'                (smart_template_interfaces.action.MoveStage) - robot frame
# 
#########################################################################

class ControllerManualSmart(Node):

    def __init__(self):
        super().__init__('controller_manual_smart')

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

        self.robot_idle = False

#### Subscribed topics ###################################################

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PointStamped, '/stage/state/guide_pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

#### Action/service client ###################################################

        # Action client 
        self.action_client = ActionClient(self, MoveStage, '/stage/move')
        if not self.action_client.server_is_ready():
            self.get_logger().info('/stage/move action not available, waiting...')
        while not self.action_client.wait_for_server(timeout_sec=5.0):
            pass
        # Service client
        self.service_client = self.create_client(ControllerCommand, '/stage/command')
        if not self.service_client.service_is_ready():
            self.get_logger().info('/stage/command service not available, waiting...')
        while not self.service_client.wait_for_service(timeout_sec=5.0):
            pass     

#### Listening callbacks ###################################################

    # Get robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.point
        self.stage = np.array([robot.x, robot.y, robot.z])

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        # Only takes new control input after converged to previous
        if (self.robot_idle is True) and (self.stage.size != 0):
            x = self.stage[0]
            y = self.stage[1]
            z = self.stage[2]
            # Send move_stage action
            if (msg.data == 50): # move down
                z = z - self.lateral_step
                self.get_logger().info('Current = %s' %self.stage)
                self.get_logger().info('Move DOWN = [%f, %f, %f]' %(x,y,z))
                self.move_stage(x, y, z) 
            elif (msg.data == 52): # move left
                x = x - self.lateral_step
                self.get_logger().info('Current = %s' %self.stage)
                self.get_logger().info('Move LEFT = [%f, %f, %f]' %(x,y,z))
                self.move_stage(x, y, z) 
            elif (msg.data == 54): # move right
                x = x + self.lateral_step
                self.get_logger().info('Current = %s' %self.stage)
                self.get_logger().info('Move RIGHT = [%f, %f, %f]' %(x,y,z))
                self.move_stage(x, y, z) 
            elif (msg.data == 56): # move up
                z = z + self.lateral_step
                self.get_logger().info('Current = %s' %self.stage)
                self.get_logger().info('Move UP = [%f, %f, %f]' %(x,y,z))
                self.move_stage(x, y, z) 
            elif (msg.data == 43): # insert one step
                y = y + self.insertion_step
                self.get_logger().info('Current = %s' %self.stage)
                self.get_logger().info('Move IN = [%f, %f, %f]' %(x,y,z))
                self.move_stage(x, y, z) 
            elif (msg.data == 45): # retract one step
                y = y - self.insertion_step
                self.get_logger().info('Current = %s' %self.stage)
                self.get_logger().info('Move OUT = [%f, %f, %f]' %(x,y,z))
                self.move_stage(x, y, z) 
            # Send command service
            elif (msg.data == 65): # abort
                self.get_logger().info('Current = %s' %self.stage)
                self.get_logger().info('ABORT')
                self.command('ABORT')
            elif (msg.data == 72): # home
                self.get_logger().info('Current = %s' %self.stage)
                self.get_logger().info('Command HOME')
                self.command('HOME')
            elif (msg.data == 82): # retract
                self.get_logger().info('Current = %s' %self.stage)
                self.get_logger().info('Command RETRACT')
                self.command('RETRACT')

#### Service client functions ###################################################

    # Send Command service to robot 
    def command(self, cmd_string):
        self.robot_idle = False
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
        self.robot_idle = True

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
        self.get_logger().debug('Send goal request... Control u: x=%.4f, y=%.4f, z=%.4f' % (x, y, z))
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
            self.get_logger().info('Reached: [%.4f, %.4f, %.4f]' %(result.x, result.y, result.z))
        elif result.error_code == 1:
            self.get_logger().info('Goal failed: TIMETOUT')
        self.robot_idle = True       # Set robot status to IDLE

#########################################################################

def main(args=None):
    rclpy.init(args=args)
    controller_manual_smart = ControllerManualSmart()

    # Wait for experiment initialization
    if controller_manual_smart.wait_initialization is True:
        controller_manual_smart.get_logger().info('/planning/get_initial_point service not available, waiting...')
        while rclpy.ok():
            rclpy.spin_once(controller_manual_smart)
            if not controller_manual_smart.planning_client.service_is_ready():
                pass
            else:
                controller_manual_smart.planning_client.destroy()
                break

    controller_manual_smart.get_logger().info('All required actions and services available')
    controller_manual_smart.robot_idle = True  

    controller_manual_smart.get_logger().warn('***** Ready to manually control *****')
    controller_manual_smart.get_logger().warn('Use numerical keyboad to move template step-wise')
    controller_manual_smart.get_logger().warn('Arrows for lateral motion and +/- to insert/retract')
    controller_manual_smart.get_logger().warn('R = RETRACT, H = HOME, A = ABORT')
    controller_manual_smart.get_logger().warn('Step size: Lateral = %.1f mm / Insertion = %.1f mm' % (controller_manual_smart.lateral_step,controller_manual_smart.insertion_step))
    

    rclpy.spin(controller_manual_smart)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_manual_smart.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()