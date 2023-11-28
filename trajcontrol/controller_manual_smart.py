import rclpy
import numpy as np

from std_msgs.msg import Int8
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PointStamped
from smart_control_interfaces.action import MoveStage

#########################################################################
#
# Controller Manual Smart
#
# Description:
# This node manually controls the SmartTemplate robot
#
# Subscribes:   
# '/keyboard/key'               (std_msgs.msg.Int8)
# '/stage/state/guide_pose'     (geometry_msgs.msg.PoseStamped)  - robot frame
# '/stage/initial_point'        (geometry_msgs.msg.PointStamped) - robot frame
#
# Action client:
# '/movestage'              (smart_control_interfaces.action.MoveStage) - robot frame
# 
#########################################################################

class ControllerManualSmart(Node):

#### Subscribed topics ###################################################
    def __init__(self):
        super().__init__('controller_manual_smart')

        #Declare node parameters
        self.declare_parameter('motion_step', 1.0) #Insertion length parameter

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning

        #Topics from interface node
        self.subscription_initial_point = self.create_subscription(PointStamped, '/stage/initial_point', self.initial_point_callback, 10)
        self.subscription_initial_point # prevent unused variable warning

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PoseStamped, '/stage/state/guide_pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

#### Action client ###################################################

        #Action client 
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

#### Stored variables ###################################################

        # Stored values
        self.stage_initial = np.empty(shape=[3,0])  # Stage home position
        self.stage = np.empty(shape=[3,0])          # Current stage pose
        self.robot_idle = False                     # Stage status

        self.motion_step = self.get_parameter('motion_step').get_parameter_value().double_value

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

#### Listening callbacks ###################################################

    # Get robot initial point
    def initial_point_callback(self, msg):
        # Stores robot initial position (only once)
        if (self.stage_initial.size == 0):
            initial_point = msg.point
            self.stage_initial = np.array([initial_point.x, initial_point.y, initial_point.z])
            self.robot_idle = True                  # Initialize robot status

    # Get robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.pose
        self.stage = np.array([robot.position.x, robot.position.y, robot.position.z])

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        # Only takes new control input after converged to previous
        if (self.robot_idle == True):
            x = self.stage[0]
            y = self.stage[1]
            z = self.stage[2]
            if (msg.data == 50): # move down
                z = z - self.motion_step
            elif (msg.data == 52): # move left
                x = x - self.motion_step
            elif (msg.data == 54): # move right
                x = x + self.motion_step
            elif (msg.data == 56): # move up
                z = z + self.motion_step
            elif (msg.data == 10): # insert one step
                y = y + self.motion_step
            elif (msg.data == 32): # retract one step
                y = y - self.motion_step
            # Send command to stage
            self.send_cmd(x, y, z)  

    # Send MoveStage action to Stage
    def send_cmd(self, x, y, z):
        # Send command to stage (convert mm to m)
        self.robot_idle = False     # Set robot status to NOT IDLE
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.z = float(z)
        goal_msg.eps = 0.0001
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
    controller_manual_smart = ControllerManualSmart()

    # Wait for experiment initialization
    while rclpy.ok():
        rclpy.spin_once(controller_manual_smart)
        if controller_manual_smart.stage_initial.size == 0: # Not initialized yet
            pass
        else:
            controller_manual_smart.get_logger().info('***** Ready to manually control *****')
            controller_manual_smart.get_logger().info('Use arrows from numerical keyboad to move template and enter/space to insert/retract the needle')
            break
    rclpy.spin(controller_manual_smart)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_manual_smart.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()