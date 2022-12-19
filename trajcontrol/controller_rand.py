import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, PointStamped
from stage_control_interfaces.action import MoveStage
from trajcontrol.sensor_processing import INSERTION_STEP

SAFE_LIMIT = 6.0        # Maximum control output delta from stage initial position [mm] (in each direction)
DEPTH_MARGIN = 1.5      # Final insertion length margin [mm]

class ControllerRand(Node):

    def __init__(self):
        super().__init__('controller_rand')

        #Declare node parameters
        self.declare_parameter('insertion_length', -100.0) #Jacobian update parameter

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning
        
        #Topics from sensor processing node
        self.subscription_robot = self.create_subscription(PoseStamped, '/sensor/base', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Published topics
        self.publisher_control = self.create_publisher(PointStamped, '/stage/control/cmd', 10)

        #Action client 
        #Check the correct action name and msg type from John's code
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        # Stored values
        self.target = np.empty(shape=[0,3])         # Target 
        self.stage_initial = np.empty(shape=[0,3])     # Initial stage pose
        self.stage = np.empty(shape=[0,3])          # Current stage pose
        self.cmd = np.zeros((1,3))                  # Control output to the robot stage
        self.depth = 0.0
        self.robot_idle = True                      # Robot free to new command
        self.insertion_length = self.get_parameter('insertion_length').get_parameter_value().double_value
        self.step = 0

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        # Check if experiment is ready to begin (all topics received)
        # Only takes new inputs if robot finished previous action (robot IDLE)
        if (msg.data == 32) and (self.robot_idle == True) and (self.stage_initial.size != 0): # Hit SPACE and robot is free
            self.step = self.step + 1
            self.send_cmd()         # Calls routine to calculate and send new control signal

    # Get current base pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.pose
        self.stage = np.array([robot.position.x, robot.position.y, robot.position.z])
        self.depth = robot.position.y
        if (self.stage_initial.size == 0):
            self.stage_initial = np.array([robot.position.x, robot.position.y, robot.position.z])
            self.get_logger().info('Stage initial: [%f, %f, %f] ' % (self.stage_initial[0], self.stage_initial[1], self.stage_initial[2]))

    # Send MoveStage action to robot
    def send_cmd(self):

        # SELECT ONE OF THE OPTIONS:

        # 1)Generate random control output
        P = np.random.uniform(-SAFE_LIMIT, SAFE_LIMIT, 2)
        self.cmd = self.stage_initial + np.array([P[0], self.step*INSERTION_STEP, P[1]])

        # # 2) Generate pre-defined control output
        # P = np.array([[2, -2, -3, -2], #deltaX sequence
        #               [2, 2, -3, -2]]) #deltaZ sequence
        # self.cmd = self.stage_initial + np.array([P[0,self.step-1], self.step*INSERTION_STEP, P[1,self.step-1]])


        self.get_logger().info('Step #%i: [%f, %f, %f] ' % (self.step, self.cmd[0], self.cmd[1], self.cmd[2]))

        ## Keeping this just to be on the safe side
        # Limit control output to maximum SAFE_LIMIT[mm] around entry stage_initial
        self.cmd[0] = min(self.cmd[0], self.stage_initial[0]+SAFE_LIMIT)
        self.cmd[0] = max(self.cmd[0], self.stage_initial[0]-SAFE_LIMIT)
        self.cmd[2] = min(self.cmd[2], self.stage_initial[2]+SAFE_LIMIT)
        self.cmd[2] = max(self.cmd[2], self.stage_initial[2]-SAFE_LIMIT)

        # Test for stage limits
        self.cmd[0] = min(self.cmd[0], 0.0)
        self.cmd[0] = max(self.cmd[0], -90.0)
        self.cmd[2] = min(self.cmd[2], 90.0)
        self.cmd[2] = max(self.cmd[2], 0.0)

        # Prepare service message
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(self.cmd[0]*0.001)
        goal_msg.z = float(self.cmd[2]*0.001)
        goal_msg.eps = 0.0001
        self.robot_idle = False
        self.get_logger().info('Control: x=%f, z=%f' % (self.cmd[0], self.cmd[2]))

        # MoveStage call - send command to stage
        self.action_client.wait_for_server() # Waiting for action server        
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        # Publish control output
        msg = PointStamped()
        msg.point.x = float(self.cmd[0])
        msg.point.z = float(self.cmd[2])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_control.publish(msg)

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
            self.get_logger().info('Goal succeeded! Result: %f, %f' %(result.x*1000, result.z*1000))
            # Check if max depth reached
            if (abs(self.depth-self.insertion_length) <= DEPTH_MARGIN): 
                self.robot_idle = False
                self.get_logger().info('ATTENTION: Depth margin reached! Please stop insertion')                
            else:
                self.robot_idle = True
                self.get_logger().info('Depth count: %.1fmm. Please insert %.1fmm more, then hit SPACE' % (self.stage[1], INSERTION_STEP))      
        else:
            self.get_logger().info('Goal failed with status: %s' %(result.status))

def main(args=None):
    rclpy.init(args=args)

    controller_rand = ControllerRand()

    rclpy.spin(controller_rand)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_rand.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()