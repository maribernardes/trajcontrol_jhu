import rclpy
import numpy as np

from std_msgs.msg import Int8
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped
from stage_control_interfaces.action import MoveStage

DELTA_MM = 1  #Increment for manual movement

class ControllerManual(Node):

    def __init__(self):
        super().__init__('controller_manual')

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PoseStamped, 'stage/state/pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Action client 
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        # Stored values
        self.stage_initial = np.empty(shape=[2,0])  # Stage home position
        self.stage = np.empty(shape=[2,0])          # Current stage pose
        self.robot_idle = False                     # Stage status

    # Get robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.pose
        # Stores robot initial position (only once)
        if (self.stage_initial.size == 0):
            self.stage_initial = np.array([robot.position.x*1000, robot.position.z*1000])
            self.get_logger().info('Initial position in (%f, %f)' %(self.stage_initial[0], self.stage_initial[1])) 
            self.robot_idle = True                  # Initialize robot status
        # Stores current robot position
        self.stage = np.array([robot.position.x*1000, robot.position.z*1000])

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        # Only takes new control input after converged to previous
        if (self.robot_idle == True):
            x = self.stage[0]
            z = self.stage[1]
            if (msg.data == 50): # move down
                z = z - DELTA_MM
            elif (msg.data == 52): # move left
                x = x - DELTA_MM
            elif (msg.data == 54): # move right
                x = x + DELTA_MM
            elif (msg.data == 56): # move up
                z = z + DELTA_MM
            # Send command to stage
            self.send_cmd(x, z)  

    # Send MoveStage action to Stage
    def send_cmd(self, x, z):
        # Send command to stage (convert mm to m)
        self.robot_idle = False     # Set robot status to NOT IDLE
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(x*0.001)
        goal_msg.z = float(z*0.001)
        goal_msg.eps = 0.0001
        self.get_logger().info('Send goal request... Control u: x=%f, z=%f' % (x, z))

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
            self.robot_idle = True       # Set robot status to IDLE
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.x*1000))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

def main(args=None):
    rclpy.init(args=args)
    controller_manual = ControllerManual()
    rclpy.spin(controller_manual)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_manual.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()