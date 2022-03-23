import rclpy
import message_filters
import numpy as np

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseArray
from std_msgs.msg import Int8
from stage_control_interfaces.action import MoveStage

class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        #Declare node parameters
        self.declare_parameter('K', -0.01) #Controller gain

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning
        self.listen_keyboard = False

        #Topics from UI
        self.subscription_entry_point = self.create_subscription(Point, '/subject/state/skin_entry', self.entry_callback, 10)
        self.subscription_entry_point  # prevent unused variable warning
        self.subscription_target = self.create_subscription(Point, '/subject/state/target', self.target_callback, 10)
        self.subscription_target  # prevent unused variable warning

        #Topics from sensorized needle
        self.subscription_sensor = self.create_subscription(PoseArray, '/needle/state/current_shape', self.tip_callback, 10)
        self.subscription_sensor # prevent unused variable warning

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PoseStamped, '/stage/state/needle_pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Published topics
        self.publisher_control = self.create_publisher(PointStamped, '/stage/control/cmd', 10)

        #Action client 
        #Check the correct action name and msg type from John's code
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        # Stored values
        self.entry_point = np.empty(shape=[3,0])    # Entry point x and z
        self.target = np.empty(shape=[3,0])         # Target x and z
        self.tip = np.empty(shape=[7,0])            # Current needle tip x and z (from aurora)
        self.stage = np.empty(shape=[7,0])          # Current stage pose
        self.cmd = np.zeros((2,1))                  # Control output to the robot stage
        self.robot_ready = True                     # Stage move action status

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        # Only takes new inputs if robot finished previous action
        if (msg.data == 32) and (self.robot_ready == True): # Hit SPACE and robot is free
            self.get_logger().info('DO NOT insert the needle now... robot correcting trajectory')
            self.send_cmd()                                 # Calls routine to calculate and send new control signal

    # Get current entry point
    def entry_callback(self, msg):
        entry_point = msg
        self.entry_point = np.array([entry_point.x, entry_point.y, entry_point.z])

    # Get current target
    def target_callback(self, msg):
        target = msg
        self.target = np.array([target.x, target.y, target.z])

    # Get current tip pose
    def tip_callback(self, msg):
        # From shape, get Z
        shape = msg.poses
        N = len(shape)
        self.tip = np.array([shape[N-1].position.x, shape[N-1].position.y, shape[N-1].position.z, \
            shape[N-1].orientation.w, shape[N-1].orientation.x, shape[N-1].orientation.y, shape[N-1].orientation.z])  #tip pose

        # Have a forced target for testing 
        self.target = np.array([self.tip[0], self.tip[1], self.tip[2]]) + np.array([1,1,1]) # REMOVE AFTER TESTS
        self.entry_point = np.array([1,1,1]) # REMOVE AFTER TESTS

    # Get current base pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.pose
        self.stage = np.array([robot.position.x, robot.position.y, robot.position.z, \
            robot.orientation.w, robot.orientation.x, robot.orientation.y, robot.orientation.z])

    # Send MoveStage action to Stage node (Goal)
    def send_cmd(self):
        self.get_logger().info('Sending goal')      
        # Send new control signal only tip and target available and if robot has finished previous action
        if (self.robot_ready == True) and (len(self.tip) != 0) and (len(self.target) != 0):
            # Calculate control output
            K = self.get_parameter('K').get_parameter_value().double_value          # Get K value          
            error = np.array([self.tip[0], self.tip[1], self.tip[2]])-self.target   # Calculate error between tip and target            
            self.cmd = np.array([self.stage[0], self.stage[2]]) + K*np.array([error[0], error[2]])

            # Limit control output to maximum +-5mm around entry point
            self.cmd[0] = min(self.cmd[0], self.entry_point[0]+5)
            self.cmd[1] = min(self.cmd[1], self.entry_point[2]+5)
            self.cmd[0] = max(self.cmd[0], self.entry_point[0]-5)
            self.cmd[1] = max(self.cmd[1], self.entry_point[2]-5)

            # Publish control output
            msg = PointStamped()
            msg.point.x = float(self.cmd[0]) - self.entry_point[0]
            msg.point.z = float(self.cmd[1]) - self.entry_point[2]
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_control.publish(msg)

            # Send command to stage
            self.robot_ready = False
            goal_msg = MoveStage.Goal()
            goal_msg.x = float(self.cmd[0])
            goal_msg.z = float(self.cmd[1])
            goal_msg.eps = 0.0

            self.get_logger().info('Waiting for action server...')
            self.action_client.wait_for_server()
            
            self.get_logger().info('Sending goal request... Control u: x=%f, z=%f' % (goal_msg.x, goal_msg.z))      
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
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.x))
            self.robot_ready = True
            self.get_logger().info('Please proceed with next insertion step and hit SPACE when done')
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))


def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    controller_node.get_logger().info('**********BEGIN EXPERIMENT********** \nStart first insertion step and hit SPACE when done')
    rclpy.spin(controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
