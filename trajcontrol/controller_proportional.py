import rclpy
import numpy as np
import math

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_msgs.msg import Int8
from stage_control_interfaces.action import MoveStage
from trajcontrol.sensor_processing import INSERTION_STEP

SAFE_LIMIT = 6.0    # Maximum control output delta from entry point [mm]
DEPTH_MARGIN = 1.5   # Final insertion length margin [mm]

#This version calculates the control output and sends to robot when a new Jacobian is published
class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        #Declare node parameters
        self.declare_parameter('K', 0.05) #Controller gain

        #Topics from estimator node
        self.subscription_estimator = self.create_subscription(PoseStamped, '/needle/state/jacobian', self.robot_callback, 10)
        self.subscription_estimator # prevent unused variable warning

        #Topics from sensor processing node
        self.subscription_tip = self.create_subscription(PoseStamped, '/sensor/tip', self.tip_callback, 10)
        self.subscription_tip  # prevent unused variable warning
        self.subscription_robot = self.create_subscription(PoseStamped, '/sensor/base', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Topics from UI (currently sensor processing node is doing the job)
        self.subscription_target = self.create_subscription(PointStamped, '/subject/state/target', self.target_callback, 10)
        self.subscription_target  # prevent unused variable warning

        #Topics from estimator node
        self.subscription_jacobian = self.create_subscription(Image, '/needle/state/jacobian', self.jacobian_callback, 10)
        self.subscription_jacobian  # prevent unused variable warning

        #Action client 
        #Check the correct action name and msg type from John's code
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        #Published topics
        self.publisher_control = self.create_publisher(PointStamped, '/stage/control/cmd', 10)

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        # Stored values
        self.tip = np.empty(shape=[0,3])            # Current needle tip x and z (from aurora)
        self.stage_initial = np.empty(shape=[0,3])  # Stage initial position
        self.stage = np.empty(shape=[0,3])          # Current stage position
        self.target = np.empty(shape=[0,3])         # Target 
        self.cmd = np.zeros(3)                      # Control output to the robot stage
        self.depth = 0.0                            # Current insertion depth
        self.robot_idle = True                      # Stage move action status
        self.J = np.zeros((5,3))
        self.K = self.get_parameter('K').get_parameter_value().double_value      # Get K value          
        self.get_logger().info('K for this trial: %f' %(self.K))

    # Get current Jacobian matrix from Estimator node
    def jacobian_callback(self, msg):
        self.J = np.asarray(CvBridge().imgmsg_to_cv2(msg))
        # Check if controller is ready to begin (topics received)
        # Only takes new inputs if robot finished previous action (robot IDLE)
        if (self.robot_idle == True) and (self.target.size != 0) and (self.tip.size != 0) and (self.stage_initial.size != 0): # Hit SPACE and robot is free
            self.send_cmd()         # Calls routine to calculate and send new control signal
 
    # Get current target (only once)
    def target_callback(self, msg):
        if (self.target.size == 0):
            target = msg.point
            self.target = np.array([target.x, target.y, target.z])

    # Get current tip pose
    def tip_callback(self, msg):
        tip = msg.pose
        self.tip = np.array([tip.position.x, tip.position.y, tip.position.z])

    # Get current base pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.pose
        self.stage = np.array([robot.position.x, robot.position.y, robot.position.z])
        self.depth = robot.position.y
        if (self.stage_initial.size == 0):
            self.stage_initial = np.array([robot.position.x, robot.position.y, robot.position.z])
            self.get_logger().info('Stage initial: (%f, %f, %f) ' % (self.stage_initial[0], self.stage_initial[1], self.stage_initial[2]))

    # Calculates control output and send MoveStage action to robot
    def send_cmd(self):
        # Reduce Jacobian to use angles as free DOF's for control
        Jc = self.J[0:3,:].copy()

        # Calculate base inputs delta
        error = self.tip - self.target                          # Calculate control error
        deltaU = self.K*np.matmul(np.linalg.pinv(Jc), error)    # Calculate control output
        self.cmd = self.stage + deltaU

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

        # TO MAKE INSERTIONS WITHOUT COMPENSATION (DELETE AFTER)
        # self.cmd[0] = self.stage_initial[0]
        # self.cmd[2] = self.stage_initial[2]

        self.get_logger().info('Applying trajectory compensation... DO NOT insert the needle now\nTip: (%f, %f, %f) \
            \nTarget: (%f, %f, %f) \nError: (%f, %f, %f) \nDeltaU: (%f, %f)  \nCmd: (%f, %f) \nStage: (%f, %f)' % (self.tip[0],\
            self.tip[1], self.tip[2], self.target[0], self.target[1], self.target[2], error[0], error[1], error[2],\
            deltaU[0], deltaU[2], self.cmd[0], self.cmd[2], self.stage[0], self.stage[2]))    

        # Send command to stage
        self.robot_idle = False
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(self.cmd[0]*0.001)
        goal_msg.z = float(self.cmd[2]*0.001)
        goal_msg.eps = 0.0001

        self.action_client.wait_for_server() # Waiting for action server        
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        # Publish cmd
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "stage"
        msg.point = Point(x=self.cmd[0], y=self.cmd[1], z=self.cmd[2])
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
            self.get_logger().info('Tip: (%f, %f, %f)'   % (self.tip[0], self.tip[1], self.tip[2]))
             # Check if max depth reached
            # if (abs(self.tip[1]-self.target[1]) <= DEPTH_MARGIN): 
            if (abs(self.depth) >= abs(self.target[1])):
                self.robot_idle = False
                self.get_logger().info('ATTENTION: Insertion depth reached! Please stop insertion')                
            else:
                self.robot_idle = True
                self.get_logger().info('Depth count: %.1fmm. Please insert %.1fmm more, then hit SPACE' % (self.stage[1], INSERTION_STEP))      
        else:
            self.get_logger().info('Goal failed with status: %s' %(result.status))


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
