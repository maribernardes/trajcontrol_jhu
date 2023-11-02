import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PointStamped, Point
from stage_control_interfaces.action import MoveStage

class ControllerSequence(Node):

    def __init__(self):
        super().__init__('controller_sequence')

        #Topics from sensor processing node
        self.subscription_tip = self.create_subscription(PoseStamped, '/sensor/tip', self.tip_callback, 10)
        self.subscription_tip  # prevent unused variable warning

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PoseStamped, 'stage/state/pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        timer_period_control = 0.6  # seconds
        self.timer = self.create_timer(timer_period_control, self.timer_control_robot_callback)

        #Published topics
        self.publisher_control = self.create_publisher(PointStamped, '/stage/control/cmd', 10)

        #Action client 
        #Check the correct action name and msg type from John's code
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        #Targets
        self.target = np.array([[-90, -85, -80, -75, -70, -65, -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5,
                                 -5, -10, -15, -20, -25, -30, -35, -40, -45, -50, -55, -60, -65, -70, -75, -80, -85, -90,
                                 -90, -85, -80, -75, -70, -65, -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5,
                                 -5, -10, -15, -20, -25, -30, -35, -40, -45, -50, -55, -60, -65, -70, -75, -80, -85, -90,
                                 -90, -85, -80, -75, -70, -65, -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5,
                                 -5, -10, -15, -20, -25, -30, -35, -40, -45, -50, -55, -60, -65, -70, -75, -80, -85, -90,
                                 -90, -85, -80, -75, -70, -65, -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5,
                                 -5, -10, -15, -20, -25, -30, -35, -40, -45, -50, -55, -60, -65, -70, -75, -80, -85, -90,
                                 -90, -85, -80, -75, -70, -65, -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5
                                ],
                                [15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 
                                 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 
                                 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 
                                 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 
                                 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 
                                 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 
                                 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 
                                 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 
                                 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55 
                                ]])   

        # self.target = np.array([[0, 0, 0, 0, -20, -20, -20, -20, -40, -40, -40, -40, -60, -60, -60, -60, -80, -80, -80, -80],
        #                         [0, 2.5, 5, 7.5, 7.5, 5, 2.5, 0, 0, 2.5, 5, 7.5, 7.5, 5, 2.5, 0, 0, 2.5, 5, 7.5]])   

        # self.target = np.array([[-5, -10, -15, -20, -25, -30, 0],
        #                         [0, 0, 0, 0, 0, 0, 0]])   

        # Stored values
        self.stage_initial = np.empty(shape=[0,2])  # Stage home position
        self.stage = np.empty(shape=[0,2])          # Current stage pose
        self.tip = np.empty(shape=[0,7])            # Current needle tip x and z (from aurora)
        self.cmd = np.empty((0,2))                  # Control output to the robot stage
        self.robot_idle = False                     # Stage status
        self.next_target = 0                        # Next target number
        self.current_target = None                  # Sent target number
        self.sent_cmd = True                        # Current target number

    # Get robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.pose
        # Get robot initial position (only once)
        if (self.stage_initial.size == 0):
            self.stage_initial = np.array([robot.position.x*1000, robot.position.z*1000])
            self.get_logger().info('Robot initial position in (%f, %f)' %(self.stage_initial[0], self.stage_initial[1])) 
            self.robot_idle = True                  # Initialize robot status
        # Get current robot position
        self.stage = np.array([robot.position.x*1000, robot.position.z*1000])

    # Get current tip pose
    def tip_callback(self, msg):
        tip = msg.pose
        self.tip = np.array([tip.position.x, tip.position.y, tip.position.z, \
                                tip.orientation.w, tip.orientation.x, tip.orientation.y, tip.orientation.z])    

    # Timer to robot control
    def timer_control_robot_callback(self):
        # Robot available
        if (self.robot_idle == True):
                # Check if finished sequence
                if (self.next_target+1 >= self.target.shape[1]):
                    self.get_logger().info('Sequence successfully completed! Press Ctrl + C to quit')
                    input()
                else:
                    # Robot previoulsy sent a sequence point
                    if (self.current_target != None):
                        delta_time = self.get_clock().now() - self.time_begin
                        # Wait a little for Aurora to converge after reaching point
                        if (delta_time.nanoseconds*1e-9 >= 3.0):
                            # Select next target
                            self.next_target = self.next_target +1
                    if (self.current_target != self.next_target):
                        # Send command to robot
                        self.current_target = self.next_target
                        self.cmd = self.target[:,self.next_target]
                        self.send_cmd(self.cmd[0], self.cmd[1])          
                        self.get_logger().info('Sending robot to Target #%i = (%f, %f)' % (self.next_target, self.cmd[0], self.cmd[1]))
                        
                        # Publish control output
                        msg = PointStamped()
                        msg.point = Point(x=float(self.cmd[0]), z=float(self.cmd[1]))
                        msg.header.stamp = self.get_clock().now().to_msg()
                        self.publisher_control.publish(msg)

    # Send MoveStage action to Stage
    def send_cmd(self, x, z):
        # Send command to stage (convert mm to m)
        self.robot_idle = False
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(x*0.001)
        goal_msg.z = float(z*0.001)
        goal_msg.eps = 0.0001
        self.get_logger().debug('Send goal request... Control u: x=%f, z=%f' % ((goal_msg.x)*1000, (goal_msg.z)*1000))      

        self.action_client.wait_for_server() # Wait for action server 
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
            self.robot_idle = True                      #put robot in IDLE state
            self.time_begin = self.get_clock().now()    #begin timer for sending new command
            self.get_logger().info('Goal succeeded! Result: %f, %f' %(result.x*1000, result.z*1000))
            self.get_logger().info('Tip: (%f, %f, %f)'   % (self.tip[0], self.tip[1], self.tip[2]))
        else:
            self.get_logger().info('Goal failed with status: %s' %(result.status))


def main(args=None):
    rclpy.init(args=args)

    controller_sequence = ControllerSequence()

    rclpy.spin(controller_sequence)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_sequence.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
