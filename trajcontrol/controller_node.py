import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from stage_control_interfaces.action import MoveStage


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        #Topics from Estimator node
        self.subscription_estimator = self.create_subscription(Image, '/needle/state/jacobian', self.jacobian_callback, 10)
        self.subscription_estimator  # prevent unused variable warning

        #Topics from UI node
        self.subscription_UI = self.create_subscription(PoseStamped, '/subject/state/target', self.target_callback, 10)
        self.subscription_UI  # prevent unused variable warning

        #Action client 
        #Check the correct action name and msg type from John's code
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        self.i=0


    # Get current Jacobian matrix from Estimator node
    def jacobian_callback(self, msg):
        J = CvBridge().imgmsg_to_cv2(msg)
        self.get_logger().info('Estimator: Jacobian received') # % np.array2string(J)

        ###################################
        #TODO: Calculate control output u = [x, y, z] 
        # x and z for stage
        # y for user input (insertion depth)
        ##################################
        # Send command to stage:
        self.send_cmd(0.05, 0.04)

    # Get current target point from UI node
    def target_callback(self, msg):
        target = msg.pose.position
        self.get_logger().info('Listening UI - Target: x=%f, y=%f, z=%f' % (target.x, target.y, target.z))

    # Send MoveStage action to Stage node (Goal)
    def send_cmd(self, x, z):

        goal_msg = MoveStage.Goal()
        goal_msg.x = x
        goal_msg.z = z
        goal_msg.eps = 0.0

        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        
        self.get_logger().info('Action stage - Sending goal request... Control u: x=%f, z=%f' % (goal_msg.x, goal_msg.z))      
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    # Get MoveStage action progress messages (Feedback)
    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.x))

    # Check if MoveStage action was accepted 
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Get MoveStage action finish message (Result)
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.x))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))


def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    rclpy.spin(controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
