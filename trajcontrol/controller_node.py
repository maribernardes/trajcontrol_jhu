import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.action import ActionClient
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
        self._action_client = ActionClient(self, MoveStage, '/move_stage')

        self.i=0


    def send_goal(self, x, z):
        goal_msg = MoveStage.Goal()
        goal_msg.x = x
        goal_msg.z = z
        goal_msg.eps = 0.0

        self.get_logger().info('Action stage - Control u: x=%f, z=%f' % (goal_msg.x, goal_msg.z))
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)


    def jacobian_callback(self, msg):
        J = CvBridge().imgmsg_to_cv2(msg)
        self.get_logger().info('Listening estimator - Jacobian: %s' % np.array2string(J))

    def target_callback(self, msg):
        target = msg.pose.position
        self.get_logger().info('Listening UI - Target: x=%f, y=%f, z=%f' % (target.x, target.y, target.z))

    def timer_control_callback(self):
        ###################################
        #Calculate control output u = [x, y, z] 
        # x and z for stage
        # y for user input (insertion depth)
        u = np.asarray([1.0+self.i, 2.0+self.i, 3.0+self.i])
        ##################################
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "stage"
        msg.pose.position.x = u[0]
        msg.pose.position.y = u[1]
        msg.pose.position.z = u[2]

        self.publisher_control.publish(msg)
        self.get_logger().info('Publish - Control outputs: x=%f, y=%f, z=%f in %s frame'  % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.header.frame_id))



def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    controller_node.send_goal(1.5, 2.1)

    rclpy.spin(controller_node)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()