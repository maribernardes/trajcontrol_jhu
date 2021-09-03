import rclpy
import numpy as np
import ament_index_python 

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from stage_control_interfaces.action import MoveStage

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from transforms3d.euler import euler2quat
from scipy.io import loadmat


class VirtualRobot(Node):

    def __init__(self):
        super().__init__('virtual_robot')
        
        #Declare node parameters
        self.declare_parameter('dataset', 'fbg_10') #Dataset file name

        #Published topics
        self.publisher_needle_pose = self.create_publisher(PoseStamped, '/stage/state/needle_pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_needlepose_callback)

        #Action server
        self._action_server = ActionServer(self, MoveStage, '/move_stage', execute_callback=self.execute_callback,\
            callback_group=ReentrantCallbackGroup(), goal_callback=self.goal_callback, cancel_callback=self.cancel_callback)

        #Load data from matlab file
        package_path = str(ament_index_python.get_package_share_path('trajcontrol'))
        file_name = self.get_parameter('dataset').get_parameter_value().string_value
        file_path = package_path + '/../../../../files/'+ file_name +'.mat'
        trial_data = loadmat(file_path, mat_dtype=True)
        
        self.needle_pose = trial_data['needle_pose'][0]
        self.time_stamp = trial_data['time_stamp'][0]
        self.i = 0
        
    # Publish current needle_pose
    def timer_needlepose_callback(self):

        # Use Aurora timestamp
        now = self.get_clock().now().to_msg()
        decimal = np.mod(self.time_stamp[self.i],1)
        now.nanosec = int(decimal*1e9)
        now.sec = int(self.time_stamp[self.i]-decimal)
        
        msg = PoseStamped()
        msg.header.stamp = now
        msg.header.frame_id = "stage"

        # Populate message with Z data from matlab file
        Z = self.needle_pose[self.i]
        msg.pose.position.x = float(Z[0])
        msg.pose.position.y = float(Z[1])
        msg.pose.position.z = float(Z[2])

        msg.pose.orientation = Quaternion(x=float(Z[3]), y=float(Z[4]), z=float(Z[5]), w=float(Z[6]))

        self.publisher_needle_pose.publish(msg)
        #self.get_logger().info('Publish - Needle pose %i: x=%f, y=%f, z=%f, q=[%f, %f, %f, %f] in %s frame'  % (self.i, msg.pose.position.x, \
        #    msg.pose.position.y, msg.pose.position.z,  msg.pose.orientation.x, \
        #    msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w, msg.header.frame_id))
        self.i += 1

    # Destroy de action server
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    # Accept or reject a client request to begin an action
    # This server allows multiple goals in parallel
    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    # Accept or reject a client request to cancel an action
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    # Execute a goal
    # This is a dummy action: the "goal" is to increment x from 0 to 4
    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = MoveStage.Feedback()
        feedback_msg.x = 0.0

        # Start executing the action
        for k in range(1, 5):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return MoveStage.Result()

            # Update control input
            feedback_msg.x = float(k)

            self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.x))

            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)
            
        goal_handle.succeed()

        # Populate result message
        result = MoveStage.Result()
        result.x = feedback_msg.x

        self.get_logger().info('Returning result: {0}'.format(result.x))

        return result


def main(args=None):
    rclpy.init(args=args)

    virtual_robot = VirtualRobot()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(virtual_robot, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    virtual_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()