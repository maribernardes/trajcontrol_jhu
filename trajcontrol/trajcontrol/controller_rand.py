import rclpy
import numpy as np
import math
import quaternion

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, PointStamped, Point, Quaternion, Pose
from smart_control_interfaces.action import MoveStage
from smart_control_interfaces.srv import GetPoint, GetPose
from trajcontrol_interfaces.srv import GetJacobian
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from functools import partial

#########################################################################
#
# Controller Rand
#
# Description:
# This node sends the robot to rand positions within the safe limit
# Useful for Jacobian estimation and validation
#
# Subscribes:   
# '/keyboard/key'               (std_msgs.msg.Int8)
# '/stage/state/guide_pose'     (geometry_msgs.msg.PointStamped)  - robot frame
#
# Action/service client:
# '/move_stage'             (smart_control_interfaces.action.MoveStage) - robot frame
# '/command'                (smart_control_interfaces.action.MoveStage) - robot frame
# 
#########################################################################

SAFE_LIMIT = 6.0        # Maximum control output delta from stage initial position [mm] (in each direction)
DEPTH_MARGIN = 1.5      # Final insertion length margin [mm]

class ControllerRand(Node):

    def __init__(self):
        super().__init__('controller_rand')

        #Declare node parameters
        self.declare_parameter('insertion_step', 10.0)          # Insertion step parameter
        
#### Stored variables ###################################################

        self.skin_entry = np.empty(shape=[0,3])     # Initial stage pose (relative to beginning of the random insertion)
        self.stage = np.empty(shape=[0,3])          # Current stage pose
        self.cmd = np.zeros((1,3))                  # Control output to the robot stage
        self.depth = 0.0                            # Total insertion depth (from skin_entry)
        self.Jc = np.zeros((5,3))                   # Jacobian matrix

        self.step = 0                               # Current insertion step
        self.robot_idle = False                     # Robot free to new command

#### Subscribed topics ###################################################

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PointStamped, '/stage/state/guide_pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

#### Action/service client ###################################################

        # Action client 
        # /stage/move Action client
        self.move_action_client = ActionClient(self, MoveStage, '/stage/move', callback_group=MutuallyExclusiveCallbackGroup())
        if not self.move_action_client.server_is_ready():
            self.get_logger().warn('/stage/move action not available, waiting...')
        while not self.move_action_client.wait_for_server(timeout_sec=5.0):
            pass
        # /planning/get_skin_entry Service client
        self.skin_entry_service_client = self.create_client(GetPoint, '/planning/get_skin_entry', callback_group=MutuallyExclusiveCallbackGroup())
        if not self.skin_entry_service_client.service_is_ready():
            self.get_logger().warn('/planning/get_skin_entry service not available, waiting...')
        while not self.skin_entry_service_client.wait_for_service(timeout_sec=5.0):
            pass   

        # /needle/get_tip and /stage/get_position should be in the same callbackgroup
        jacobian_inputs_callgroup = MutuallyExclusiveCallbackGroup() 
        # /stage/get_position Service client
        self.stage_position_service_client = self.create_client(GetPoint, '/stage/get_position', callback_group=jacobian_inputs_callgroup)
        if not self.stage_position_service_client.service_is_ready():
            self.get_logger().warn('/stage/get_position service not available, waiting...')
        while not self.stage_position_service_client.wait_for_service(timeout_sec=5.0):
            pass          
        # /needle/get_tip Service client
        self.tip_service_client = self.create_client(GetPose, '/needle/get_tip', callback_group=jacobian_inputs_callgroup)
        if not self.tip_service_client.service_is_ready():
            self.get_logger().warn('/needle/get_tip service not available, waiting...')
        while not self.tip_service_client.wait_for_service(timeout_sec=5.0):
            pass  
        # /estimator/get_jacobian Service client
        self.jacobian_service_client = self.create_client(GetJacobian, '/estimator/get_jacobian', callback_group=jacobian_inputs_callgroup)
        if not self.jacobian_service_client.service_is_ready():
            self.get_logger().warn('/estimator/get_jacobian service not available, waiting...')
        while not self.jacobian_service_client.wait_for_service(timeout_sec=5.0):
            pass  
        self.get_logger().info('Action and services available')


    #### Initialize variables ###################################################

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        self.insertion_step = self.get_parameter('insertion_step').get_parameter_value().double_value

        # Request planning (initialized only one - no update implemented yet)
        self.skin_entry = self.get_skin_entry()

        # # Define motion safety limits
        # limit_x = (float(self.skin_entry[0])-SAFE_LIMIT, float(self.skin_entry[0])+SAFE_LIMIT)
        # limit_z = (float(self.skin_entry[2])-SAFE_LIMIT, float(self.skin_entry[2])+SAFE_LIMIT)
        # self.limit = [limit_x, limit_z]

        # self.get_logger().info('Skin entry = %s' %self.skin_entry)
        # self.get_logger().info('Limit = %s' %self.limit)

#### Listening callbacks ###################################################

    # Get robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.point
        self.stage = np.array([robot.x, robot.y, robot.z])

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        if (msg.data == 32):
            if (self.robot_idle is True):
                self.robot_idle = False
                self.next_step()
            else:
                self.get_logger().info('Motion not available')

#### Service client functions ###################################################

    # Get skin_entry (blocking service request)
    def get_skin_entry(self):
        request = GetPoint.Request()
        future = self.skin_entry_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        skin_entry = future.result()
        if skin_entry.valid is True:
            return np.array([skin_entry.x, skin_entry.y, skin_entry.z])
        else:
            self.get_logger().error('Invalid skin_entry')
            return None
        
    # Update stage (non blocking service request)
    def update_stage(self):
        request = GetPoint.Request()
        future = self.stage_position_service_client.call_async(request)
        # When stage request done, do callback
        future.add_done_callback(partial(self.update_stage_callback))
        return future.result()

    # Update tip (non blocking service request)
    def update_tip(self):
        request = GetPose.Request()
        future = self.tip_service_client.call_async(request)
        # When tip request done, do callback
        future.add_done_callback(partial(self.update_tip_callback))
        return future.result()
    
    # Update jacobian (non blocking service request)
    def update_jacobian(self):
        request = GetJacobian.Request()
        request.base = Point(x=self.stage[0], y=self.stage[1], z=self.stage[2])
        Z_p = Point(x=self.tip[0], y=self.tip[1], z=self.tip[2])
        Z_q = Quaternion(w=self.tip[3], x=self.tip[4], y=0.0, z=0.0)
        request.tip = Pose(position=Z_p, orientation=Z_q)  
        future = self.jacobian_service_client.call_async(request)
        # When jacobian request done, do callback
        future.add_done_callback(partial(self.update_jacobian_callback))
        return future.result()

    # Update stage service response message (Response)
    def update_stage_callback(self, future):
        try:
            stage = future.result()
            if stage.valid is True:
                self.stage = np.array([stage.x, stage.y, stage.z])
                self.wait_stage = False
            else:
                self.get_logger().error('Invalid stage position')
        except Exception as e:
            self.get_logger().error('Stage call failed: %r' %(e,))
        if (self.wait_tip is False):
            self.move_step()

    # Update tip service response message (Response)
    def update_tip_callback(self, future):
        try:
            tip = future.result()
            if tip.valid is True:
                q = np.array([tip.qw, tip.qx, tip.qy, tip.qz])
                angles = get_angles(q)
                self.tip = np.array([tip.x, tip.y, tip.z, angles[0],angles[1]])
                self.wait_tip = False
            else:
                self.get_logger().error('Invalid tip pose')
        except Exception as e:
            self.get_logger().error('Tip call failed: %r' %(e,))
        if (self.wait_stage is False):
            self.move_step()

    # Update jacobian service response message (Response)
    def update_jacobian_callback(self, future):
        try:
            jacobian = future.result()
            self.Jc = np.asarray(CvBridge().imgmsg_to_cv2(jacobian.image_matrix))
            self.wait_jacobian = False
        except Exception as e:
            self.get_logger().error('Jacobian call failed: %r' %(e,))
        self.send_cmd()

#### Internal functions ###################################################

    def next_step(self):
        # Update jacobian request inputs (tip and base)
        # Responses trigger move_step
        self.wait_stage = True
        self.wait_tip = True
        self.update_stage()
        self.update_tip()

    def move_step(self):
        self.step += 1
        # Update Jacobian
        # Response triggers send_cmd (mpc_controller)
        self.wait_jacobian = True
        self.update_jacobian()
    
    # Send MoveStage action to robot
    def send_cmd(self):
        # Random select an input withing SAFE_LIMIT
        P = np.random.uniform(-SAFE_LIMIT, SAFE_LIMIT, 2)
        self.cmd = self.skin_entry + np.array([P[0], self.step*self.insertion_step, P[1]])
        self.get_logger().info('Step #%i: [%f, %f, %f] ' % (self.step, self.cmd[0], self.cmd[1], self.cmd[2]))

        # Prepare service message
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(self.cmd[0])
        goal_msg.y = float(self.cmd[1])
        goal_msg.z = float(self.cmd[2])
        goal_msg.eps = 0.0001
        self.robot_idle = False
        self.get_logger().info('Control: x=%f, y = %f, z=%f' % (self.cmd[0], self.cmd[1], self.cmd[2]))

        # MoveStage call - send command to stage
        self.move_action_client.wait_for_server() # Waiting for action server        
        self.send_goal_future = self.move_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.move_response_callback)

    # Check if MoveStage action was accepted 
    def move_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_move_result_callback)

    # Get MoveStage action finish message (Result)
    def get_move_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.stage = np.array([result.x, result.y, result.z])
            self.depth = self.stage[1] - self.skin_entry[1]
            self.get_logger().info('Goal succeeded! Result: %s, %f, %f' %(result.x, result.y, result.z))
            self.get_logger().info('Depth count: %.1fmm' % (self.depth))      
            self.get_logger().warn('Press SPACE for next step...')
            self.robot_idle = True
            # # Check if max depth reached
            # if (abs(self.depth-self.insertion_length) <= DEPTH_MARGIN): 
            #     self.robot_idle = False
            #     self.get_logger().info('ATTENTION: Depth margin reached! Please stop insertion')                
            # else:
            #     self.robot_idle = True
        else:
            self.get_logger().info('Goal failed with status: %s' %(result.status))

########################################################################

# Function: get_angles(q)
# DO: Get needle angles in horizontal and vertical plane
# Inputs: 
#   q: quaternion (numpy array [qw, qx, qy, qz])
# Output:
#   angles: angle vector (numpy array [horiz, vert])
def get_angles(q):

    #Define rotation quaternion
    q_tf= np.quaternion(q[0], q[1], q[2], q[3])

    #Get needle current Z axis vector (needle insertion direction)
    u_z = np.quaternion(0, 0, 0, 1)
    v = q_tf*u_z*q_tf.conj()

    #Angles are components in x (horizontal) and z(vertical)
    horiz = math.atan2(v.x, -v.y)
    vert = math.atan2(v.z, math.sqrt(v.x**2+v.y**2))
    return np.array([horiz, vert])

########################################################################

def main(args=None):
    rclpy.init(args=args)

    controller_rand = ControllerRand()

    executor = MultiThreadedExecutor()

    controller_rand.get_logger().warn('***** Ready to INSERT - Random insertion *****')
    controller_rand.get_logger().warn('Use SPACE to signal each insertion step')
    controller_rand.robot_idle = True

    try:
        rclpy.spin(controller_rand, executor=executor)
    except KeyboardInterrupt:
        controller_rand.get_logger().info('Keyboard interrupt, shutting down.\n')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_rand.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()