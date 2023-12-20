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
from .utils import get_angles

#########################################################################
#
# Controller Rand
#
# Description:
# This node sends the robot to rand positions within the safe limit
# Useful for Jacobian estimation and validation
#
# Subscribes:   
# '/keyboard/key'           (std_msgs.msg.Int8)
# '/stage/state/guide_pose' (geometry_msgs.msg.PointStamped)  - robot frame
#
# Service client:    
# '/estimator/get_jacobian' (trajcontrol_interfaces.srv.GetJacobian) - robot frame
#
# Action/service client:
# '/move_stage'             (smart_control_interfaces.action.MoveStage) - robot frame
# 
#########################################################################

SAFE_LIMIT = 4.0        # Maximum control output delta from stage initial position [mm] (in each direction)

class ControllerRand(Node):

    def __init__(self):
        super().__init__('controller_rand')

        #Declare node parameters
        self.declare_parameter('insertion_step', 5.0)    # Insertion step parameter
        self.declare_parameter('total_steps', 4)          # Insertion step parameter
        
#### Stored variables ###################################################

        self.skin_entry = np.empty(shape=[0,3])     # Initial stage pose (relative to beginning of the random insertion)

        self.stage = np.empty(shape=[0,3])          # Current stage pose        
        self.cmd = np.zeros((1,3))                  # Control output to the robot stage
        self.Jc = np.zeros((5,3))                   # Jacobian matrix
 
        self.step = 0                               # Current insertion step
        self.depth = 0.0                            # Total insertion depth (from skin_entry)

        self.wait_stage = False       # Flag to wait for tip value
        self.wait_tip = False         # Flag to wait for base value
        self.wait_jacobian = False    # Flag to wait for jacobian value
        self.robot_idle = False       # Robot status

#### Subscribed topics ###################################################

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning

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
        self.total_steps = self.get_parameter('total_steps').get_parameter_value().integer_value

        # Request planning (initialized only one - no update implemented yet)
        self.skin_entry = self.get_skin_entry()

        self.tip_pose = np.empty(shape=[0,7])    # Current tip pose   [x, y, z, qw, qx, qy, qz] in robot frame
        self.tip = np.empty(shape=[0,5])         # Current tip input  [x, y, z, angle_h, angle_v]
        self.stage = np.empty(shape=[0,3])       # Current base input [x, y, z]

        # # Define motion safety limits
        # limit_x = (float(self.skin_entry[0])-SAFE_LIMIT, float(self.skin_entry[0])+SAFE_LIMIT)
        # limit_z = (float(self.skin_entry[2])-SAFE_LIMIT, float(self.skin_entry[2])+SAFE_LIMIT)
        # self.limit = [limit_x, limit_z]

        # self.get_logger().info('Skin entry = %s' %self.skin_entry)
        # self.get_logger().info('Limit = %s' %self.limit)


#### Internal functions ###################################################

    def next_step(self):
        # Update jacobian request inputs (tip and base)
        # Responses trigger move_step
        self.wait_stage = True
        self.wait_tip = True
        self.update_stage()
        self.update_tip()

    def move_step(self):
        self.robot_idle = False
        self.wait_jacobian = True
        self.step += 1
        # Update Jacobian
        # Response triggers move_stage
        self.update_jacobian()

#### Listening callbacks ###################################################

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        if (msg.data == 32):
            if (self.robot_idle is True) and (self.skin_entry.size!=0):
                self.get_logger().warn('Starting step #%i...' %(self.step+1))  
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
                self.depth = self.stage[1] - self.skin_entry[1]
                self.wait_stage = False
            else:
                self.get_logger().error('Invalid stage position')
        except Exception as e:
            self.get_logger().error('Stage call failed: %r' %(e,))
        if (self.wait_stage is False) and (self.wait_tip is False) and (self.robot_idle is True):
            self.move_step()

    # Update tip service response message (Response)
    def update_tip_callback(self, future):
        try:
            tip = future.result()
            if tip.valid is True:
                q = np.array([tip.qw, tip.qx, tip.qy, tip.qz])
                angles = get_angles(q)
                self.tip = np.array([tip.x, tip.y, tip.z, angles[0],angles[1]])
                self.tip_pose = np.array([tip.x, tip.y, tip.z,tip.qw, tip.qx, tip.qy, tip.qz])
                self.wait_tip = False
            else:
                self.get_logger().error('Invalid tip pose')
        except Exception as e:
            self.get_logger().error('Tip call failed: %r' %(e,))
        if (self.wait_stage is False) and (self.wait_tip is False) and (self.robot_idle is True):
            self.move_step()

    # Update jacobian service response message (Response)
    def update_jacobian_callback(self, future):
        try:
            jacobian = future.result()
            self.Jc = np.asarray(CvBridge().imgmsg_to_cv2(jacobian.image_matrix))
            self.wait_jacobian = False
        except Exception as e:
            self.get_logger().error('Jacobian call failed: %r' %(e,))
        # Random select an input withing SAFE_LIMIT
        P = np.random.uniform(-SAFE_LIMIT, SAFE_LIMIT, 2)
        self.cmd = self.skin_entry + np.array([P[0], self.step*self.insertion_step, P[1]])
        self.get_logger().info('Step #%i: [%.4f, %.4f, %.4f] ' % (self.step, self.cmd[0], self.cmd[1], self.cmd[2]))
        self.move_stage(self.cmd[0], self.cmd[1], self.cmd[2])

#### Action client functions ###################################################

    # Send MoveStage action to robot
    def move_stage(self, x, y, z):
        # Send command to stage (in mm)
        self.robot_idle = False     # Set robot status to NOT IDLE
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.z = float(z)
        goal_msg.eps = 0.5 # in mm
        self.get_logger().debug('Move request: %.4f, %.4f, %.4f' % (x, y, z))
        # Wait for action server
        self.move_action_client.wait_for_server()        
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
            # Print values
            self.get_logger().info('\n****** STEP #%i ******\nInsertion depth: %f\nTip: (%f, %f, %f)\
                \nStage: (%f, %f, %f)\nCmd: (%f, %f, %f)\
                \nReached: (%.4f, %.4f, %.4f)\n*********************' \
                %(self.step, self.depth, \
                self.tip[0],self.tip[1], self.tip[2],\
                self.stage[0], self.stage[1], self.stage[2],\
                self.cmd[0], self.cmd[1], self.cmd[2], \
                result.x, result.y, result.z)
            )
            self.stage = np.array([result.x, result.y, result.z])
            if (self.step < self.total_steps):
                self.get_logger().warn('Hit SPACE for next step')
                self.robot_idle = True       # Set robot status to IDLE
            else:
                self.get_logger().warn('ATTENTION: Total number of steps reached! Please stop insertion')                
                self.get_logger().warn('Press Ctrl+C to finalize')                
                self.robot_idle = False      # Not taking more steps
        elif result.error_code == 1:
            self.get_logger().info('Move failed: TIMETOUT')

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